# device.py
# one emulated GigE Vision camera: a register file, a GVCP command
# handler, and the GVSP packetiser for its frames.
#
# no sockets here -- NightGigEServer owns those and calls into this.

import threading
import struct
import queue
import time

from NightEngine.GigE import protocol as P
from NightEngine.GigE import registers as R
from NightEngine.GigE import genapi
from NightEngine.NightVision import NightVisionCamera

TICK_FREQUENCY = 1_000_000          # 1 us ticks, matching the Genie Nano
DEFAULT_HEARTBEAT_MS = 3000
DEFAULT_PACKET_SIZE = 1500
RESEND_HISTORY = 4                  # blocks retained for PACKETRESEND


class NightGigEDevice:
    """an emulated GigE Vision device backed by a NightVisionCamera."""

    def __init__(self, camera, ip, mac, subnet_mask="255.255.0.0",
                 gateway="0.0.0.0", vendor="NightEngine",
                 model="NightEngineCam", version="1.0",
                 serial="NE00000001", user_name=None,
                 pixel_format="Mono8", frame_rate=15.0,
                 manufacturer_info="NightEngine emulated GigE Vision camera"):

        # duck-typed rather than isinstance, so the whole GVCP/GVSP stack
        # can be exercised in tests without an OpenGL context
        for attribute in ("name", "width", "height", "capture"):
            if not hasattr(camera, attribute):
                raise TypeError(f"camera is missing '{attribute}'; expected a "
                                f"NightVisionCamera-like object")

        self.camera = camera
        self.ip = ip
        self.mac = mac
        self.vendor = vendor
        self.model = model
        self.serial = serial
        self.user_name = user_name if user_name else camera.name

        self.width = camera.width
        self.height = camera.height

        self.lock = threading.RLock()

        # ---------------- control state ---------------- #

        self.controller = None          # (ip, port) of the primary application
        self.controller_seen = 0.0
        self.acquiring = False
        self.single_frame = False
        self._t0 = time.monotonic()

        # ---------------- stream state ---------------- #

        self.frames = queue.Queue(maxsize=2)
        self.block_id = 1               # 16-bit, must never be 0
        self.next_frame_due = 0.0
        self.stream_sender = None       # injected by the server
        self.source_port = 0            # injected by the server
        self._recent = {}               # block_id -> [packets] for resend
        self._recent_order = []
        self.frames_sent = 0

        # ---------------- register file ---------------- #

        self.registers = R.RegisterFile()
        self._init_registers(subnet_mask, gateway, version, manufacturer_info,
                             pixel_format, frame_rate)

    # ------------------------------------------------------------
    # setup
    # ------------------------------------------------------------

    def _init_registers(self, subnet_mask, gateway, version,
                        manufacturer_info, pixel_format, frame_rate):
        reg = self.registers

        # GEV 1.2. DeviceMode bit31 set = big-endian device, charset UTF-8,
        # exactly as the Genie Nano reports (0x80000001).
        reg.write_u32(R.VERSION, 0x00010002)
        reg.write_u32(R.DEVICE_MODE, 0x80000001)

        mac_bytes = R.mac_to_bytes(self.mac)
        reg.write_u32(R.MAC_HIGH, struct.unpack(">H", mac_bytes[0:2])[0])
        reg.write_u32(R.MAC_LOW, struct.unpack(">I", mac_bytes[2:6])[0])

        reg.write_u32(R.SUPPORTED_IP_CONFIG, 0x00000007)   # persistent|dhcp|lla
        reg.write_u32(R.CURRENT_IP_CONFIG, 0x00000006)     # dhcp|lla
        reg.write_u32(R.CURRENT_IP, R.ip_to_u32(self.ip))
        reg.write_u32(R.SUBNET_MASK, R.ip_to_u32(subnet_mask))
        reg.write_u32(R.GATEWAY, R.ip_to_u32(gateway))

        reg.write_string(R.MANUFACTURER_NAME, self.vendor, 32)
        reg.write_string(R.MODEL_NAME, self.model, 32)
        reg.write_string(R.DEVICE_VERSION, version, 32)
        reg.write_string(R.MANUFACTURER_INFO, manufacturer_info, 48)
        reg.write_string(R.SERIAL_NUMBER, self.serial, 16)
        reg.write_string(R.USER_DEFINED_NAME, self.user_name, 16)

        reg.write_u32(R.NUM_NETWORK_INTERFACES, 1)
        reg.write_u32(R.LINK_SPEED, 1000)
        reg.write_u32(R.NUM_MESSAGE_CHANNELS, 0)   # no events; keeps HALCON simple
        reg.write_u32(R.NUM_STREAM_CHANNELS, 1)
        reg.write_u32(R.SC_CAPABILITY, 0x80000000)
        reg.write_u32(R.MESSAGE_CHANNEL_CAPABILITY, 0x00000000)
        reg.write_u32(R.GVCP_CAPABILITY, R.GVCP_CAPABILITY_VALUE)
        reg.write_u32(R.HEARTBEAT_TIMEOUT, DEFAULT_HEARTBEAT_MS)
        reg.write_u32(R.TICK_FREQ_HIGH, 0)
        reg.write_u32(R.TICK_FREQ_LOW, TICK_FREQUENCY)
        reg.write_u32(R.CCP, 0)

        # stream channel 0 defaults: 1500 bytes with do-not-fragment,
        # the same as the Nano's 0x400005dc
        reg.write_u32(R.SCPS, R.SCPS_DO_NOT_FRAGMENT | DEFAULT_PACKET_SIZE)
        reg.write_u32(R.SCPD, 0)
        reg.write_u32(R.SCP, 0)
        reg.write_u32(R.SCDA, 0)
        reg.write_u32(R.SCC, 0x00000001)
        reg.write_u32(R.SCCFG, 0x00000001)

        # vendor / feature registers
        pf = {"Mono8": P.PFNC_MONO8, "RGB8": P.PFNC_RGB8,
              "BGR8": P.PFNC_BGR8}[pixel_format]
        reg.write_u32(R.REG_WIDTH, self.width)
        reg.write_u32(R.REG_HEIGHT, self.height)
        reg.write_u32(R.REG_SENSOR_WIDTH, self.width)
        reg.write_u32(R.REG_SENSOR_HEIGHT, self.height)
        reg.write_u32(R.REG_PIXEL_FORMAT, pf)
        reg.write_u32(R.REG_ACQUISITION_MODE, 0)
        reg.write_u32(R.REG_FRAME_PERIOD_US, int(1_000_000 / max(frame_rate, 0.1)))
        reg.write_u32(R.REG_EXPOSURE_TIME_US, 10000)
        reg.write_u32(R.REG_GAIN_RAW, 0)
        reg.write_u32(R.REG_OFFSET_X, 0)
        reg.write_u32(R.REG_OFFSET_Y, 0)
        reg.write_u32(R.REG_TRIGGER_MODE, 0)
        reg.write_u32(R.REG_PAYLOAD_SIZE, self.payload_size)

        # ------------- genapi xml + first url ------------- #

        xml_text = genapi.build_xml(vendor=self.vendor, model=self.model,
                                    version=version,
                                    width=self.width, height=self.height)
        self.xml_text = xml_text
        xml_name = f"{self.vendor}_{self.model}.zip".replace(" ", "")
        blob = genapi.zip_xml(xml_text, xml_name.replace(".zip", ".xml"))
        reg.set_xml(blob)
        # plain "Local:" form with hex address and hex length, exactly the
        # shape the Nano uses
        url = f"Local:{xml_name};{R.XML_BASE:x};{len(blob):x}"
        reg.write_string(R.FIRST_URL, url, 512)
        reg.write_string(R.SECOND_URL, "", 512)
        self.first_url = url

    # ------------------------------------------------------------
    # derived values
    # ------------------------------------------------------------

    @property
    def pixel_format(self):
        return self.registers.read_u32(R.REG_PIXEL_FORMAT)

    @property
    def payload_size(self):
        return self.width * self.height * P.pixel_format_bytes(self.pixel_format)

    @property
    def packet_size(self):
        return self.registers.read_u32(R.SCPS) & R.SCPS_SIZE_MASK

    @property
    def frame_period(self):
        return max(self.registers.read_u32(R.REG_FRAME_PERIOD_US), 1) / 1e6

    def stream_destination(self):
        """(ip, port) the consumer asked us to stream to, or None."""
        addr = self.registers.read_u32(R.SCDA)
        port = self.registers.read_u32(R.SCP) & 0xFFFF
        if not addr or not port:
            return None
        return (R.u32_to_ip(addr), port)

    def timestamp(self):
        return int((time.monotonic() - self._t0) * TICK_FREQUENCY)

    # ------------------------------------------------------------
    # GVCP
    # ------------------------------------------------------------

    def _controller_expired(self, now):
        timeout = self.registers.read_u32(R.HEARTBEAT_TIMEOUT) / 1000.0
        return self.controller and (now - self.controller_seen) > max(timeout, 0.5)

    def handle_gvcp(self, data, sender):
        """handles one GVCP command. returns the acknowledge bytes, or
        None if no reply should be sent."""
        cmd = P.parse_cmd(data)
        if cmd is None:
            return None

        with self.lock:
            now = time.monotonic()

            # drop a controller that stopped talking to us
            if self._controller_expired(now):
                self.controller = None
                self.registers.write_u32(R.CCP, 0)
                self._stop_acquisition()

            # any traffic from the controller refreshes the heartbeat.
            # Aravis only refreshes on a READREG of CCP; being more
            # permissive avoids dropping HALCON when it pauses at a
            # debugger breakpoint.
            if self.controller == sender:
                self.controller_seen = now

            has_control = (self.controller is None or self.controller == sender)

            if cmd.code == P.DISCOVERY_CMD:
                return P.pack_ack(P.DISCOVERY_ACK, cmd.req_id,
                                  self.registers.discovery_data())

            if cmd.code == P.READREG_CMD:
                return self._handle_readreg(cmd)

            if cmd.code == P.WRITEREG_CMD:
                return self._handle_writereg(cmd, sender, has_control, now)

            if cmd.code == P.READMEM_CMD:
                address, count = P.parse_readmem(cmd.payload)
                count = min(count, P.DATA_SIZE_MAX)
                chunk = self.registers.read(address, count)
                return P.pack_ack(P.READMEM_ACK, cmd.req_id,
                                  P.pack_readmem_ack(address, chunk))

            if cmd.code == P.WRITEMEM_CMD:
                if not has_control:
                    return P.pack_ack(P.WRITEMEM_ACK, cmd.req_id,
                                      status=P.STATUS_ACCESS_DENIED)
                address, blob = P.parse_writemem(cmd.payload)
                ok = self.registers.write(address, blob)
                if not ok:
                    return P.pack_ack(P.WRITEMEM_ACK, cmd.req_id,
                                      status=P.STATUS_WRITE_PROTECT)
                return P.pack_ack(P.WRITEMEM_ACK, cmd.req_id,
                                  P.pack_writemem_ack(len(blob)))

            if cmd.code == P.PACKETRESEND_CMD:
                return self._handle_resend(cmd)

            if cmd.code == P.FORCEIP_CMD:
                # acknowledged but ignored: our address is set by the host
                return P.pack_ack(P.FORCEIP_ACK, cmd.req_id)

            return P.pack_ack(cmd.code + 1, cmd.req_id,
                              status=P.STATUS_NOT_IMPLEMENTED)

    def _handle_readreg(self, cmd):
        addresses = P.parse_readreg(cmd.payload)
        if not addresses:
            return P.pack_ack(P.READREG_ACK, cmd.req_id,
                              status=P.STATUS_INVALID_PARAMETER)
        values = []
        for address in addresses:
            if address % 4 or address + 4 > R.MEMORY_SIZE:
                # the Nano answers 0x8003 for registers it does not
                # implement and HALCON tolerates it
                return P.pack_ack(P.READREG_ACK, cmd.req_id,
                                  status=P.STATUS_INVALID_ADDRESS)
            if address == R.SCSP:
                values.append(self.source_port)
            else:
                values.append(self.registers.read_u32(address))
        return P.pack_ack(P.READREG_ACK, cmd.req_id, P.pack_readreg_ack(values))

    def _handle_writereg(self, cmd, sender, has_control, now):
        pairs = P.parse_writereg(cmd.payload)
        if not pairs:
            return P.pack_ack(P.WRITEREG_ACK, cmd.req_id,
                              status=P.STATUS_INVALID_PARAMETER)

        # CCP is how a client claims control, so it must be writable
        # before control exists
        if not has_control and any(a != R.CCP for a, _ in pairs):
            return P.pack_ack(P.WRITEREG_ACK, cmd.req_id,
                              status=P.STATUS_ACCESS_DENIED)

        written = 0
        for address, value in pairs:
            if address not in R.WRITABLE:
                return P.pack_ack(P.WRITEREG_ACK, cmd.req_id,
                                  status=P.STATUS_INVALID_ADDRESS)
            self.registers.write_u32(address, value)
            written += 1
            self._apply_side_effects(address, value, sender, now)

        return P.pack_ack(P.WRITEREG_ACK, cmd.req_id, P.pack_writereg_ack(written))

    def _apply_side_effects(self, address, value, sender, now):
        if address == R.CCP:
            # HALCON writes 1, Aravis writes 2: any non-zero means a
            # controller now exists.
            if value:
                self.controller = sender
                self.controller_seen = now
            else:
                self.controller = None
                self._stop_acquisition()

        elif address == R.SCPS and (value & R.SCPS_FIRE_TEST_PACKET):
            # send exactly one datagram of the requested size, then clear
            # the bit. consumers use this to negotiate packet size.
            size = value & R.SCPS_SIZE_MASK
            self.registers.write_u32(R.SCPS, value & ~R.SCPS_FIRE_TEST_PACKET)
            if self.stream_sender:
                self.stream_sender(P.pack_gvsp_test_packet(size))

        elif address == R.REG_ACQUISITION_COMMAND:
            if value:
                self._start_acquisition()
            else:
                self._stop_acquisition()

        elif address == R.REG_PIXEL_FORMAT:
            self.registers.write_u32(R.REG_PAYLOAD_SIZE, self.payload_size)

        elif address == R.REG_TRIGGER_SOFTWARE and value:
            self.next_frame_due = 0.0       # emit one frame promptly

    def _handle_resend(self, cmd):
        extended = bool(cmd.flags & P.FLAG_ALLOW_BROADCAST_ACK)
        try:
            _channel, block, first, last = P.parse_packetresend(cmd.payload, extended)
        except struct.error:
            return P.pack_ack(P.PACKETRESEND_ACK, cmd.req_id,
                              status=P.STATUS_INVALID_PARAMETER)
        packets = self._recent.get(block)
        if not packets:
            return P.pack_ack(P.PACKETRESEND_ACK, cmd.req_id,
                              status=P.STATUS_PACKET_UNAVAILABLE)
        if self.stream_sender:
            for packet_id in range(first, min(last, len(packets) - 1) + 1):
                if 0 <= packet_id < len(packets):
                    self.stream_sender(packets[packet_id])
        return P.pack_ack(P.PACKETRESEND_ACK, cmd.req_id)

    # ------------------------------------------------------------
    # acquisition
    # ------------------------------------------------------------

    def _start_acquisition(self):
        self.single_frame = self.registers.read_u32(R.REG_ACQUISITION_MODE) == 1
        self.acquiring = True
        self.next_frame_due = 0.0
        self.registers.write_u32(R.REG_PAYLOAD_SIZE, self.payload_size)

    def _stop_acquisition(self):
        self.acquiring = False
        while not self.frames.empty():
            try:
                self.frames.get_nowait()
            except queue.Empty:
                break

    def wants_frame(self, now):
        """true when the render thread should capture for this device."""
        if not self.acquiring or self.stream_destination() is None:
            return False
        if self.frames.full():
            return False
        return now >= self.next_frame_due

    def capture(self, engine, now):
        """renders and enqueues one frame. render thread only."""
        frame = self.camera.capture(engine)
        pf = self.pixel_format
        if pf == P.PFNC_MONO8:
            payload = NightVisionCamera.to_mono8(frame)
        elif pf == P.PFNC_RGB8:
            payload = NightVisionCamera.to_rgb8(frame)
        else:
            payload = frame
        try:
            self.frames.put_nowait(payload.tobytes())
        except queue.Full:
            return False
        self.next_frame_due = now + self.frame_period
        if self.single_frame:
            self.acquiring = False
        return True

    # ------------------------------------------------------------
    # GVSP
    # ------------------------------------------------------------

    def build_packets(self, payload):
        """splits one frame into leader + payload packets + trailer."""
        with self.lock:
            block = self.block_id
            self.block_id = self.block_id + 1 if self.block_id < 0xFFFF else 1
            pixel_format = self.pixel_format
            per_packet = P.payload_per_packet(self.packet_size)

        packets = [P.pack_gvsp_leader(block, self.timestamp(), pixel_format,
                                      self.width, self.height)]
        packet_id = 1
        for offset in range(0, len(payload), per_packet):
            packets.append(P.pack_gvsp_payload(block, packet_id,
                                               payload[offset:offset + per_packet]))
            packet_id += 1
        packets.append(P.pack_gvsp_trailer(block, packet_id, self.height))

        with self.lock:
            self._recent[block] = packets
            self._recent_order.append(block)
            while len(self._recent_order) > RESEND_HISTORY:
                self._recent.pop(self._recent_order.pop(0), None)
            self.frames_sent += 1

        return packets

    def info(self):
        return {"name": self.camera.name, "ip": self.ip, "mac": self.mac,
                "model": self.model, "serial": self.serial,
                "width": self.width, "height": self.height,
                "pixel_format": f"0x{self.pixel_format:08x}",
                "payload_size": self.payload_size,
                "packet_size": self.packet_size,
                "acquiring": self.acquiring,
                "frames_sent": self.frames_sent,
                "xml_bytes": self.registers.xml_size,
                "first_url": self.first_url}

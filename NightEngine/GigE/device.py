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
FRAME_QUEUE_DEPTH = 16              # blocks buffered when transfer is stopped


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

        # 16 rather than 2: under UserControlled the client may hold the
        # transfer stopped for a while, and captured blocks have to buffer
        # somewhere. A long stop still overflows -- see capture().
        self.frames = queue.Queue(maxsize=FRAME_QUEUE_DEPTH)
        self.transfer_active = True     # meaningless unless UserControlled
        self.transfer_credit = 0        # blocks TransferStart still owes
        self.block_id = 1               # 16-bit, must never be 0
        self.next_frame_due = 0.0
        self.trigger_pending = False    # armed by TriggerSoftware or a line edge
        self.log = None                 # set by the server when diagnostics are on
        self._last_block_reason = None
        self.stream_sender = None       # injected by the server
        self.source_port = 0            # injected by the server
        self._recent = {}               # block_id -> [packets] for resend
        self._recent_order = []
        self.frames_sent = 0

        # ---------------- physical I/O lines ---------------- #

        # the hardware pins. `level` is what the outside world drives; the
        # level the trigger logic sees is `level XOR inverter`, so
        # LineInverter behaves the way it does in silicon.
        self.lines = {index: {"level": 0, "inverter": 0, "mode": R.LINE_INPUT}
                      for index in range(1, R.LINE_COUNT + 1)}

        # ---------------- register file ---------------- #

        self.registers = R.RegisterFile()
        # LineMode/LineInverter/LineStatus are windows onto whichever line
        # LineSelector currently names, so they resolve through these
        # rather than living in the flat register file. SCSP is here too:
        # it used to be a lone special case inside _handle_readreg.
        self._read_hooks = {
            R.SCSP: lambda: self.source_port,
            R.REG_LINE_MODE: lambda: self._selected_line()["mode"],
            R.REG_LINE_INVERTER: lambda: self._selected_line()["inverter"],
            R.REG_LINE_STATUS: lambda: self.line_level(self.selected_line_index),
            R.REG_LINE_STATUS_ALL: self._line_status_all,
            R.REG_TRANSFER_QUEUE_COUNT: lambda: self.frames.qsize(),
        }
        self._write_hooks = {
            R.REG_LINE_MODE: lambda value: self._set_line_field("mode", value),
            R.REG_LINE_INVERTER:
                lambda value: self._set_line_field("inverter", 1 if value else 0),
        }
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
        reg.write_u32(R.REG_TRIGGER_SELECTOR, 0)      # FrameStart
        reg.write_u32(R.REG_TRIGGER_SOURCE, 0)        # Software
        reg.write_u32(R.REG_TRIGGER_ACTIVATION, 0)    # RisingEdge
        reg.write_u32(R.REG_TRIGGER_DELAY_US, 0)
        reg.write_u32(R.REG_GAIN_SELECTOR, 0)         # All
        reg.write_u32(R.REG_GAIN_AUTO, 0)             # Off
        reg.write_u32(R.REG_EXPOSURE_MODE, 0)         # Timed
        reg.write_u32(R.REG_EXPOSURE_AUTO, 0)         # Off
        reg.write_u32(R.REG_LINE_SELECTOR, 1)         # Line1
        reg.write_u32(R.REG_TRANSFER_CONTROL_MODE, R.TRANSFER_CONTROL_BASIC)
        reg.write_u32(R.REG_TRANSFER_BLOCK_COUNT, 1)
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

    @property
    def halcon_name(self):
        """the device name HALCON forms for open_framegrabber: the MAC with
        its separators stripped, then the vendor and model with their
        spaces stripped, joined by underscores -- e.g.
        1c0faf7ae6bc_LucidVisionLabs_TRI050SM. Scripts hardcode this, so
        an example that prints it saves guessing at the convention."""
        return "_".join((self.mac.replace(":", "").replace("-", "").lower(),
                         self.vendor.replace(" ", ""),
                         self.model.replace(" ", "")))

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
    # physical I/O lines
    # ------------------------------------------------------------

    @property
    def selected_line_index(self):
        """the line LineSelector currently names, clamped to one that
        exists so a stray selector write cannot raise."""
        index = self.registers.read_u32(R.REG_LINE_SELECTOR)
        return index if index in self.lines else 1

    def _selected_line(self):
        return self.lines[self.selected_line_index]

    def _set_line_field(self, field, value):
        self.lines[self.selected_line_index][field] = value

    def line_level(self, index):
        """the *effective* level of a line: what the trigger logic sees
        once LineInverter has been applied."""
        line = self.lines.get(index)
        if line is None:
            return 0
        return line["level"] ^ line["inverter"]

    def _line_status_all(self):
        bits = 0
        for index in self.lines:
            bits |= self.line_level(index) << (index - 1)
        return bits

    def set_line(self, index, level):
        """drives a physical input pin. This is the emulated wire from the
        lighting controller: callers hand us a *level*, and we do our own
        edge detection against our own TriggerActivation, exactly as real
        hardware does. Safe to call from any thread."""
        with self.lock:
            line = self.lines.get(index)
            if line is None:
                return
            level = 1 if level else 0
            if line["level"] == level:
                return
            previous = self.line_level(index)
            line["level"] = level
            current = self.line_level(index)
            if previous != current:
                self._line_edge(index, previous, current)

    def _line_edge(self, index, previous, current):
        """called on every change of a line's effective level. Arms a
        frame when this line is the configured trigger source and the edge
        matches TriggerActivation."""
        if not self.triggered or self.trigger_line != index:
            return
        activation = self.trigger_activation
        if activation in R.TRIGGER_ACTIVATION_LEVEL_MODES:
            # level modes gate a free-run rather than arming one frame;
            # wants_frame reads the level directly, so nothing to arm here.
            self._note(f"Line{index} -> {current} (level-gated)")
            return
        rising = current > previous
        wanted = {R.TRIGGER_ACTIVATION_RISING_EDGE: rising,
                  R.TRIGGER_ACTIVATION_FALLING_EDGE: not rising,
                  R.TRIGGER_ACTIVATION_ANY_EDGE: True}.get(activation, rising)
        if not wanted:
            return
        self.trigger_pending = True
        self.next_frame_due = 0.0       # emit the triggered frame promptly
        self._note(f"triggered by Line{index} "
                   f"{'rising' if rising else 'falling'} edge")

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
                self._note(f"heartbeat expired after "
                           f"{now - self.controller_seen:.1f}s -- dropping "
                           f"control and stopping acquisition")
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
            hook = self._read_hooks.get(address)
            values.append(hook() if hook else self.registers.read_u32(address))
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
            hook = self._write_hooks.get(address)
            if hook:
                hook(value)
            self._apply_side_effects(address, value, sender, now)

        return P.pack_ack(P.WRITEREG_ACK, cmd.req_id, P.pack_writereg_ack(written))

    def _apply_side_effects(self, address, value, sender, now):
        if address == R.CCP:
            # HALCON writes 1, Aravis writes 2: any non-zero means a
            # controller now exists.
            if value:
                self.controller = sender
                self.controller_seen = now
                self._note(f"control taken by {sender[0]}:{sender[1]}")
            else:
                self._note("control released")
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

        elif address == R.REG_TRANSFER_START and value:
            self._transfer_start()

        elif address == R.REG_TRANSFER_STOP and value:
            self._transfer_stop()

        elif address == R.REG_TRANSFER_ABORT and value:
            self._transfer_abort()

        elif address == R.REG_TRIGGER_SOFTWARE and value:
            # only the Software source responds to TriggerSoftware. A
            # device wired to Line2 that still fired on a software command
            # would silently hide a mis-wired rig, which is exactly the
            # failure this emulator exists to reproduce.
            if self.trigger_line is None:
                self.trigger_pending = True
                self.next_frame_due = 0.0   # emit one frame promptly
            else:
                self._note(f"TriggerSoftware ignored: TriggerSource is "
                           f"Line{self.trigger_line}")

    # ------------------------------------------------------------
    # transfer control
    # ------------------------------------------------------------

    @property
    def transfer_control_mode(self):
        return self.registers.read_u32(R.REG_TRANSFER_CONTROL_MODE)

    @property
    def user_controlled_transfer(self):
        return self.transfer_control_mode == R.TRANSFER_CONTROL_USER_CONTROLLED

    def transfer_ready(self):
        """whether the sender may release a block right now. Basic and
        Automatic always may; UserControlled only while a TransferStart
        still has credit left."""
        with self.lock:
            if not self.user_controlled_transfer:
                return True
            return self.transfer_active and self.transfer_credit > 0

    def transfer_consume(self):
        """called by the sender once it has taken a block."""
        with self.lock:
            if self.user_controlled_transfer and self.transfer_credit > 0:
                self.transfer_credit -= 1

    def _transfer_start(self):
        count = max(self.registers.read_u32(R.REG_TRANSFER_BLOCK_COUNT), 1)
        self.transfer_active = True
        self.transfer_credit += count
        self._note(f"TransferStart: releasing up to {count} block(s) "
                   f"({self.frames.qsize()} queued)")

    def _transfer_stop(self):
        self.transfer_active = False
        self.transfer_credit = 0
        self._note(f"TransferStop: {self.frames.qsize()} block(s) held")

    def _transfer_abort(self):
        dropped = self.frames.qsize()
        self.transfer_active = False
        self.transfer_credit = 0
        self._drain_frames()
        self._note(f"TransferAbort: discarded {dropped} queued block(s)")

    def _drain_frames(self):
        while not self.frames.empty():
            try:
                self.frames.get_nowait()
            except queue.Empty:
                break

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

    def _note(self, message):
        """diagnostic line, only emitted when the server enables logging."""
        if self.log:
            self.log(f"[{self.camera.name}] {message}")

    @property
    def triggered(self):
        """true when TriggerMode is On for the FrameStart trigger, i.e.
        frames are emitted on a trigger rather than free-run."""
        return self.registers.read_u32(R.REG_TRIGGER_MODE) == 1

    @property
    def trigger_source(self):
        return self.registers.read_u32(R.REG_TRIGGER_SOURCE)

    @property
    def trigger_activation(self):
        return self.registers.read_u32(R.REG_TRIGGER_ACTIVATION)

    @property
    def trigger_line(self):
        """the line index TriggerSource names, or None for Software.
        Source value N means LineN, which is why LineSelector uses the
        same numbering."""
        source = self.trigger_source
        return source if source in self.lines else None

    def _level_gate_open(self):
        """for the level-triggered activations: whether the configured
        line currently sits at the level that allows capture."""
        index = self.trigger_line
        if index is None:
            return False
        wanted = 1 if self.trigger_activation == R.TRIGGER_ACTIVATION_LEVEL_HIGH else 0
        return self.line_level(index) == wanted

    def _start_acquisition(self):
        self.single_frame = self.registers.read_u32(R.REG_ACQUISITION_MODE) == 1
        self.acquiring = True
        self.next_frame_due = 0.0
        self.trigger_pending = False
        # under UserControlled the client owes us a TransferStart before
        # anything leaves the device; the other modes flow immediately.
        self.transfer_active = not self.user_controlled_transfer
        self.transfer_credit = 0
        self.registers.write_u32(R.REG_PAYLOAD_SIZE, self.payload_size)
        self._last_block_reason = None
        self._note(f"AcquisitionStart: mode="
                   f"{'SingleFrame' if self.single_frame else 'Continuous'}, "
                   f"trigger={'On' if self.triggered else 'Off'}, "
                   f"transfer={self._TRANSFER_NAMES.get(self.transfer_control_mode)}, "
                   f"payload={self.payload_size} B, "
                   f"destination={self.stream_destination()}")

    _TRANSFER_NAMES = {R.TRANSFER_CONTROL_BASIC: "Basic",
                       R.TRANSFER_CONTROL_AUTOMATIC: "Automatic",
                       R.TRANSFER_CONTROL_USER_CONTROLLED: "UserControlled"}

    def _stop_acquisition(self):
        if self.acquiring:
            self._note("AcquisitionStop")
        self.acquiring = False
        self.trigger_pending = False
        self.transfer_credit = 0
        self._drain_frames()

    # a normal, constantly-toggling condition; logging it would drown the log
    _QUIET_REASON = "free-running, next frame not due"

    _ACTIVATION_NAMES = {R.TRIGGER_ACTIVATION_RISING_EDGE: "rising edge",
                         R.TRIGGER_ACTIVATION_FALLING_EDGE: "falling edge",
                         R.TRIGGER_ACTIVATION_ANY_EDGE: "any edge"}

    def _waiting_reason(self):
        """names the signal we are actually waiting on, so a consumer that
        blocks forever says which wire is silent."""
        index = self.trigger_line
        if index is None:
            return "armed, waiting for TriggerSoftware"
        edge = self._ACTIVATION_NAMES.get(self.trigger_activation, "edge")
        return f"armed, waiting for a Line{index} {edge}"

    def wants_frame(self, now):
        """true when the render thread should capture for this device.

        Also reports WHY not, when logging is on. This previously returned a
        bare False for four different reasons, which is precisely what made a
        consumer receiving no images impossible to diagnose."""
        if not self.acquiring:
            reason = "not acquiring (no AcquisitionStart)"
        elif self.stream_destination() is None:
            reason = "no stream destination (SCDA/SCP are zero)"
        elif self.frames.full():
            reason = "frame queue full (consumer is not draining the stream)"
        elif self.triggered and self.trigger_activation in R.TRIGGER_ACTIVATION_LEVEL_MODES:
            # a gated free-run: capture repeatedly for as long as the line
            # holds the active level, rather than once per edge.
            if not self._level_gate_open():
                reason = (f"level-gated, Line{self.trigger_line} not at the "
                          f"active level")
            else:
                reason = None if now >= self.next_frame_due else self._QUIET_REASON
        elif self.triggered:
            reason = None if self.trigger_pending else self._waiting_reason()
        else:
            reason = None if now >= self.next_frame_due else self._QUIET_REASON

        if reason != self._QUIET_REASON and reason != self._last_block_reason:
            if reason:
                self._note(f"capture blocked: {reason}")
            elif self._last_block_reason is not None:
                self._note("capture ready")
            self._last_block_reason = reason
        return reason is None

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
            # a transfer held stopped for long enough will overflow. Drop
            # the OLDEST block rather than the newest: a consumer that
            # resumes wants the most recent view of the scene, and a
            # silently-stale image is worse than a missing one.
            try:
                self.frames.get_nowait()
                self.frames.put_nowait(payload.tobytes())
                self._note("frame queue overflowed; dropped the oldest block")
            except (queue.Empty, queue.Full):
                return False
        self.next_frame_due = now + self.frame_period
        self.trigger_pending = False
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
                "halcon_name": self.halcon_name,
                "width": self.width, "height": self.height,
                "pixel_format": f"0x{self.pixel_format:08x}",
                "payload_size": self.payload_size,
                "packet_size": self.packet_size,
                "acquiring": self.acquiring,
                "frames_sent": self.frames_sent,
                "xml_bytes": self.registers.xml_size,
                "first_url": self.first_url}

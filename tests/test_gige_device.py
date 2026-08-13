# test_gige_device.py
# end-to-end test of the emulated GigE Vision device over real UDP
# sockets on loopback, driven by a from-scratch GVCP/GVSP client.
#
# uses a stub camera, so the whole control + streaming + GenApi stack is
# exercised with no OpenGL context and no HALCON.
#
# run:  python tests/test_gige_device.py

import os
import sys
import io
import time
import socket
import struct
import zipfile
import threading
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from NightEngine.GigE import protocol as P
from NightEngine.GigE import registers as R
from NightEngine.GigE.device import NightGigEDevice
from NightEngine.GigE.server import NightGigEServer

DEVICE_IP = "127.0.0.1"
WIDTH, HEIGHT = 64, 48
NS = "{http://www.genicam.org/GenApi/Version_1_1}"


# ------------------------------------------------------------
# a camera stand-in: no OpenGL, deterministic frames
# ------------------------------------------------------------

class StubCamera:
    def __init__(self, name="stub", width=WIDTH, height=HEIGHT):
        self.name = name
        self.width = width
        self.height = height
        self.captures = 0

    def frame_bgr(self):
        """deterministic pattern: horizontal gradient plus a bright block."""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        gradient = (np.arange(self.width) * 255 // max(self.width - 1, 1)).astype(np.uint8)
        frame[:, :, 0] = gradient
        frame[:, :, 1] = gradient
        frame[:, :, 2] = gradient
        frame[4:12, 4:12] = 255
        return frame

    def capture(self, engine):
        self.captures += 1
        return self.frame_bgr()


# ------------------------------------------------------------
# minimal GVCP / GVSP client
# ------------------------------------------------------------

class Client:
    def __init__(self, device_ip):
        self.device = (device_ip, P.GVCP_PORT)
        self.control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control.bind((device_ip, 0))
        self.control.settimeout(3.0)
        self.stream = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # a frame arrives as a burst of hundreds of datagrams. the default
        # ~64 KB receive buffer overflows and the kernel silently drops
        # packets, which is why real consumers enlarge it (and why
        # HALCON ships a filter driver). without this a 640x480 mono
        # frame loses ~28 of its 212 packets.
        try:
            self.stream.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 << 20)
        except OSError:
            pass
        self.stream.bind((device_ip, 0))
        self.stream.settimeout(5.0)
        self.req_id = 0

    def close(self):
        self.control.close()
        self.stream.close()

    @property
    def stream_port(self):
        return self.stream.getsockname()[1]

    def command(self, code, payload=b"", flags=P.FLAG_ACK_REQUIRED):
        self.req_id = (self.req_id % 0xFFFE) + 1
        self.control.sendto(P.pack_cmd(code, self.req_id, payload, flags), self.device)
        data, _ = self.control.recvfrom(65535)
        status, ack_code, ack_id, body = P.parse_ack(data)
        assert ack_id == self.req_id, f"ack id {ack_id} != req id {self.req_id}"
        return status, ack_code, body

    def discover(self):
        status, code, body = self.command(P.DISCOVERY_CMD, b"",
                                          flags=P.FLAG_ACK_REQUIRED |
                                          P.FLAG_ALLOW_BROADCAST_ACK)
        assert status == 0 and code == P.DISCOVERY_ACK
        return body

    def read_reg(self, *addresses):
        status, code, body = self.command(P.READREG_CMD, P.pack_readreg(list(addresses)))
        if status != 0:
            raise RuntimeError(f"READREG failed: status=0x{status:04x}")
        values = list(struct.unpack(f">{len(addresses)}I", body))
        return values[0] if len(addresses) == 1 else values

    def write_reg(self, address, value):
        status, code, body = self.command(P.WRITEREG_CMD,
                                          P.pack_writereg([(address, value)]))
        if status != 0:
            raise RuntimeError(f"WRITEREG 0x{address:x} failed: status=0x{status:04x}")
        return struct.unpack(">I", body)[0]

    def read_mem(self, address, count):
        status, code, body = self.command(P.READMEM_CMD, P.pack_readmem(address, count))
        if status != 0:
            raise RuntimeError(f"READMEM failed: status=0x{status:04x}")
        return body[4:]

    def read_string(self, address, length):
        return self.read_mem(address, length).split(b"\x00")[0].decode()

    def fetch_xml(self):
        """follows the First URL exactly as a real consumer does."""
        url = self.read_string(R.FIRST_URL, 512)
        assert url.startswith("Local:"), f"unexpected url scheme: {url}"
        name, address_hex, length_hex = url[len("Local:"):].split(";")
        address, length = int(address_hex, 16), int(length_hex, 16)

        blob = b""
        while len(blob) < length:
            chunk = self.read_mem(address + len(blob),
                                  min(P.DATA_SIZE_MAX, length - len(blob)))
            blob += chunk
        blob = blob[:length]

        if name.endswith(".zip"):
            with zipfile.ZipFile(io.BytesIO(blob)) as archive:
                members = archive.namelist()
                assert len(members) == 1, f"expected one member, got {members}"
                return archive.read(members[0]).decode("utf-8"), name, length
        return blob.decode("utf-8"), name, length

    def receive_frame(self, timeout=6.0):
        """reassembles one GVSP block: leader, payload packets, trailer."""
        deadline = time.time() + timeout
        leader = None
        chunks = {}
        while time.time() < deadline:
            try:
                data, _ = self.stream.recvfrom(65535)
            except socket.timeout:
                break
            packet = P.parse_gvsp(data)
            if packet is None:
                continue
            kind = packet["content_type"]
            if kind == P.GVSP_CONTENT_LEADER:
                body = packet["payload"]
                leader = {
                    "block_id": packet["block_id"],
                    "payload_type": struct.unpack_from(">H", body, 2)[0],
                    "pixel_format": struct.unpack_from(">I", body, 12)[0],
                    "width": struct.unpack_from(">I", body, 16)[0],
                    "height": struct.unpack_from(">I", body, 20)[0],
                }
                chunks = {}
            elif kind == P.GVSP_CONTENT_PAYLOAD and leader:
                chunks[packet["packet_id"]] = packet["payload"]
            elif kind == P.GVSP_CONTENT_TRAILER and leader:
                body = packet["payload"]
                size_y = struct.unpack_from(">I", body, 4)[0]
                ordered = [chunks[k] for k in sorted(chunks)]
                return leader, b"".join(ordered), size_y, packet["packet_id"]
        raise TimeoutError("no complete GVSP block received")


# ------------------------------------------------------------
# fixture: device + server + a fake render loop
# ------------------------------------------------------------

class Harness:
    def __init__(self, bind_any=False):
        self.camera = StubCamera()
        self.device = NightGigEDevice(camera=self.camera, ip=DEVICE_IP,
                                      mac="02:00:00:00:00:01",
                                      subnet_mask="255.0.0.0",
                                      model="NightEngineCam",
                                      serial="NE0001",
                                      user_name="StubCam",
                                      pixel_format="Mono8",
                                      frame_rate=30.0)
        # bind_broadcast off: on Windows binding a broadcast address is
        # rejected outright, and this test drives unicast discovery
        self.server = NightGigEServer(None, devices=[self.device],
                                      bind_broadcast=False, verbose=False,
                                      bind_any=bind_any)
        self._running = True
        self._thread = threading.Thread(target=self._render_loop, daemon=True)
        self._thread.start()

    def _render_loop(self):
        """stands in for the engine loop calling gige_server.process()."""
        while self._running:
            self.server.process()
            time.sleep(0.005)

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self.server.close()


# ------------------------------------------------------------
# tests
# ------------------------------------------------------------

def test_discovery_ack_carries_bootstrap_identity():
    h, c = Harness(), None
    try:
        c = Client(DEVICE_IP)
        data = c.discover()
        assert len(data) == 248, f"discovery ack payload is {len(data)}B, want 248"
        assert struct.unpack_from(">I", data, 0)[0] == 0x00010002, "GEV 1.2 version"
        assert struct.unpack_from(">I", data, 4)[0] == 0x80000001, "device mode"
        assert data[10:16] == bytes([2, 0, 0, 0, 0, 1]), "mac at payload offset 10"
        assert struct.unpack_from(">I", data, 36)[0] == R.ip_to_u32(DEVICE_IP)
        assert data[72:83] == b"NightEngine", "manufacturer name at offset 72"
        assert data[104:118] == b"NightEngineCam", "model name at offset 104"
        assert data[216:222] == b"NE0001", "serial at offset 216"
        assert data[232:239] == b"StubCam", "user-defined name at offset 232"
    finally:
        if c:
            c.close()
        h.close()


def test_control_privilege_and_write_protection():
    h, c = Harness(), None
    try:
        c = Client(DEVICE_IP)
        assert c.read_reg(R.CCP) == 0, "device should start unclaimed"
        # HALCON writes 1 here, not 2
        c.write_reg(R.CCP, 1)
        assert c.read_reg(R.CCP) == 1
        # a read-only register must be refused
        try:
            c.write_reg(R.VERSION, 0)
            assert False, "writing the Version register should be refused"
        except RuntimeError as e:
            assert "0x8003" in str(e), f"expected invalid-address status, got {e}"
        c.write_reg(R.CCP, 0)
        assert c.read_reg(R.CCP) == 0, "releasing control should clear CCP"
    finally:
        if c:
            c.close()
        h.close()


def test_unimplemented_register_returns_0x8003():
    h, c = Harness(), None
    try:
        c = Client(DEVICE_IP)
        # misaligned / out of range must error rather than answer garbage,
        # mirroring the Nano answering 0x8003 for PendingTimeout
        status, _code, _body = c.command(P.READREG_CMD,
                                        P.pack_readreg([R.MEMORY_SIZE + 4]))
        assert status == P.STATUS_INVALID_ADDRESS, f"got status 0x{status:04x}"
    finally:
        if c:
            c.close()
        h.close()


def test_genapi_xml_is_fetchable_and_wellformed():
    h, c = Harness(), None
    try:
        c = Client(DEVICE_IP)
        c.write_reg(R.CCP, 1)
        xml_text, name, length = c.fetch_xml()
        assert name.endswith(".zip"), "we serve compressed xml"
        assert length == h.device.registers.xml_size

        root = ET.fromstring(xml_text)
        assert root.tag == f"{NS}RegisterDescription"
        assert root.get("StandardNameSpace") == "GEV"
        assert root.get("SchemaMinorVersion") == "1"

        names = {e.get("Name") for e in root.iter() if e.get("Name")}
        for mandatory in ("Root", "Device", "TLParamsLocked"):
            assert mandatory in names, f"missing mandatory node {mandatory}"
        for feature in ("Width", "Height", "PixelFormat", "PayloadSize",
                        "AcquisitionMode", "AcquisitionStart", "AcquisitionStop",
                        "DeviceVendorName", "DeviceModelName",
                        "GevSCPSPacketSize", "GevSCDA", "GevSCPHostPort"):
            assert feature in names, f"missing feature {feature}"

        # the Device port must not be inside the feature tree
        root_cat = [e for e in root.iter()
                    if e.tag == f"{NS}Category" and e.get("Name") == "Root"][0]
        assert "Device" not in [f.text for f in root_cat]

        # every pValue/pFeature reference must resolve
        for element in root.iter():
            if element.tag in (f"{NS}pValue", f"{NS}pFeature", f"{NS}pMin",
                               f"{NS}pMax", f"{NS}pInc", f"{NS}pIsLocked",
                               f"{NS}pVariable"):
                assert element.text in names, f"dangling reference: {element.text}"
    finally:
        if c:
            c.close()
        h.close()


def test_no_partial_selector_groups_in_the_xml():
    """SFNC trigger/gain/exposure features are selector-governed. Exposing
    TriggerMode without TriggerSelector makes a standards-following client
    fail on the selector write it performs first -- which is exactly what
    happened in the field. Omitting a whole group is safe; exposing half
    of one is not."""
    h, c = Harness(), None
    try:
        c = Client(DEVICE_IP)
        c.write_reg(R.CCP, 1)
        xml_text, _name, _length = c.fetch_xml()
        names = {e.get("Name") for e in ET.fromstring(xml_text).iter()
                 if e.get("Name")}

        groups = {
            "TriggerSelector": ["TriggerMode", "TriggerSource",
                                "TriggerActivation", "TriggerSoftware"],
            "GainSelector": ["Gain", "GainAuto"],
            "ExposureMode": ["ExposureTime", "ExposureAuto"],
            # LineSelector governs its group exactly as TriggerSelector
            # does: exposing LineInverter without it strands a client that
            # writes the selector first.
            "LineSelector": ["LineMode", "LineFormat", "LineInverter",
                             "LineStatus"],
        }
        for governor, members in groups.items():
            present = [m for m in members if m in names]
            if present:
                assert governor in names, (
                    f"{present} exposed without its governing feature "
                    f"'{governor}'; a client that writes the selector first "
                    f"will fail")

        # and the full trigger group should resolve to real registers
        for feature in ("TriggerSelector", "TriggerMode", "TriggerSource",
                        "TriggerActivation", "TriggerSoftware", "TriggerDelay",
                        "GainSelector", "GainAuto", "ExposureMode",
                        "ExposureAuto", "TransferControlMode",
                        "TransferBlockCount", "TransferStart", "TransferStop",
                        "TransferAbort", "LineSelector", "LineMode",
                        "LineFormat", "LineInverter", "LineStatus",
                        "LineStatusAll"):
            assert feature in names, f"missing {feature}"

        # every entry the customer's production script selects by name
        # must exist, or the corresponding set_framegrabber_param fails
        root = ET.fromstring(xml_text)
        for feature, wanted in (("TriggerSource", {"Software", "Line1", "Line2"}),
                                ("TriggerActivation", {"RisingEdge", "FallingEdge",
                                                       "AnyEdge", "LevelHigh",
                                                       "LevelLow"}),
                                ("TransferControlMode", {"Basic", "Automatic",
                                                         "UserControlled"}),
                                ("LineSelector", {"Line1", "Line2"})):
            node = next(e for e in root.iter()
                        if e.tag.endswith("Enumeration") and e.get("Name") == feature)
            have = {c.get("Name") for c in node
                    if c.tag.endswith("EnumEntry")}
            assert wanted <= have, f"{feature} missing {wanted - have}"
    finally:
        if c:
            c.close()
        h.close()


def test_triggered_mode_emits_only_on_software_trigger():
    h, c = Harness(), None
    try:
        c = Client(DEVICE_IP)
        c.write_reg(R.CCP, 1)
        c.write_reg(R.SCDA, R.ip_to_u32(DEVICE_IP))
        c.write_reg(R.SCP, c.stream_port)
        c.write_reg(R.REG_TRIGGER_SELECTOR, 0)      # FrameStart
        c.write_reg(R.REG_TRIGGER_SOURCE, 0)        # Software
        c.write_reg(R.REG_TRIGGER_MODE, 1)          # On
        c.write_reg(R.REG_FRAME_PERIOD_US, 10000)   # would be 100 fps free-running
        c.write_reg(R.REG_ACQUISITION_COMMAND, 1)

        # nothing should arrive without a trigger
        c.stream.settimeout(1.0)
        try:
            c.receive_frame(timeout=1.0)
            assert False, "a frame arrived with TriggerMode On and no trigger"
        except TimeoutError:
            pass

        c.stream.settimeout(5.0)
        c.write_reg(R.REG_TRIGGER_SOFTWARE, 1)
        leader, payload, _size_y, _pid = c.receive_frame()
        assert len(payload) == WIDTH * HEIGHT

        # and exactly one frame per trigger
        try:
            c.receive_frame(timeout=1.0)
            assert False, "a second frame arrived from a single trigger"
        except TimeoutError:
            pass

        c.write_reg(R.REG_ACQUISITION_COMMAND, 0)
    finally:
        if c:
            c.close()
        h.close()


def test_fire_test_packet_returns_one_datagram_of_requested_size():
    h, c = Harness(), None
    try:
        c = Client(DEVICE_IP)
        c.write_reg(R.CCP, 1)
        c.write_reg(R.SCDA, R.ip_to_u32(DEVICE_IP))
        c.write_reg(R.SCP, c.stream_port)
        size = 1500
        c.write_reg(R.SCPS, R.SCPS_FIRE_TEST_PACKET | R.SCPS_DO_NOT_FRAGMENT | size)
        data, _ = c.stream.recvfrom(65535)
        assert len(data) == size - 28, f"test packet is {len(data)}B, want {size-28}"
        # the fire bit must be self-clearing
        assert not (c.read_reg(R.SCPS) & R.SCPS_FIRE_TEST_PACKET)
        assert c.read_reg(R.SCPS) & R.SCPS_SIZE_MASK == size
    finally:
        if c:
            c.close()
        h.close()


def test_streams_a_frame_matching_the_rendered_source():
    h, c = Harness(), None
    try:
        c = Client(DEVICE_IP)
        c.write_reg(R.CCP, 1)
        xml_text, _name, _length = c.fetch_xml()

        # drive acquisition through the address our own XML advertises,
        # which validates the xml <-> register wiring
        root = ET.fromstring(xml_text)
        # EnumEntry names are scoped to their enumeration and legitimately
        # collide with feature names (the Genie Nano has a Command and five
        # EnumEntry nodes all called AcquisitionStart), so index only
        # feature-level nodes here
        by_name = {e.get("Name"): e for e in root.iter()
                   if e.get("Name") and not e.tag.endswith(("EnumEntry",
                                                            "StructEntry"))}
        start = by_name["AcquisitionStart"]
        reg_name = start.find(f"{NS}pValue").text
        command_value = int(start.find(f"{NS}CommandValue").text)
        address = int(by_name[reg_name].find(f"{NS}Address").text, 16)
        assert address == R.REG_ACQUISITION_COMMAND

        payload_size = c.read_reg(R.REG_PAYLOAD_SIZE)
        assert payload_size == WIDTH * HEIGHT, "mono8 payload is w*h bytes"

        c.write_reg(R.SCDA, R.ip_to_u32(DEVICE_IP))
        c.write_reg(R.SCP, c.stream_port)
        c.write_reg(R.SCPS, R.SCPS_DO_NOT_FRAGMENT | 576)  # force multi-packet
        c.write_reg(address, command_value)

        leader, data, size_y, trailer_packet_id = c.receive_frame()

        assert leader["payload_type"] == P.GVSP_PAYLOAD_TYPE_IMAGE
        assert leader["pixel_format"] == P.PFNC_MONO8
        assert (leader["width"], leader["height"]) == (WIDTH, HEIGHT)
        assert leader["block_id"] != 0, "block id 0 is reserved"
        assert size_y == HEIGHT
        assert len(data) == payload_size, \
            f"got {len(data)}B, PayloadSize says {payload_size}"

        expected_packets = -(-payload_size // P.payload_per_packet(576))
        assert trailer_packet_id == expected_packets + 1, \
            "trailer packet id must follow the last payload packet"

        # the streamed pixels must equal the mono8 of the source render
        from NightEngine.NightVision import NightVisionCamera
        expected = NightVisionCamera.to_mono8(h.camera.frame_bgr())
        received = np.frombuffer(data, dtype=np.uint8).reshape(HEIGHT, WIDTH)
        assert np.array_equal(received, expected), "streamed image differs from source"

        c.write_reg(address, 0)     # AcquisitionStop
    finally:
        if c:
            c.close()
        h.close()


def test_continuous_acquisition_increments_block_ids():
    h, c = Harness(), None
    try:
        c = Client(DEVICE_IP)
        c.write_reg(R.CCP, 1)
        c.write_reg(R.SCDA, R.ip_to_u32(DEVICE_IP))
        c.write_reg(R.SCP, c.stream_port)
        c.write_reg(R.REG_FRAME_PERIOD_US, 20000)       # 50 fps
        c.write_reg(R.REG_ACQUISITION_COMMAND, 1)

        blocks = []
        for _ in range(3):
            leader, data, _size_y, _pid = c.receive_frame()
            blocks.append(leader["block_id"])
            assert len(data) == WIDTH * HEIGHT
        assert blocks == sorted(blocks) and len(set(blocks)) == 3, \
            f"block ids should advance monotonically, got {blocks}"

        c.write_reg(R.REG_ACQUISITION_COMMAND, 0)
        assert h.device.acquiring is False
    finally:
        if c:
            c.close()
        h.close()


def test_rgb8_switch_changes_payload_size_and_pixels():
    h, c = Harness(), None
    try:
        c = Client(DEVICE_IP)
        c.write_reg(R.CCP, 1)
        c.write_reg(R.SCDA, R.ip_to_u32(DEVICE_IP))
        c.write_reg(R.SCP, c.stream_port)
        c.write_reg(R.REG_PIXEL_FORMAT, P.PFNC_RGB8)
        assert c.read_reg(R.REG_PAYLOAD_SIZE) == WIDTH * HEIGHT * 3

        c.write_reg(R.REG_ACQUISITION_COMMAND, 1)
        leader, data, _size_y, _pid = c.receive_frame()
        assert leader["pixel_format"] == P.PFNC_RGB8
        assert len(data) == WIDTH * HEIGHT * 3

        received = np.frombuffer(data, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3)
        expected = h.camera.frame_bgr()[:, :, ::-1]
        assert np.array_equal(received, expected), "rgb channel order is wrong"
        c.write_reg(R.REG_ACQUISITION_COMMAND, 0)
    finally:
        if c:
            c.close()
        h.close()


def test_bind_any_fallback_serves_discovery_and_streaming():
    """the 0.0.0.0 fallback used when a consumer's discovery broadcast
    never reaches a socket bound to the device's own address."""
    h, c = Harness(bind_any=True), None
    try:
        c = Client(DEVICE_IP)
        data = c.discover()
        assert len(data) == 248
        c.write_reg(R.CCP, 1)
        c.write_reg(R.SCDA, R.ip_to_u32(DEVICE_IP))
        c.write_reg(R.SCP, c.stream_port)
        c.write_reg(R.REG_ACQUISITION_COMMAND, 1)
        leader, payload, size_y, _pid = c.receive_frame()
        assert len(payload) == WIDTH * HEIGHT
        assert size_y == HEIGHT
        c.write_reg(R.REG_ACQUISITION_COMMAND, 0)
    finally:
        if c:
            c.close()
        h.close()


def test_multiple_devices_in_one_process_are_independent():
    """N cameras need N local IPs (the standard fixes the control port at
    3956), but they are hosted by a single process and a single render
    thread. Uses loopback aliases 127.0.0.x, which Windows and Linux both
    route locally."""
    ips = ["127.0.0.1", "127.0.0.2", "127.0.0.3"]
    devices = [NightGigEDevice(camera=StubCamera(name=f"cam{i}"), ip=ip,
                               mac=f"02:00:00:00:00:{i + 1:02x}",
                               subnet_mask="255.0.0.0",
                               serial=f"NE{i:08d}", user_name=f"cam{i}")
               for i, ip in enumerate(ips)]
    server = NightGigEServer(None, devices=devices, bind_broadcast=False,
                             verbose=False)
    running = True

    def render_loop():
        while running:
            server.process()
            time.sleep(0.005)

    thread = threading.Thread(target=render_loop, daemon=True)
    thread.start()
    clients = []
    try:
        for index, ip in enumerate(ips):
            client = Client(ip)
            clients.append(client)
            data = client.discover()
            # distinct serial / name / MAC, or consumers collapse them
            serial = data[216:232].split(b"\x00")[0].decode()
            name = data[232:248].split(b"\x00")[0].decode()
            assert serial == f"NE{index:08d}", f"{ip}: serial {serial!r}"
            assert name == f"cam{index}", f"{ip}: user name {name!r}"
            assert data[15] == index + 1, f"{ip}: MAC low byte {data[15]}"

        for client in clients:
            client.write_reg(R.CCP, 1)
            client.write_reg(R.SCDA, R.ip_to_u32(client.device[0]))
            client.write_reg(R.SCP, client.stream_port)
            client.write_reg(R.REG_ACQUISITION_COMMAND, 1)

        for client in clients:
            leader, payload, _size_y, _pid = client.receive_frame(timeout=8)
            assert len(payload) == WIDTH * HEIGHT

        for client in clients:
            client.write_reg(R.REG_ACQUISITION_COMMAND, 0)

        # control of one device must not disturb another
        clients[0].write_reg(R.CCP, 0)
        assert clients[1].read_reg(R.CCP) == 1
    finally:
        running = False
        thread.join(timeout=1.0)
        for client in clients:
            client.close()
        server.close()


def test_bind_any_rejects_a_second_device():
    h = Harness(bind_any=True)
    try:
        second = NightGigEDevice(camera=StubCamera(name="two"), ip=DEVICE_IP,
                                 mac="02:00:00:00:00:02", subnet_mask="255.0.0.0")
        try:
            h.server.add_device(device=second)
            assert False, "a second device under bind_any must be refused"
        except ValueError as e:
            assert "single device" in str(e)
    finally:
        h.close()


def test_line_trigger_needs_a_real_edge_on_the_configured_line():
    """the whole point of the line model: with TriggerSource Line2 the
    device must wait for Line2, must NOT fall back to free-running, and
    must NOT answer TriggerSoftware. That last one is the fidelity bug the
    line model exists to fix -- the trigger registers used to be written
    and then never consulted."""
    h, c = Harness(), None
    try:
        c = Client(DEVICE_IP)
        c.write_reg(R.CCP, 1)
        # this test deliberately waits in silence several times over; a
        # real consumer polls CCP meanwhile, so raise the timeout rather
        # than have our own heartbeat drop control mid-test
        c.write_reg(R.HEARTBEAT_TIMEOUT, 30000)
        c.write_reg(R.SCDA, R.ip_to_u32(DEVICE_IP))
        c.write_reg(R.SCP, c.stream_port)
        c.write_reg(R.REG_TRIGGER_SELECTOR, 0)          # FrameStart
        c.write_reg(R.REG_TRIGGER_SOURCE, 2)            # Line2
        c.write_reg(R.REG_TRIGGER_ACTIVATION, R.TRIGGER_ACTIVATION_RISING_EDGE)
        c.write_reg(R.REG_TRIGGER_MODE, 1)              # On
        c.write_reg(R.REG_FRAME_PERIOD_US, 10000)       # 100 fps if free-running
        c.write_reg(R.REG_ACQUISITION_COMMAND, 1)

        try:
            c.receive_frame(timeout=1.0)
            assert False, "device free-ran instead of waiting for Line2"
        except TimeoutError:
            pass

        # a software trigger must be ignored: we are wired to Line2
        c.write_reg(R.REG_TRIGGER_SOFTWARE, 1)
        try:
            c.receive_frame(timeout=1.0)
            assert False, "TriggerSoftware fired while TriggerSource was Line2"
        except TimeoutError:
            pass

        # the wrong line must be ignored too
        h.device.set_line(1, 1)
        try:
            c.receive_frame(timeout=1.0)
            assert False, "a Line1 edge fired a trigger configured for Line2"
        except TimeoutError:
            pass

        c.stream.settimeout(5.0)
        h.device.set_line(2, 1)
        _leader, payload, _size_y, _pid = c.receive_frame()
        assert len(payload) == WIDTH * HEIGHT

        # exactly one frame per edge: the falling edge must not fire too
        h.device.set_line(2, 0)
        try:
            c.receive_frame(timeout=1.0)
            assert False, "a falling edge fired a RisingEdge trigger"
        except TimeoutError:
            pass

        c.write_reg(R.REG_ACQUISITION_COMMAND, 0)
    finally:
        if c:
            c.close()
        h.close()


def _armed_device(activation, inverter=0, source=2):
    """a device configured for line triggering, with no sockets involved:
    the activation matrix is pure logic and does not need UDP."""
    device = NightGigEDevice(camera=StubCamera(), ip=DEVICE_IP,
                             mac="02:00:00:00:00:09")
    device.registers.write_u32(R.REG_TRIGGER_MODE, 1)
    device.registers.write_u32(R.REG_TRIGGER_SOURCE, source)
    device.registers.write_u32(R.REG_TRIGGER_ACTIVATION, activation)
    device.lines[2]["inverter"] = inverter
    device.acquiring = True
    device.registers.write_u32(R.SCDA, R.ip_to_u32(DEVICE_IP))
    device.registers.write_u32(R.SCP, 12345)
    return device


def test_trigger_activation_matrix():
    """each of the five activations, driven high then low."""
    now = time.monotonic()
    cases = [
        # activation,                          fires on rise, fires on fall
        (R.TRIGGER_ACTIVATION_RISING_EDGE,     True,  False),
        (R.TRIGGER_ACTIVATION_FALLING_EDGE,    False, True),
        (R.TRIGGER_ACTIVATION_ANY_EDGE,        True,  True),
    ]
    for activation, on_rise, on_fall in cases:
        device = _armed_device(activation)
        assert not device.wants_frame(now), "armed device wanted a frame unbidden"
        device.set_line(2, 1)
        assert device.wants_frame(now) == on_rise, (
            f"activation {activation}: rising edge should "
            f"{'' if on_rise else 'not '}fire")
        device.trigger_pending = False
        device.set_line(2, 0)
        assert device.wants_frame(now) == on_fall, (
            f"activation {activation}: falling edge should "
            f"{'' if on_fall else 'not '}fire")

    # the level modes are a gated free-run, not a one-shot: they stay
    # ready for as long as the line holds, and arm nothing on the edge.
    for activation, active_level in ((R.TRIGGER_ACTIVATION_LEVEL_HIGH, 1),
                                     (R.TRIGGER_ACTIVATION_LEVEL_LOW, 0)):
        device = _armed_device(activation)
        device.set_line(2, active_level)
        assert device.wants_frame(now), "level gate should be open"
        assert not device.trigger_pending, (
            "a level activation must not arm a one-shot trigger")
        # still open on a second look -- that is what makes it a free-run
        device.next_frame_due = 0.0
        assert device.wants_frame(now), "level gate closed after one frame"
        device.set_line(2, 1 - active_level)
        assert not device.wants_frame(now), "level gate should be shut"


def test_line_inverter_flips_which_edge_fires():
    """LineInverter is applied before edge detection, so an inverted line
    with RisingEdge fires on the raw FALLING edge. This is what places the
    customer's exposure mid-window instead of at the window edge."""
    now = time.monotonic()
    device = _armed_device(R.TRIGGER_ACTIVATION_RISING_EDGE, inverter=1)

    # inverted and idle low means the effective level already reads high
    assert device.line_level(2) == 1
    device.set_line(2, 1)               # raw rise -> effective FALL
    assert not device.wants_frame(now), "inverted line fired on the raw rise"
    device.set_line(2, 0)               # raw fall -> effective RISE
    assert device.wants_frame(now), "inverted line did not fire on the raw fall"


def test_line_selector_addresses_exactly_one_line():
    """LineMode/LineInverter/LineStatus are windows onto whichever line
    LineSelector names; writing one must not disturb its neighbours."""
    device = NightGigEDevice(camera=StubCamera(), ip=DEVICE_IP,
                             mac="02:00:00:00:00:0a")

    device.registers.write_u32(R.REG_LINE_SELECTOR, 2)
    device._write_hooks[R.REG_LINE_INVERTER](1)
    assert device.lines[2]["inverter"] == 1
    for other in (1, 3, 4):
        assert device.lines[other]["inverter"] == 0, (
            f"writing LineInverter for Line2 also changed Line{other}")

    # and reads come back through the same window
    assert device._read_hooks[R.REG_LINE_INVERTER]() == 1
    device.registers.write_u32(R.REG_LINE_SELECTOR, 3)
    assert device._read_hooks[R.REG_LINE_INVERTER]() == 0

    # LineStatus reports the effective level; LineStatusAll packs them
    # with Line1 in bit 0
    device.set_line(1, 1)
    device.set_line(4, 1)
    assert device._read_hooks[R.REG_LINE_STATUS_ALL]() == 0b1001 | (1 << 1)


def _transfer_client(c, mode, block_count=1):
    c.write_reg(R.CCP, 1)
    c.write_reg(R.HEARTBEAT_TIMEOUT, 30000)
    c.write_reg(R.SCDA, R.ip_to_u32(DEVICE_IP))
    c.write_reg(R.SCP, c.stream_port)
    c.write_reg(R.REG_TRANSFER_CONTROL_MODE, mode)
    c.write_reg(R.REG_TRANSFER_BLOCK_COUNT, block_count)
    c.write_reg(R.REG_FRAME_PERIOD_US, 20000)       # 50 fps
    c.write_reg(R.REG_ACQUISITION_COMMAND, 1)


def test_transfer_automatic_streams_without_being_asked():
    """Basic and Automatic must behave exactly as before this feature
    existed -- the customer's script sets Automatic."""
    for mode in (R.TRANSFER_CONTROL_BASIC, R.TRANSFER_CONTROL_AUTOMATIC):
        h, c = Harness(), None
        try:
            c = Client(DEVICE_IP)
            _transfer_client(c, mode)
            _leader, payload, _size_y, _pid = c.receive_frame(timeout=5.0)
            assert len(payload) == WIDTH * HEIGHT, f"mode {mode} streamed nothing"
            c.write_reg(R.REG_ACQUISITION_COMMAND, 0)
        finally:
            if c:
                c.close()
            h.close()


def test_transfer_user_controlled_holds_blocks_until_asked():
    h, c = Harness(), None
    try:
        c = Client(DEVICE_IP)
        _transfer_client(c, R.TRANSFER_CONTROL_USER_CONTROLLED, block_count=2)

        # acquisition is running, but nothing may leave the device
        try:
            c.receive_frame(timeout=1.5)
            assert False, "UserControlled streamed without a TransferStart"
        except TimeoutError:
            pass
        assert h.device.frames.qsize() > 0, "blocks should be queued, not dropped"
        assert c.read_reg(R.REG_TRANSFER_QUEUE_COUNT) > 0, (
            "TransferQueueCurrentBlockCount should report the held blocks")

        # one TransferStart releases exactly TransferBlockCount blocks
        c.write_reg(R.REG_TRANSFER_START, 1)
        for index in range(2):
            _l, payload, _s, _p = c.receive_frame(timeout=5.0)
            assert len(payload) == WIDTH * HEIGHT, f"block {index} truncated"
        try:
            c.receive_frame(timeout=1.5)
            assert False, "TransferStart released more than TransferBlockCount"
        except TimeoutError:
            pass

        c.write_reg(R.REG_ACQUISITION_COMMAND, 0)
    finally:
        if c:
            c.close()
        h.close()


def test_transfer_abort_discards_the_held_blocks():
    h, c = Harness(), None
    try:
        c = Client(DEVICE_IP)
        _transfer_client(c, R.TRANSFER_CONTROL_USER_CONTROLLED, block_count=8)
        time.sleep(0.8)                     # let some blocks pile up
        assert h.device.frames.qsize() > 0, "nothing was held to abort"

        c.write_reg(R.REG_TRANSFER_ABORT, 1)
        assert h.device.frames.qsize() == 0, "TransferAbort left blocks queued"

        # and an abort also stops the transfer: a later block must not
        # escape on the credit the aborted start would have had
        try:
            c.receive_frame(timeout=1.5)
            assert False, "a block escaped after TransferAbort"
        except TimeoutError:
            pass

        c.write_reg(R.REG_ACQUISITION_COMMAND, 0)
    finally:
        if c:
            c.close()
        h.close()


def test_transfer_stop_holds_and_overflow_drops_the_oldest():
    """a transfer held stopped long enough must overflow, and when it does
    the device keeps the newest view of the scene rather than a stale one."""
    device = NightGigEDevice(camera=StubCamera(), ip=DEVICE_IP,
                             mac="02:00:00:00:00:0b")
    device.registers.write_u32(R.REG_TRANSFER_CONTROL_MODE,
                               R.TRANSFER_CONTROL_USER_CONTROLLED)
    device.registers.write_u32(R.SCDA, R.ip_to_u32(DEVICE_IP))
    device.registers.write_u32(R.SCP, 12345)
    device._start_acquisition()
    assert not device.transfer_ready(), "UserControlled started already flowing"

    depth = device.frames.maxsize
    assert depth >= 16, "the queue must buffer while a transfer is stopped"
    for index in range(depth + 4):
        device.capture(None, time.monotonic())
    assert device.frames.qsize() == depth, "queue grew past its bound"

    # TransferStart with the default block count releases exactly one
    device.registers.write_u32(R.REG_TRANSFER_BLOCK_COUNT, 1)
    device._transfer_start()
    assert device.transfer_ready()
    device.frames.get_nowait()
    device.transfer_consume()
    assert not device.transfer_ready(), "credit outlived its one block"


def test_the_production_hdevelop_parameter_sequence_succeeds():
    """the customer's photometric script configures the camera in this
    exact order before grab_image_start. Four of these writes had no
    register behind them at all, which is where the script died; this is
    the direct regression test for 'will their script run unmodified'.

    write_reg raises on a non-zero GVCP status, so reaching the end is the
    assertion."""
    h, c = Harness(), None
    try:
        c = Client(DEVICE_IP)
        c.write_reg(R.CCP, 1)
        for address, value, label in (
                (R.REG_TRANSFER_CONTROL_MODE, R.TRANSFER_CONTROL_AUTOMATIC,
                 "TransferControlMode Automatic"),
                (R.REG_EXPOSURE_AUTO, 0, "ExposureAuto Off"),
                (R.REG_ACQUISITION_MODE, 0, "AcquisitionMode Continuous"),
                (R.REG_TRIGGER_SELECTOR, 0, "TriggerSelector FrameStart"),
                (R.REG_TRIGGER_MODE, 1, "TriggerMode On"),
                (R.REG_TRIGGER_SOURCE, 2, "TriggerSource Line2"),
                (R.REG_LINE_SELECTOR, 2, "LineSelector Line2"),
                (R.REG_LINE_INVERTER, 1, "LineInverter 1")):
            try:
                c.write_reg(address, value)
            except RuntimeError as error:
                assert False, f"{label} rejected: {error}"

        # and the configuration must have actually landed, not merely been
        # accepted and dropped
        assert h.device.trigger_line == 2, "TriggerSource did not select Line2"
        assert h.device.lines[2]["inverter"] == 1, "LineInverter did not apply"
        assert h.device.lines[1]["inverter"] == 0, "LineInverter hit the wrong line"
        assert c.read_reg(R.REG_TRANSFER_CONTROL_MODE) == R.TRANSFER_CONTROL_AUTOMATIC
    finally:
        if c:
            c.close()
        h.close()


def test_halcon_device_name_matches_the_production_convention():
    """HALCON scripts hardcode this string in open_framegrabber, so the
    emulator has to reproduce the convention exactly: MAC with separators
    stripped, then vendor and model with spaces stripped. The expected
    value here is the one from the customer's production script."""
    device = NightGigEDevice(camera=StubCamera(), ip=DEVICE_IP,
                             mac="1C:0F:AF:7A:E6:BC",
                             vendor="Lucid Vision Labs", model="TRI050SM")
    assert device.halcon_name == "1c0faf7ae6bc_LucidVisionLabs_TRI050SM", (
        f"got {device.halcon_name}")
    assert device.info()["halcon_name"] == device.halcon_name

    # dashes are a legal MAC separator too
    dashed = NightGigEDevice(camera=StubCamera(), ip=DEVICE_IP,
                             mac="1c-0f-af-7a-e6-bc",
                             vendor="Lucid Vision Labs", model="TRI050SM")
    assert dashed.halcon_name == device.halcon_name


def test_heartbeat_timeout_releases_control():
    h, c = Harness(), None
    try:
        c = Client(DEVICE_IP)
        c.write_reg(R.CCP, 1)
        c.write_reg(R.HEARTBEAT_TIMEOUT, 600)       # clamped to >= 0.5 s
        time.sleep(1.2)
        # any inbound command makes the device notice the lapse
        assert c.read_reg(R.CCP) == 0, "stale controller should be dropped"
    finally:
        if c:
            c.close()
        h.close()


if __name__ == "__main__":
    tests = [(name, obj) for name, obj in sorted(globals().items())
             if name.startswith("test_") and callable(obj)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as e:
            failed += 1
            print(f"  FAIL  {name}: {type(e).__name__}: {e}")
        time.sleep(0.2)     # let sockets on 3956 fully release
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)

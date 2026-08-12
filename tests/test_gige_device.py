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
        by_name = {e.get("Name"): e for e in root.iter() if e.get("Name")}
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

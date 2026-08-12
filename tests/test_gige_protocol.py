# test_gige_protocol.py
# offline unit tests for the GVCP/GVSP codec and the register file.
# no sockets, no OpenGL. run directly:  python tests/test_gige_protocol.py

import os
import sys
import struct

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from NightEngine.GigE import protocol as P
from NightEngine.GigE import registers as R


# ------------------------------------------------------------
# GVCP headers
# ------------------------------------------------------------

def test_cmd_header_layout():
    packet = P.pack_cmd(P.READREG_CMD, 0x1234, struct.pack(">I", 0x0A00))
    assert len(packet) == 12
    assert packet[0] == 0x42, "byte 0 must be the 0x42 message key"
    assert packet[1] == P.FLAG_ACK_REQUIRED
    assert struct.unpack_from(">H", packet, 2)[0] == P.READREG_CMD
    assert struct.unpack_from(">H", packet, 4)[0] == 4, "length excludes the header"
    assert struct.unpack_from(">H", packet, 6)[0] == 0x1234

    cmd = P.parse_cmd(packet)
    assert cmd.code == P.READREG_CMD and cmd.req_id == 0x1234
    assert cmd.payload == struct.pack(">I", 0x0A00)


def test_parse_cmd_rejects_non_commands():
    assert P.parse_cmd(b"") is None
    assert P.parse_cmd(b"\x00" * 8) is None, "acks have no 0x42 key"
    assert P.parse_cmd(b"\x42\x01\x00\x80\x00\xff\x00\x01") is None, "truncated payload"


def test_ack_header_layout():
    ack = P.pack_ack(P.READREG_ACK, 0x1234, struct.pack(">I", 7))
    # first two bytes are a 16-bit status, NOT a key byte
    assert struct.unpack_from(">H", ack, 0)[0] == P.STATUS_SUCCESS
    assert struct.unpack_from(">H", ack, 2)[0] == P.READREG_ACK
    assert struct.unpack_from(">H", ack, 4)[0] == 4
    assert struct.unpack_from(">H", ack, 6)[0] == 0x1234

    status, code, ack_id, payload = P.parse_ack(ack)
    assert (status, code, ack_id) == (0, P.READREG_ACK, 0x1234)
    assert payload == struct.pack(">I", 7)


def test_error_status_uses_high_bit():
    # the Genie Nano answers 0x8003 for an unimplemented register
    ack = P.pack_ack(P.READREG_ACK, 1, status=P.STATUS_INVALID_ADDRESS)
    assert P.parse_ack(ack)[0] == 0x8003


# ------------------------------------------------------------
# GVCP payloads
# ------------------------------------------------------------

def test_readreg_roundtrip():
    addresses = [R.CCP, R.VERSION, R.SCPS]
    assert P.parse_readreg(P.pack_readreg(addresses)) == addresses
    values = [1, 0x00010002, 1500]
    assert list(struct.unpack(">3I", P.pack_readreg_ack(values))) == values


def test_writereg_roundtrip():
    pairs = [(R.SCDA, 0xA9FE055E), (R.SCP, 0xD730)]
    assert P.parse_writereg(P.pack_writereg(pairs)) == pairs
    assert struct.unpack(">I", P.pack_writereg_ack(2))[0] == 2


def test_readmem_roundtrip():
    payload = P.pack_readmem(0x10000, 512)
    assert len(payload) == 8
    assert P.parse_readmem(payload) == (0x10000, 512)
    ack = P.pack_readmem_ack(0x10000, b"abcd")
    assert struct.unpack_from(">I", ack, 0)[0] == 0x10000
    assert ack[4:] == b"abcd"


def test_writemem_roundtrip():
    payload = struct.pack(">I", 0x0100) + b"\x00\x00\x02\x80"
    assert P.parse_writemem(payload) == (0x0100, b"\x00\x00\x02\x80")


def test_packetresend_standard_layout():
    payload = struct.pack(">HHII", 0, 42, 3, 9)
    assert P.parse_packetresend(payload) == (0, 42, 3, 9)


# ------------------------------------------------------------
# GVSP
# ------------------------------------------------------------

def test_gvsp_header_packs_content_type_and_packet_id():
    header = P.pack_gvsp_header(block_id=7, packet_id=0x123456,
                               content_type=P.GVSP_CONTENT_PAYLOAD)
    assert len(header) == 8
    parsed = P.parse_gvsp(header)
    assert parsed["block_id"] == 7
    assert parsed["packet_id"] == 0x123456
    assert parsed["content_type"] == P.GVSP_CONTENT_PAYLOAD
    assert parsed["extended"] is False
    # byte 4 carries EI<<7 | content_type
    assert header[4] == P.GVSP_CONTENT_PAYLOAD


def test_gvsp_image_leader_is_36_bytes():
    packet = P.pack_gvsp_leader(block_id=1, timestamp=0x1122334455667788,
                                pixel_format=P.PFNC_MONO8,
                                width=640, height=480,
                                padding_x=3, padding_y=5)
    assert len(packet) == 8 + 36
    body = packet[8:]
    assert struct.unpack_from(">H", body, 2)[0] == P.GVSP_PAYLOAD_TYPE_IMAGE
    assert struct.unpack_from(">I", body, 4)[0] == 0x11223344
    assert struct.unpack_from(">I", body, 8)[0] == 0x55667788
    assert struct.unpack_from(">I", body, 12)[0] == P.PFNC_MONO8
    assert struct.unpack_from(">I", body, 16)[0] == 640
    assert struct.unpack_from(">I", body, 20)[0] == 480
    # paddings are 16-bit; Aravis writes 32 bits here and corrupts them
    assert struct.unpack_from(">H", body, 32)[0] == 3
    assert struct.unpack_from(">H", body, 34)[0] == 5


def test_gvsp_image_trailer_is_8_bytes():
    packet = P.pack_gvsp_trailer(block_id=1, packet_id=9, size_y=480)
    assert len(packet) == 8 + 8
    body = packet[8:]
    assert struct.unpack_from(">H", body, 2)[0] == P.GVSP_PAYLOAD_TYPE_IMAGE
    assert struct.unpack_from(">I", body, 4)[0] == 480


def test_pixel_format_bit_encoding():
    assert P.pixel_format_bits(P.PFNC_MONO8) == 8
    assert P.pixel_format_bytes(P.PFNC_MONO8) == 1
    assert P.pixel_format_bytes(P.PFNC_RGB8) == 3
    assert P.pixel_format_bytes(P.PFNC_MONO16) == 2


def test_payload_per_packet_matches_36_byte_overhead():
    # confirmed independently by the Nano's own GenApi formula: PACKET_SIZE-36
    assert P.payload_per_packet(1500) == 1464
    assert P.payload_per_packet(9000) == 8964


def test_test_packet_size_matches_requested_datagram():
    # the Nano answered SCPS=1500 with a 1472-byte UDP payload
    assert len(P.pack_gvsp_test_packet(1500)) == 1472


# ------------------------------------------------------------
# register file
# ------------------------------------------------------------

def test_register_file_is_big_endian():
    reg = R.RegisterFile()
    reg.write_u32(R.VERSION, 0x00010002)
    assert reg.mem[0:4] == b"\x00\x01\x00\x02"
    assert reg.read_u32(R.VERSION) == 0x00010002


def test_string_registers_are_nul_padded():
    reg = R.RegisterFile()
    reg.write_string(R.MODEL_NAME, "NightEngineCam", 32)
    assert reg.read_string(R.MODEL_NAME, 32) == "NightEngineCam"
    assert reg.mem[R.MODEL_NAME + 14] == 0
    assert len(reg.mem[R.MODEL_NAME:R.MODEL_NAME + 32]) == 32


def test_discovery_data_is_248_bytes():
    reg = R.RegisterFile()
    reg.write_string(R.MANUFACTURER_NAME, "NightEngine", 32)
    data = reg.discovery_data()
    assert len(data) == P.DISCOVERY_DATA_SIZE == 0xF8
    # offsets inside the payload equal the bootstrap addresses
    assert data[72:83] == b"NightEngine"


def test_read_zero_fills_past_the_end():
    reg = R.RegisterFile()
    reg.set_xml(b"<xml/>")
    chunk = reg.read(R.XML_BASE, 512)
    assert len(chunk) == 512
    assert chunk[:6] == b"<xml/>"
    assert chunk[6:] == b"\x00" * 506


def test_xml_region_is_read_only():
    reg = R.RegisterFile()
    reg.set_xml(b"<xml/>")
    assert reg.write(R.XML_BASE, b"nope") is False
    assert reg.write(R.REG_WIDTH, b"\x00\x00\x02\x80") is True
    assert reg.read_u32(R.REG_WIDTH) == 640


def test_ip_and_mac_conversion():
    assert R.ip_to_u32("169.254.5.50") == 0xA9FE0532
    assert R.u32_to_ip(0xA9FE0532) == "169.254.5.50"
    assert R.mac_to_bytes("00:01:0d:c6:2b:38") == bytes([0, 1, 0x0D, 0xC6, 0x2B, 0x38])


def test_stream_channel_stride():
    assert R.stream_reg(R.SCPS, 0) == 0x0D04
    assert R.stream_reg(R.SCPS, 1) == 0x0D44


def test_manifest_table_capability_stays_clear():
    # advertising it makes HALCON chase a manifest at 0x9000 and fetch
    # multiple XML files
    assert not (R.GVCP_CAPABILITY_VALUE & R.CAP_MANIFEST_TABLE)


# ------------------------------------------------------------
# runner
# ------------------------------------------------------------

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
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)

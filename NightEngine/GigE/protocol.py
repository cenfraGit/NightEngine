# protocol.py
# pure GVCP / GVSP packet codec: no sockets, no OpenGL, no device state.
# every multi-byte field is big-endian.
#
# layouts cross-verified against the Aravis and Wireshark
# implementations and against a captured Genie Nano <-> HALCON session.

import struct

# ------------------------------------------------------------
# GVCP
# ------------------------------------------------------------

GVCP_PORT = 3956

CMD_KEY = 0x42                      # first byte of every command

# command / acknowledge codes
DISCOVERY_CMD, DISCOVERY_ACK = 0x0002, 0x0003
FORCEIP_CMD, FORCEIP_ACK = 0x0004, 0x0005
PACKETRESEND_CMD, PACKETRESEND_ACK = 0x0040, 0x0041
READREG_CMD, READREG_ACK = 0x0080, 0x0081
WRITEREG_CMD, WRITEREG_ACK = 0x0082, 0x0083
READMEM_CMD, READMEM_ACK = 0x0084, 0x0085
WRITEMEM_CMD, WRITEMEM_ACK = 0x0086, 0x0087
PENDING_ACK = 0x0089
EVENT_CMD, EVENT_ACK = 0x00C0, 0x00C1

# command flag bits
FLAG_ACK_REQUIRED = 0x01
FLAG_ALLOW_BROADCAST_ACK = 0x10     # on DISCOVERY; extended ids on PACKETRESEND

# status codes. the capture proves errors are 0x8000|code and that a
# consumer tolerates them: the Nano answers 0x8003 for a register it
# does not implement and HALCON carries on.
STATUS_SUCCESS = 0x0000
STATUS_NOT_IMPLEMENTED = 0x8001
STATUS_INVALID_PARAMETER = 0x8002
STATUS_INVALID_ADDRESS = 0x8003
STATUS_WRITE_PROTECT = 0x8004
STATUS_BAD_ALIGNMENT = 0x8005
STATUS_ACCESS_DENIED = 0x8006
STATUS_BUSY = 0x8007
STATUS_PACKET_UNAVAILABLE = 0x800C
STATUS_PACKET_REMOVED = 0x8012
STATUS_GENERIC = 0x8FFF

# a READMEM acknowledge carries at most this many data bytes
DATA_SIZE_MAX = 512

# DISCOVERY_ACK payload is a verbatim copy of bootstrap 0x0000..0x00F7
DISCOVERY_DATA_SIZE = 0xF8


class Command:
    """a parsed GVCP command."""

    __slots__ = ("code", "flags", "req_id", "payload")

    def __init__(self, code, flags, req_id, payload):
        self.code = code
        self.flags = flags
        self.req_id = req_id
        self.payload = payload

    def __repr__(self):
        return (f"Command(code=0x{self.code:04x}, flags=0x{self.flags:02x}, "
                f"req_id={self.req_id}, payload={len(self.payload)}B)")


def pack_cmd(code, req_id, payload=b"", flags=FLAG_ACK_REQUIRED):
    """builds a GVCP command (used by our test client)."""
    return struct.pack(">BBHHH", CMD_KEY, flags, code, len(payload), req_id) + payload


def parse_cmd(data):
    """parses a GVCP command. returns None if this is not one."""
    if len(data) < 8 or data[0] != CMD_KEY:
        return None
    flags = data[1]
    code, length, req_id = struct.unpack_from(">HHH", data, 2)
    payload = data[8:8 + length]
    if len(payload) < length:
        return None
    return Command(code, flags, req_id, payload)


def pack_ack(ack_code, ack_id, payload=b"", status=STATUS_SUCCESS):
    """builds a GVCP acknowledge. note there is no 0x42 key byte in an
    ack -- the first two bytes are the 16-bit status."""
    return struct.pack(">HHHH", status, ack_code, len(payload), ack_id) + payload


def parse_ack(data):
    """returns (status, ack_code, ack_id, payload) or None."""
    if len(data) < 8:
        return None
    status, code, length, ack_id = struct.unpack_from(">HHHH", data, 0)
    return status, code, ack_id, data[8:8 + length]


# ---------------------- payloads ---------------------- #

def parse_readreg(payload):
    """READREG_CMD payload -> list of register addresses."""
    n = len(payload) // 4
    return list(struct.unpack_from(f">{n}I", payload, 0)) if n else []


def pack_readreg(addresses):
    return struct.pack(f">{len(addresses)}I", *addresses)


def pack_readreg_ack(values):
    return struct.pack(f">{len(values)}I", *values)


def parse_writereg(payload):
    """WRITEREG_CMD payload -> list of (address, value) pairs."""
    pairs = []
    for i in range(0, len(payload) - 7, 8):
        pairs.append(struct.unpack_from(">II", payload, i))
    return pairs


def pack_writereg(pairs):
    out = b""
    for addr, value in pairs:
        out += struct.pack(">II", addr, value)
    return out


def pack_writereg_ack(written_count):
    """the acknowledge reports how many registers were written."""
    return struct.pack(">I", written_count)


def parse_readmem(payload):
    """READMEM_CMD payload -> (address, count)."""
    address, _reserved, count = struct.unpack(">IHH", payload[:8])
    return address, count


def pack_readmem(address, count):
    return struct.pack(">IHH", address, 0, count)


def pack_readmem_ack(address, data):
    return struct.pack(">I", address) + data


def parse_writemem(payload):
    """WRITEMEM_CMD payload -> (address, data)."""
    address = struct.unpack_from(">I", payload, 0)[0]
    return address, payload[4:]


def pack_writemem_ack(nbytes):
    return struct.pack(">HH", 0, nbytes)


def parse_packetresend(payload, extended=False):
    """-> (channel, block_id, first_packet_id, last_packet_id)."""
    if extended:
        channel = struct.unpack_from(">I", payload, 0)[0] & 0xFFFF
        first, last = struct.unpack_from(">II", payload, 4)
        block = struct.unpack_from(">Q", payload, 12)[0]
        return channel, block, first, last
    channel, block = struct.unpack_from(">HH", payload, 0)
    first = struct.unpack_from(">I", payload, 4)[0] & 0xFFFFFF
    last = struct.unpack_from(">I", payload, 8)[0] & 0xFFFFFF
    return channel, block, first, last


# ------------------------------------------------------------
# GVSP
# ------------------------------------------------------------

GVSP_CONTENT_LEADER = 0x01
GVSP_CONTENT_TRAILER = 0x02
GVSP_CONTENT_PAYLOAD = 0x03
GVSP_CONTENT_ALLIN = 0x04

GVSP_PAYLOAD_TYPE_IMAGE = 0x0001

# per-packet protocol overhead: 20 (IPv4) + 8 (UDP) + 8 (GVSP header).
# independently confirmed by the Nano's own GenApi formula, which
# computes its data packet size as PACKET_SIZE - 36.
GVSP_OVERHEAD = 36

# PFNC pixel format values. bits 23..16 encode bits-per-pixel.
PFNC_MONO8 = 0x01080001
PFNC_MONO16 = 0x01100007
PFNC_RGB8 = 0x02180014
PFNC_BGR8 = 0x02180015
PFNC_BAYERRG8 = 0x01080009


def pixel_format_bits(pixel_format):
    return (pixel_format >> 16) & 0xFF


def pixel_format_bytes(pixel_format):
    return pixel_format_bits(pixel_format) // 8


def payload_per_packet(scps_packet_size):
    """image bytes carried by one GVSP payload packet."""
    return max(1, scps_packet_size - GVSP_OVERHEAD)


def pack_gvsp_header(block_id, packet_id, content_type, status=0):
    """8-byte standard GVSP header (no extended ids)."""
    infos = ((content_type << 24) & 0x7F000000) | (packet_id & 0x00FFFFFF)
    return struct.pack(">HHI", status, block_id & 0xFFFF, infos)


def parse_gvsp(data):
    """-> dict describing a GVSP packet, or None."""
    if len(data) < 8:
        return None
    status, block_id, infos = struct.unpack_from(">HHI", data, 0)
    return {"status": status,
            "block_id": block_id,
            "extended": bool(infos & 0x80000000),
            "content_type": (infos >> 24) & 0x7F,
            "packet_id": infos & 0x00FFFFFF,
            "payload": data[8:]}


def pack_gvsp_leader(block_id, timestamp, pixel_format, width, height,
                     offset_x=0, offset_y=0, padding_x=0, padding_y=0):
    """leader packet: 8-byte header + 36-byte image leader."""
    body = struct.pack(">HHIIIIIII",
                       0,                              # field info / reserved
                       GVSP_PAYLOAD_TYPE_IMAGE,
                       (timestamp >> 32) & 0xFFFFFFFF,
                       timestamp & 0xFFFFFFFF,
                       pixel_format, width, height,
                       offset_x, offset_y)
    # 16-bit stores: Aravis has a bug writing these as 32-bit
    body += struct.pack(">HH", padding_x, padding_y)
    return pack_gvsp_header(block_id, 0, GVSP_CONTENT_LEADER) + body


def pack_gvsp_payload(block_id, packet_id, data):
    return pack_gvsp_header(block_id, packet_id, GVSP_CONTENT_PAYLOAD) + data


def pack_gvsp_trailer(block_id, packet_id, size_y):
    body = struct.pack(">HHI", 0, GVSP_PAYLOAD_TYPE_IMAGE, size_y)
    return pack_gvsp_header(block_id, packet_id, GVSP_CONTENT_TRAILER) + body


def pack_gvsp_test_packet(packet_size):
    """the fire-test-packet response: one datagram of exactly the
    requested IP-datagram size, with a zeroed GVSP header. matches what
    the Genie Nano sends (observed: 1472 UDP bytes for SCPS 1500)."""
    udp_len = max(8, packet_size - 28)          # minus IPv4 + UDP headers
    return b"\x00" * 8 + b"\x00" * (udp_len - 8)

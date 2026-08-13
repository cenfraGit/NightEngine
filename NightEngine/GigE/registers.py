# registers.py
# the GigE Vision bootstrap register map and the device register file.
#
# every multi-byte register is big-endian. addresses and the values
# marked "observed" were taken from a real Teledyne DALSA Genie Nano
# answering HALCON, so they are known-good rather than inferred.

import struct

# ------------------------------------------------------------
# bootstrap register addresses
# ------------------------------------------------------------

VERSION = 0x0000
DEVICE_MODE = 0x0004
MAC_HIGH = 0x0008
MAC_LOW = 0x000C
SUPPORTED_IP_CONFIG = 0x0010
CURRENT_IP_CONFIG = 0x0014
CURRENT_IP = 0x0024
SUBNET_MASK = 0x0034
GATEWAY = 0x0044
MANUFACTURER_NAME = 0x0048          # 32 bytes
MODEL_NAME = 0x0068                 # 32 bytes
DEVICE_VERSION = 0x0088             # 32 bytes
MANUFACTURER_INFO = 0x00A8          # 48 bytes
SERIAL_NUMBER = 0x00D8              # 16 bytes
USER_DEFINED_NAME = 0x00E8          # 16 bytes
FIRST_URL = 0x0200                  # 512 bytes
SECOND_URL = 0x0400                 # 512 bytes
NUM_NETWORK_INTERFACES = 0x0600
PERSISTENT_IP = 0x064C
PERSISTENT_SUBNET = 0x065C
PERSISTENT_GATEWAY = 0x066C
LINK_SPEED = 0x0670
NUM_MESSAGE_CHANNELS = 0x0900
NUM_STREAM_CHANNELS = 0x0904
NUM_ACTION_SIGNALS = 0x0908
SC_CAPABILITY = 0x092C
MESSAGE_CHANNEL_CAPABILITY = 0x0930
GVCP_CAPABILITY = 0x0934
HEARTBEAT_TIMEOUT = 0x0938
TICK_FREQ_HIGH = 0x093C
TICK_FREQ_LOW = 0x0940
TIMESTAMP_CONTROL = 0x0944
TIMESTAMP_LATCHED_HIGH = 0x0948
TIMESTAMP_LATCHED_LOW = 0x094C
DISCOVERY_ACK_DELAY = 0x0950
GVCP_CONFIGURATION = 0x0954
PENDING_TIMEOUT = 0x0958
CONTROL_SWITCHOVER_KEY = 0x095C
GVSP_CONFIGURATION = 0x0960
CCP = 0x0A00
PRIMARY_APP_PORT = 0x0A04
PRIMARY_APP_IP = 0x0A14
MCP = 0x0B00
MCDA = 0x0B10
MC_TIMEOUT = 0x0B14
MC_RETRY_COUNT = 0x0B18
MC_SOURCE_PORT = 0x0B1C

# stream channel registers, stride 0x40 per channel
SCP = 0x0D00                        # low 16 bits = host port
SCPS = 0x0D04                       # low 16 bits = packet size
SCPD = 0x0D08                       # packet delay
SCDA = 0x0D18                       # destination address
SCSP = 0x0D1C                       # our source port (read-only)
SCC = 0x0D20                        # channel capability
SCCFG = 0x0D24                      # channel configuration
STREAM_CHANNEL_STRIDE = 0x40

# SCPS bit flags (normal LSB-first numbering)
SCPS_FIRE_TEST_PACKET = 1 << 31
SCPS_DO_NOT_FRAGMENT = 1 << 30
SCPS_BIG_ENDIAN = 1 << 29
SCPS_SIZE_MASK = 0x0000FFFF

# CCP bits. HALCON writes 1; Aravis writes 2. Treat any non-zero as
# "a controller exists".
CCP_EXCLUSIVE = 1 << 0
CCP_CONTROL = 1 << 1

# GVCP capability bits (LSB-first shifts)
CAP_CONCATENATION = 1 << 0
CAP_WRITEMEM = 1 << 1
CAP_PACKETRESEND = 1 << 2
CAP_EVENT = 1 << 3
CAP_EVENTDATA = 1 << 4
CAP_PENDING_ACK = 1 << 5
CAP_ACTION = 1 << 6
CAP_EXTENDED_STATUS = 1 << 22
CAP_TEST_DATA = 1 << 25
CAP_MANIFEST_TABLE = 1 << 26        # keep CLEAR: setting it makes HALCON
                                    # chase a manifest at 0x9000
CAP_LINK_SPEED = 1 << 28
CAP_HEARTBEAT_DISABLE = 1 << 29
CAP_SERIAL_NUMBER = 1 << 30
CAP_USER_DEFINED_NAME = 1 << 31

# what we advertise: concatenation + WRITEMEM + PACKETRESEND +
# extended status codes + test data + link speed + serial + user name.
# deliberately excludes the manifest table, events and actions.
GVCP_CAPABILITY_VALUE = (CAP_CONCATENATION | CAP_WRITEMEM | CAP_PACKETRESEND |
                         CAP_EXTENDED_STATUS | CAP_TEST_DATA | CAP_LINK_SPEED |
                         CAP_SERIAL_NUMBER | CAP_USER_DEFINED_NAME)

# ------------------------------------------------------------
# vendor register area (referenced by our GenApi XML)
# ------------------------------------------------------------

VENDOR_BASE = 0x0100
REG_WIDTH = 0x0100
REG_HEIGHT = 0x0104
REG_PIXEL_FORMAT = 0x0108
REG_ACQUISITION_COMMAND = 0x010C    # write 1 = start, 0 = stop
REG_ACQUISITION_MODE = 0x0110       # 0 continuous, 1 single frame
REG_SENSOR_WIDTH = 0x0114
REG_SENSOR_HEIGHT = 0x0118
REG_FRAME_PERIOD_US = 0x011C
REG_PAYLOAD_SIZE = 0x0120
REG_EXPOSURE_TIME_US = 0x0124
REG_GAIN_RAW = 0x0128
REG_OFFSET_X = 0x012C
REG_OFFSET_Y = 0x0130
REG_TRIGGER_MODE = 0x0134
REG_TRIGGER_SOFTWARE = 0x0138
REG_DEVICE_RESET = 0x013C
# SFNC trigger/gain/exposure features are selector-governed: exposing
# TriggerMode without TriggerSelector makes a standards-following client
# fail on the selector write it does first. Keep each group complete.
REG_TRIGGER_SELECTOR = 0x0140
REG_TRIGGER_SOURCE = 0x0144
REG_TRIGGER_ACTIVATION = 0x0148
REG_TRIGGER_DELAY_US = 0x014C
REG_GAIN_SELECTOR = 0x0150
REG_GAIN_AUTO = 0x0154
REG_EXPOSURE_MODE = 0x0158
REG_EXPOSURE_AUTO = 0x015C

# physical I/O lines. LineMode/LineInverter/LineStatus are per-line but
# addressed through LineSelector, so these three are windows onto the
# currently selected line rather than storage in their own right --
# device.py resolves them through its read/write hooks.
REG_LINE_SELECTOR = 0x0160
REG_LINE_MODE = 0x0164
REG_LINE_INVERTER = 0x0168
REG_LINE_STATUS = 0x016C            # read-only
REG_LINE_STATUS_ALL = 0x0170        # read-only bitmask, bit0 = Line1

# Line1..Line4. The Nano numbers its lines from 1, not 0, and the
# selector value equals the line number so TriggerSource and
# LineSelector agree on what "Line2" means.
LINE_COUNT = 4
LINE_INPUT = 0
LINE_OUTPUT = 1

# TriggerSource values. Software stays 0 so existing behaviour and any
# stored client configuration are unchanged.
TRIGGER_SOURCE_SOFTWARE = 0
TRIGGER_SOURCE_LINE1 = 1            # ... Line N == N, up to LINE_COUNT

# TriggerActivation values. The first three are edge-triggered and arm a
# single frame; the level modes gate a free-run instead.
TRIGGER_ACTIVATION_RISING_EDGE = 0
TRIGGER_ACTIVATION_FALLING_EDGE = 1
TRIGGER_ACTIVATION_ANY_EDGE = 2
TRIGGER_ACTIVATION_LEVEL_HIGH = 3
TRIGGER_ACTIVATION_LEVEL_LOW = 4
TRIGGER_ACTIVATION_LEVEL_MODES = (TRIGGER_ACTIVATION_LEVEL_HIGH,
                                  TRIGGER_ACTIVATION_LEVEL_LOW)

# transfer control: who decides when an acquired block leaves the device.
REG_TRANSFER_CONTROL_MODE = 0x0174
REG_TRANSFER_START = 0x0178         # command
REG_TRANSFER_STOP = 0x017C          # command
REG_TRANSFER_ABORT = 0x0180         # command
REG_TRANSFER_BLOCK_COUNT = 0x0184   # blocks released per TransferStart
REG_TRANSFER_QUEUE_COUNT = 0x0188   # read-only: blocks waiting in the device

# Basic and Automatic both stream on acquisition; only UserControlled
# holds blocks back until the client asks for them. The Nano offers only
# Basic and UserControlled, but the customer's script sets Automatic
# (standard SFNC, which Lucid implements), so we expose all three.
TRANSFER_CONTROL_BASIC = 0
TRANSFER_CONTROL_AUTOMATIC = 1
TRANSFER_CONTROL_USER_CONTROLLED = 2

# registers a client may write (everything else is rejected)
WRITABLE = {
    USER_DEFINED_NAME, FIRST_URL, GVCP_CONFIGURATION, GVSP_CONFIGURATION,
    HEARTBEAT_TIMEOUT, DISCOVERY_ACK_DELAY, TIMESTAMP_CONTROL, CCP,
    MCP, MCDA, MC_TIMEOUT, MC_RETRY_COUNT,
    SCP, SCPS, SCPD, SCDA, SCCFG,
    REG_WIDTH, REG_HEIGHT, REG_PIXEL_FORMAT, REG_ACQUISITION_COMMAND,
    REG_ACQUISITION_MODE, REG_FRAME_PERIOD_US, REG_EXPOSURE_TIME_US,
    REG_GAIN_RAW, REG_OFFSET_X, REG_OFFSET_Y, REG_TRIGGER_MODE,
    REG_TRIGGER_SOFTWARE, REG_DEVICE_RESET,
    REG_TRIGGER_SELECTOR, REG_TRIGGER_SOURCE, REG_TRIGGER_ACTIVATION,
    REG_TRIGGER_DELAY_US, REG_GAIN_SELECTOR, REG_GAIN_AUTO,
    REG_EXPOSURE_MODE, REG_EXPOSURE_AUTO,
    REG_LINE_SELECTOR, REG_LINE_MODE, REG_LINE_INVERTER,
    REG_TRANSFER_CONTROL_MODE, REG_TRANSFER_START, REG_TRANSFER_STOP,
    REG_TRANSFER_ABORT, REG_TRANSFER_BLOCK_COUNT,
}

MEMORY_SIZE = 0x10000               # register space
XML_BASE = 0x10000                  # GenApi XML mapped read-only here


# ------------------------------------------------------------
# address -> name, so diagnostic logs read as "WRITEREG SCDA = ..."
# rather than "WRITEREG 0x00000d18 = ...". Listed explicitly rather than
# scraped from globals(), so bit masks and capability flags can never be
# mistaken for addresses.
# ------------------------------------------------------------

_ADDRESS_NAMES = (
    "VERSION", "DEVICE_MODE", "MAC_HIGH", "MAC_LOW", "SUPPORTED_IP_CONFIG",
    "CURRENT_IP_CONFIG", "CURRENT_IP", "SUBNET_MASK", "GATEWAY",
    "MANUFACTURER_NAME", "MODEL_NAME", "DEVICE_VERSION", "MANUFACTURER_INFO",
    "SERIAL_NUMBER", "USER_DEFINED_NAME", "FIRST_URL", "SECOND_URL",
    "NUM_NETWORK_INTERFACES", "PERSISTENT_IP", "PERSISTENT_SUBNET",
    "PERSISTENT_GATEWAY", "LINK_SPEED", "NUM_MESSAGE_CHANNELS",
    "NUM_STREAM_CHANNELS", "NUM_ACTION_SIGNALS", "SC_CAPABILITY",
    "MESSAGE_CHANNEL_CAPABILITY", "GVCP_CAPABILITY", "HEARTBEAT_TIMEOUT",
    "TICK_FREQ_HIGH", "TICK_FREQ_LOW", "TIMESTAMP_CONTROL",
    "TIMESTAMP_LATCHED_HIGH", "TIMESTAMP_LATCHED_LOW", "DISCOVERY_ACK_DELAY",
    "GVCP_CONFIGURATION", "PENDING_TIMEOUT", "CONTROL_SWITCHOVER_KEY",
    "GVSP_CONFIGURATION", "CCP", "PRIMARY_APP_PORT", "PRIMARY_APP_IP",
    "MCP", "MCDA", "MC_TIMEOUT", "MC_RETRY_COUNT", "MC_SOURCE_PORT",
    "SCP", "SCPS", "SCPD", "SCDA", "SCSP", "SCC", "SCCFG",
    "REG_WIDTH", "REG_HEIGHT", "REG_PIXEL_FORMAT", "REG_ACQUISITION_COMMAND",
    "REG_ACQUISITION_MODE", "REG_SENSOR_WIDTH", "REG_SENSOR_HEIGHT",
    "REG_FRAME_PERIOD_US", "REG_PAYLOAD_SIZE", "REG_EXPOSURE_TIME_US",
    "REG_GAIN_RAW", "REG_OFFSET_X", "REG_OFFSET_Y", "REG_TRIGGER_MODE",
    "REG_TRIGGER_SOFTWARE", "REG_DEVICE_RESET", "REG_TRIGGER_SELECTOR",
    "REG_TRIGGER_SOURCE", "REG_TRIGGER_ACTIVATION", "REG_TRIGGER_DELAY_US",
    "REG_GAIN_SELECTOR", "REG_GAIN_AUTO", "REG_EXPOSURE_MODE",
    "REG_EXPOSURE_AUTO", "REG_LINE_SELECTOR", "REG_LINE_MODE",
    "REG_LINE_INVERTER", "REG_LINE_STATUS", "REG_LINE_STATUS_ALL",
    "REG_TRANSFER_CONTROL_MODE", "REG_TRANSFER_START", "REG_TRANSFER_STOP",
    "REG_TRANSFER_ABORT", "REG_TRANSFER_BLOCK_COUNT",
    "REG_TRANSFER_QUEUE_COUNT",
)

REGISTER_NAMES = {}
for _name in _ADDRESS_NAMES:
    REGISTER_NAMES.setdefault(globals()[_name], _name)


def register_name(address):
    """a readable name for a register address, for diagnostics."""
    if address in REGISTER_NAMES:
        return REGISTER_NAMES[address]
    if address >= XML_BASE:
        return f"XML+0x{address - XML_BASE:x}"
    return f"0x{address:08x}"


def format_register_value(address, value):
    """renders a value the way a human wants to read it."""
    if address in (CURRENT_IP, SUBNET_MASK, GATEWAY, SCDA, PERSISTENT_IP,
                   PERSISTENT_SUBNET, PERSISTENT_GATEWAY, MCDA, PRIMARY_APP_IP):
        return u32_to_ip(value)
    if address in (SCP, MCP):
        return f"port {value & 0xFFFF}"
    if address == SCPS:
        flags = []
        if value & SCPS_FIRE_TEST_PACKET:
            flags.append("FIRE_TEST")
        if value & SCPS_DO_NOT_FRAGMENT:
            flags.append("DO_NOT_FRAGMENT")
        size = value & SCPS_SIZE_MASK
        return f"{size} bytes" + (f" [{'|'.join(flags)}]" if flags else "")
    return f"{value} (0x{value:08x})"


def stream_reg(base, channel=0):
    return base + STREAM_CHANNEL_STRIDE * channel


def ip_to_u32(text):
    parts = [int(p) for p in text.split(".")]
    if len(parts) != 4 or any(not 0 <= p <= 255 for p in parts):
        raise ValueError(f"bad ipv4 address: {text}")
    return struct.unpack(">I", bytes(parts))[0]


def u32_to_ip(value):
    return ".".join(str(b) for b in struct.pack(">I", value))


def mac_to_bytes(text):
    parts = text.replace("-", ":").split(":")
    if len(parts) != 6:
        raise ValueError(f"bad mac address: {text}")
    return bytes(int(p, 16) for p in parts)


class RegisterFile:
    """the device's flat register memory, plus the GenApi XML mapped
    read-only immediately above it (which is what a `Local:` URL
    pointing at XML_BASE resolves to)."""

    def __init__(self, size=MEMORY_SIZE, xml_base=XML_BASE):
        self.mem = bytearray(size)
        self.xml_base = xml_base
        self.xml_blob = b""

    # -------------------- xml region -------------------- #

    def set_xml(self, blob):
        self.xml_blob = bytes(blob)

    @property
    def xml_size(self):
        return len(self.xml_blob)

    # ------------------ typed accessors ------------------ #

    def read_u32(self, address):
        if address + 4 > len(self.mem):
            return 0
        return struct.unpack_from(">I", self.mem, address)[0]

    def write_u32(self, address, value):
        struct.pack_into(">I", self.mem, address, value & 0xFFFFFFFF)

    def read_u64(self, address):
        return (self.read_u32(address) << 32) | self.read_u32(address + 4)

    def write_string(self, address, text, length):
        raw = text.encode("utf-8")[:length - 1]
        self.mem[address:address + length] = raw.ljust(length, b"\x00")

    def read_string(self, address, length):
        return self.mem[address:address + length].split(b"\x00")[0].decode(
            "utf-8", errors="replace")

    def write_bytes(self, address, data):
        self.mem[address:address + len(data)] = data

    # ------------------- block access ------------------- #

    def read(self, address, count):
        """READMEM: serves register space or the XML blob, zero-filling
        past the end of either. HALCON reads counts of 512, 380, 64,
        32, 20 and 8 in practice, so arbitrary counts must work."""
        if address >= self.xml_base:
            offset = address - self.xml_base
            chunk = self.xml_blob[offset:offset + count]
        else:
            chunk = bytes(self.mem[address:address + count])
        return chunk.ljust(count, b"\x00")

    def write(self, address, data):
        """WRITEMEM into register space only; the XML is read-only."""
        if address >= self.xml_base:
            return False
        end = address + len(data)
        if end > len(self.mem):
            return False
        self.mem[address:end] = data
        return True

    def discovery_data(self):
        """the 248-byte DISCOVERY_ACK payload: a verbatim copy of
        bootstrap 0x0000..0x00F7."""
        return bytes(self.mem[0:0xF8])

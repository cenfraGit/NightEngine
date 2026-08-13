# protocol.py
# pure codec for the Gardasoft CC320 Trigger Timing Controller ASCII
# protocol: tokenising, number parsing, reply framing, error codes, and the
# output-mode table. No sockets, no OpenGL, no controller state.
#
# Read directly from the CC320 User Manual Version 015 (April 2023).
# Beware: the CC320 is a TRIGGER TIMING controller, and although it shares
# command letters with the RT/PP LED *lighting* controllers, several of
# them mean different things -- and Err 5 differs outright. Do not carry
# assumptions across from the lighting range.

import re

# ------------------------------------------------------------
# framing (§10.1-10.2)
# ------------------------------------------------------------

CR = "\r"
LF = "\n"
PROMPT = ">"

TCP_PORT = 30313            # commands, TCP and UDP
DISCOVERY_PORT = 30311      # device listens for enquiries here
DISCOVERY_REPLY_PORT = 30310  # ...and replies here
DISCOVERY_REQUEST = "Gardasoft Search"   # manual says 13 chars; it is 16
TCP_IDLE_TIMEOUT = 10.0     # a real unit closes an idle connection

# ------------------------------------------------------------
# error codes (Appendix B) -- CC320-specific
# ------------------------------------------------------------

ERR_PARAMETER = 1           # a parameter value is invalid
ERR_UNKNOWN_COMMAND = 2     # command not recognised
ERR_NUMERIC_FORMAT = 3      # numeric value is in the wrong format
ERR_PARAMETER_COUNT = 4     # wrong number of parameters
ERR_EEPROM_READ = 5         # NOTE: on RT/PP this code is "warning, clamped"
ERR_EEPROM_CORRUPT = 6
ERR_SETTINGS_READ = 8
ERR_SETTINGS_SAVE = 9
ERR_RESYNC_NOT_FOUND = 13
ERR_ETHERNET_HARDWARE = 49
ERR_FIFO_EXHAUSTED = 81

ERROR_REASONS = {
    ERR_PARAMETER: "A parameter value is invalid",
    ERR_UNKNOWN_COMMAND: "Command not recognised",
    ERR_NUMERIC_FORMAT: "Numeric value is in the wrong format",
    ERR_PARAMETER_COUNT: "Wrong number of parameters",
    ERR_EEPROM_READ: "Unable to read the EEPROM",
    ERR_EEPROM_CORRUPT: "EEPROM corrupt. The configuration has been cleared",
    ERR_SETTINGS_READ: "Unable to read settings from the EEPROM",
    ERR_SETTINGS_SAVE: "Unable to save settings to the EEPROM",
    ERR_RESYNC_NOT_FOUND: "SN command: The resync event cannot be found",
    ERR_ETHERNET_HARDWARE: "Ethernet hardware not working",
    ERR_FIFO_EXHAUSTED: "Too many FIFO events have been used",
}


class GardasoftError(Exception):
    """raised by a command handler; becomes an `Err n` reply."""

    def __init__(self, code):
        super().__init__(f"Err {code}: {ERROR_REASONS.get(code, 'unknown')}")
        self.code = code


# ------------------------------------------------------------
# output modes (§7.2)
# ------------------------------------------------------------

MODE_SET_LOW = 0
MODE_SET_HIGH = 1
MODE_PULSE_TT = 2
MODE_PULSE_TE = 3
MODE_PULSE_ET = 4
MODE_PULSE_EE = 5
MODE_BURST_T = 8
MODE_FREQUENCY = 9          # "Mode 9 is not supported" -- documented dead
MODE_BUFFER_T = 10
MODE_BUFFER_E = 11
MODE_BURST_E = 12
MODE_COUNTER = 13
MODE_MIN_PULSE_TRIG = 14
MODE_MAX_PULSE_TRIG = 15
MODE_DTYPE_LATCH = 16

MODE_NAMES = {
    MODE_SET_LOW: "OFF", MODE_SET_HIGH: "On",
    MODE_PULSE_TT: "Ptt", MODE_PULSE_TE: "PtE",
    MODE_PULSE_ET: "PEt", MODE_PULSE_EE: "PEE",
    6: "mode6", 7: "mode7",
    MODE_BURST_T: "bur", MODE_FREQUENCY: "FrE",
    MODE_BUFFER_T: "buF", MODE_BUFFER_E: "buE",
    MODE_BURST_E: "brE", MODE_COUNTER: "CoU",
    MODE_MIN_PULSE_TRIG: "iPF", MODE_MAX_PULSE_TRIG: "APF",
    MODE_DTYPE_LATCH: "dLA",
}

MODE_MIN, MODE_MAX = 0, 16

# modes that emit a burst of pulses. In these the RS gate parameter is the
# PULSE COUNT rather than a gate source, and RT's two slots mean
# width-then-gap rather than width-then-delay. Getting this wrong silently
# produces the wrong waveform, so it is centralised here.
BURST_MODES = frozenset({MODE_BURST_T, MODE_BURST_E})
BURST_MAX_PULSES = 250

# modes that emit a pulse when triggered (as opposed to holding a level)
PULSE_MODES = frozenset({MODE_PULSE_TT, MODE_PULSE_TE, MODE_PULSE_ET,
                         MODE_PULSE_EE, MODE_BURST_T, MODE_BURST_E})

# modes whose delay/width are counted in encoder pulses rather than time
ENCODER_DELAY_MODES = frozenset({MODE_PULSE_ET, MODE_PULSE_EE, MODE_BUFFER_E})
ENCODER_WIDTH_MODES = frozenset({MODE_PULSE_TE, MODE_PULSE_EE, MODE_BURST_E})


def rt_parameter_names(mode):
    """what RTc,p,d's two arguments mean for this mode.

    Argument ORDER is width-then-second-slot. The manual contradicts itself
    (§10.3 body says width first; the §10.4 summary and §7.2 tables say
    delay first) but a working rig configuration settles it: five outputs in
    Mode 2 sharing one trigger were configured RT1,1s,0 / RT2,1s,1s /
    RT3,1s,2s / RT4,1s,3s / RT5,1s,4s -- a constant first argument with a
    ramping second can only be constant WIDTH with increasing DELAY."""
    if mode in BURST_MODES:
        return ("width", "gap")
    return ("width", "delay")


def gate_is_pulse_count(mode):
    """in burst modes the RS `g` slot is the number of pulses, not a gate."""
    return mode in BURST_MODES


# ------------------------------------------------------------
# flags (§7.3, confirmed from the logic-operation examples where
# I,O,G == 7 and I,O == 3)
# ------------------------------------------------------------

FLAG_I = 1
FLAG_O = 2                  # invert the output
FLAG_G = 4                  # gate
FLAG_E = 8                  # ethernet message
FLAG_F = 16                 # FIFO
FLAG_R = 32                 # resync
FLAG_P = 64                 # pulse

# ST renders flags as these letters, lower case when clear and upper when
# set, in this order
FLAG_LETTERS = [(FLAG_I, "i"), (FLAG_O, "o"), (FLAG_G, "g"), (FLAG_E, "e"),
                (FLAG_F, "f"), (FLAG_R, "r"), (FLAG_P, "p")]


def format_flags(flags):
    """'iogefrp' with set flags upper-cased, as ST prints them."""
    return "".join(letter.upper() if flags & bit else letter
                   for bit, letter in FLAG_LETTERS)


# ------------------------------------------------------------
# channels and sources
# ------------------------------------------------------------

PHYSICAL_OUTPUTS = 8        # OP1..OP8
VIRTUAL_OUTPUTS = 8         # OP9..OP16
TOTAL_OUTPUTS = PHYSICAL_OUTPUTS + VIRTUAL_OUTPUTS
INPUTS = 8                  # IP1..IP8 (IP0 is the free-running timer)

SOURCE_TIMER = 0            # trigger: internal free-running timer; gate: none


def decode_source(value):
    """resolves an RS trigger/gate source number.

    0 = internal timer (trigger) or none (gate), 1-8 = IP1-IP8,
    9-16 = OP1-OP8, 17-24 = virtual outputs OP9-OP16. The manual states the
    rule as 'the input channel number is calculated as 8 + the output
    channel number'."""
    if value == SOURCE_TIMER:
        return ("timer", 0)
    if 1 <= value <= INPUTS:
        return ("input", value)
    if INPUTS + 1 <= value <= INPUTS + TOTAL_OUTPUTS:
        return ("output", value - INPUTS)
    raise GardasoftError(ERR_PARAMETER)


def encode_source(kind, index):
    if kind == "timer":
        return SOURCE_TIMER
    if kind == "input":
        return index
    if kind == "output":
        return INPUTS + index
    raise ValueError(f"unknown source kind {kind!r}")


# ------------------------------------------------------------
# number parsing (§10.2)
# ------------------------------------------------------------

_NUMBER = re.compile(r"^[+-]?(\d+\.?\d*|\.\d+)$")

# longest suffixes first: 'ms' and 'us' must be tested before 's'
_TIME_SCALES_MS = [("us", 0.001), ("µs", 0.001), ("ms", 1.0), ("s", 1000.0)]
_COUNT_SCALES = [("k", 1_000), ("m", 1_000_000)]


def _to_number(text):
    if not _NUMBER.match(text):
        # a decimal comma cannot even reach us (comma is the separator),
        # but anything else malformed lands here
        raise GardasoftError(ERR_NUMERIC_FORMAT)
    return float(text)


def parse_time_ms(text):
    """a time parameter in milliseconds. Bare numbers default to ms."""
    token = text.strip().lower()
    if not token:
        raise GardasoftError(ERR_NUMERIC_FORMAT)
    for suffix, scale in _TIME_SCALES_MS:
        if token.endswith(suffix) and len(token) > len(suffix):
            return _to_number(token[:-len(suffix)]) * scale
    return _to_number(token)


def parse_count(text):
    """an encoder-pulse count. Accepts K (x1000) and M (x1000000)."""
    token = text.strip().lower()
    if not token:
        raise GardasoftError(ERR_NUMERIC_FORMAT)
    for suffix, scale in _COUNT_SCALES:
        if token.endswith(suffix) and len(token) > len(suffix):
            return int(round(_to_number(token[:-len(suffix)]) * scale))
    return int(round(_to_number(token)))


def parse_integer(text, minimum=None, maximum=None):
    value = _to_number(text.strip())
    if value != int(value):
        raise GardasoftError(ERR_NUMERIC_FORMAT)
    value = int(value)
    if minimum is not None and value < minimum:
        raise GardasoftError(ERR_PARAMETER)
    if maximum is not None and value > maximum:
        raise GardasoftError(ERR_PARAMETER)
    return value


def format_time_ms(milliseconds):
    """ST renders timings like '100.00ms'."""
    return f"{milliseconds:.2f}ms"


# ------------------------------------------------------------
# tokenising
# ------------------------------------------------------------

class Command:
    """one parsed command: a two-letter code plus its arguments.

    `echo` is the normalised text the device echoes back ahead of any reply
    data."""

    __slots__ = ("code", "args", "echo")

    def __init__(self, code, args, echo):
        self.code = code
        self.args = args
        self.echo = echo

    def __repr__(self):
        return f"Command({self.code}{',' + ','.join(self.args) if self.args else ''})"

    def arity(self, *allowed):
        if len(self.args) not in allowed:
            raise GardasoftError(ERR_PARAMETER_COUNT)


_CODE = re.compile(r"^([A-Za-z]{2})(.*)$", re.DOTALL)


def split_line(line):
    """splits one received line into commands on ';'. Spaces are ignored
    everywhere, per the manual."""
    stripped = line.replace(CR, "").replace(LF, "")
    stripped = stripped.replace(" ", "").replace("\t", "")
    return [part for part in stripped.split(";") if part]


def parse_command(text):
    """parses a single (already space-stripped) command."""
    match = _CODE.match(text)
    if not match:
        raise GardasoftError(ERR_UNKNOWN_COMMAND)
    code = match.group(1).upper()
    remainder = match.group(2)
    args = remainder.split(",") if remainder else []
    # a trailing comma means an empty final argument, which is malformed
    if any(arg == "" for arg in args):
        raise GardasoftError(ERR_PARAMETER_COUNT)
    return Command(code, args, code + remainder)


def parse_line(line):
    """-> list of Command. Raises on the first malformed command."""
    return [parse_command(part) for part in split_line(line)]


# ------------------------------------------------------------
# reply framing
# ------------------------------------------------------------

def format_reply(echo, data=""):
    """one command's reply: the echoed command, then any reply data.

    The line terminator and prompt are appended once per received line by
    format_line_reply, not per command."""
    return f"{echo}{data}"


def format_error(echo, code):
    """an error replaces the reply data: 'VTErr 2'."""
    return f"{echo}Err {code}"


def format_line_reply(parts):
    """assembles the replies for one received line.

    The frame is <replies><LF><CR>'>' -- LF *then* CR, then the prompt as
    the final byte. That ordering is unusual and is confirmed both by the
    manual and by the EPICS StreamDevice definition
    (InTerminator = LF CR ">")."""
    return "".join(parts) + LF + CR + PROMPT


def format_value(value):
    """reads reply with a VL prefix: RI1 -> 'VL1', EN -> 'VL200'."""
    return f"VL{value}"


# ------------------------------------------------------------
# discovery (§8.3-8.4)
# ------------------------------------------------------------

def is_discovery_request(payload):
    """lenient on purpose: the manual mis-states the length as 13
    characters when the string is 16, so never length-check."""
    if isinstance(payload, bytes):
        payload = payload.decode("ascii", errors="ignore")
    return payload.strip().startswith(DISCOVERY_REQUEST)


def format_discovery_reply(model, serial, mac, ip):
    """'Gardasoft,CC320,<serial 6>,<MAC 12 hex>,<IP 8 hex>'

    mac is 'aa:bb:cc:dd:ee:ff' or bare hex; ip is dotted quad."""
    mac_hex = mac.replace(":", "").replace("-", "").upper()
    octets = [int(part) for part in ip.split(".")]
    ip_hex = "".join(f"{octet:02X}" for octet in octets)
    return f"Gardasoft,{model},{int(serial):06d},{mac_hex},{ip_hex}"

# test_gardasoft_protocol.py
# offline unit tests for the Gardasoft CC320 ASCII codec. No sockets, no
# OpenGL. Everything asserted here traces to the CC320 User Manual v015 or
# to a working configuration captured from real hardware.
#
# run:  python tests/test_gardasoft_protocol.py

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from NightEngine.Gardasoft import protocol as P


def expect_error(code, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except P.GardasoftError as error:
        assert error.code == code, f"expected Err {code}, got Err {error.code}"
        return
    raise AssertionError(f"expected Err {code}, no error raised")


# ------------------------------------------------------------
# tokenising
# ------------------------------------------------------------

def test_spaces_are_ignored_everywhere():
    commands = P.parse_line("RS 1 , 2 , 1 , 0 , 0\r")
    assert len(commands) == 1
    assert commands[0].code == "RS"
    assert commands[0].args == ["1", "2", "1", "0", "0"]


def test_semicolon_chains_commands():
    commands = P.parse_line("RE0;RB1,0;AW\r")
    assert [c.code for c in commands] == ["RE", "RB", "AW"]
    assert commands[0].args == ["0"]
    assert commands[1].args == ["1", "0"]
    assert commands[2].args == []


def test_command_letters_are_case_insensitive_and_normalised():
    command = P.parse_command("rs1,2,1,0,0")
    assert command.code == "RS"
    assert command.echo.startswith("RS")


def test_unknown_shaped_input_is_err2():
    expect_error(P.ERR_UNKNOWN_COMMAND, P.parse_command, "1")
    expect_error(P.ERR_UNKNOWN_COMMAND, P.parse_command, "X")


def test_trailing_comma_is_err4():
    expect_error(P.ERR_PARAMETER_COUNT, P.parse_command, "RT1,1s,")


def test_arity_check():
    command = P.parse_command("RT1,1s,0")
    command.arity(3)                     # fine
    expect_error(P.ERR_PARAMETER_COUNT, command.arity, 2)


# ------------------------------------------------------------
# number parsing
# ------------------------------------------------------------

def test_time_defaults_to_milliseconds():
    assert P.parse_time_ms("40") == 40.0
    assert P.parse_time_ms("0") == 0.0
    assert P.parse_time_ms("0.5") == 0.5


def test_time_suffixes():
    assert P.parse_time_ms("1s") == 1000.0
    assert P.parse_time_ms("0.1s") == 100.0
    assert P.parse_time_ms("50ms") == 50.0
    assert P.parse_time_ms("200us") == 0.2
    assert P.parse_time_ms("1000us") == 1.0
    assert P.parse_time_ms("500US") == 0.5, "suffixes are case-insensitive"


def test_the_rigs_own_timings_parse_correctly():
    """straight from the working configuration."""
    assert P.parse_time_ms("1s") == 1000.0        # RT1,1s,0
    assert P.parse_time_ms("4s") == 4000.0        # RT5,1s,4s
    assert P.parse_time_ms("50ms") == 50.0        # RT6,50ms,0.1s
    assert P.parse_time_ms("0.1s") == 100.0
    assert P.parse_time_ms("0.5s") == 500.0       # RT8,0.5s,0.5s
    assert P.parse_time_ms("1ms") == 1.0          # RR5,1ms


def test_encoder_counts_accept_k_and_m():
    assert P.parse_count("5K") == 5000
    assert P.parse_count("5k") == 5000
    assert P.parse_count("2M") == 2_000_000
    assert P.parse_count("250") == 250


def test_malformed_numbers_are_err3():
    for bad in ("abc", "1.2.3", "", "1s2", "--4"):
        expect_error(P.ERR_NUMERIC_FORMAT, P.parse_time_ms, bad)


def test_decimal_comma_cannot_masquerade_as_a_value():
    """the manual insists on '0.5' not '0,5'. A comma is the separator, so
    '0,5' must arrive as two arguments rather than one bad number."""
    command = P.parse_command("RB1,0,5")
    assert command.args == ["1", "0", "5"]


def test_integer_range_checks():
    assert P.parse_integer("8", minimum=1, maximum=8) == 8
    expect_error(P.ERR_PARAMETER, P.parse_integer, "9", minimum=1, maximum=8)
    expect_error(P.ERR_PARAMETER, P.parse_integer, "0", minimum=1, maximum=8)
    expect_error(P.ERR_NUMERIC_FORMAT, P.parse_integer, "1.5")


# ------------------------------------------------------------
# reply framing
# ------------------------------------------------------------

def test_reply_frame_is_lf_then_cr_then_prompt():
    """unusual ordering, confirmed by the manual and by the EPICS
    StreamDevice definition InTerminator = LF CR '>'."""
    frame = P.format_line_reply([P.format_reply("VR", "CC320 (HW02) V029")])
    assert frame == "VRCC320 (HW02) V029\n\r>"
    assert frame.endswith("\n\r>")
    assert frame[-1] == ">"


def test_reply_echoes_the_command():
    assert P.format_reply("RS1,2,1,0,0") == "RS1,2,1,0,0"


def test_error_replaces_the_reply_data():
    assert P.format_error("VT", P.ERR_UNKNOWN_COMMAND) == "VTErr 2"


def test_chained_commands_share_one_trailing_prompt():
    frame = P.format_line_reply([P.format_reply("RE0"),
                                 P.format_reply("RB1,0"),
                                 P.format_reply("AW")])
    assert frame == "RE0RB1,0AW\n\r>"
    assert frame.count(">") == 1


def test_reads_use_a_vl_prefix():
    assert P.format_value(0) == "VL0"
    assert P.format_value(1) == "VL1"
    assert P.format_value(200) == "VL200"


# ------------------------------------------------------------
# modes -- the burst trap
# ------------------------------------------------------------

def test_mode_names_match_the_manual():
    assert P.MODE_NAMES[0] == "OFF"
    assert P.MODE_NAMES[1] == "On"
    assert P.MODE_NAMES[2] == "Ptt"
    assert P.MODE_NAMES[8] == "bur"
    assert P.MODE_NAMES[10] == "buF"
    assert P.MODE_NAMES[12] == "brE"
    assert P.MODE_NAMES[16] == "dLA"


def test_rt_parameters_are_width_then_delay_for_pulse_modes():
    assert P.rt_parameter_names(P.MODE_PULSE_TT) == ("width", "delay")


def test_rt_parameters_become_width_then_gap_in_burst_modes():
    """Mode 8/12 reinterpret the second slot as the gap between pulses."""
    assert P.rt_parameter_names(P.MODE_BURST_T) == ("width", "gap")
    assert P.rt_parameter_names(P.MODE_BURST_E) == ("width", "gap")


def test_gate_slot_is_a_pulse_count_only_in_burst_modes():
    assert P.gate_is_pulse_count(P.MODE_BURST_T) is True
    assert P.gate_is_pulse_count(P.MODE_BURST_E) is True
    assert P.gate_is_pulse_count(P.MODE_PULSE_TT) is False
    assert P.gate_is_pulse_count(P.MODE_BUFFER_T) is False


def test_mode_9_is_documented_as_unsupported():
    assert P.MODE_FREQUENCY == 9
    assert P.MODE_NAMES[9] == "FrE"


# ------------------------------------------------------------
# flags
# ------------------------------------------------------------

def test_flag_bits_match_the_logic_examples():
    """the manual's NAND example shows flags 'I, O, G (7)' and NOR 'I, O (3)',
    and the gated-pulse example shows 'G (4)'."""
    assert P.FLAG_I | P.FLAG_O | P.FLAG_G == 7
    assert P.FLAG_I | P.FLAG_O == 3
    assert P.FLAG_G == 4


def test_flags_render_lower_when_clear_and_upper_when_set():
    assert P.format_flags(0) == "iogefrp"
    assert P.format_flags(P.FLAG_G) == "ioGefrp"
    assert P.format_flags(7) == "IOGefrp"


# ------------------------------------------------------------
# trigger / gate sources
# ------------------------------------------------------------

def test_source_zero_is_the_internal_timer():
    assert P.decode_source(0) == ("timer", 0)


def test_sources_one_to_eight_are_physical_inputs():
    assert P.decode_source(1) == ("input", 1)
    assert P.decode_source(8) == ("input", 8)


def test_sources_nine_to_sixteen_are_physical_outputs():
    """the manual states the rule as 'the input channel number is
    calculated as 8 + the input channel number' -- so OP4 is source 12."""
    assert P.decode_source(9) == ("output", 1)
    assert P.decode_source(12) == ("output", 4)
    assert P.decode_source(16) == ("output", 8)


def test_sources_seventeen_to_twentyfour_are_virtual_outputs():
    assert P.decode_source(17) == ("output", 9)
    assert P.decode_source(24) == ("output", 16)


def test_out_of_range_source_is_err1():
    expect_error(P.ERR_PARAMETER, P.decode_source, 25)
    expect_error(P.ERR_PARAMETER, P.decode_source, -1)


def test_source_roundtrip():
    for kind, index in [("timer", 0), ("input", 3), ("output", 4), ("output", 16)]:
        assert P.decode_source(P.encode_source(kind, index)) == (kind, index)


# ------------------------------------------------------------
# discovery
# ------------------------------------------------------------

def test_discovery_request_is_matched_leniently():
    assert P.is_discovery_request(b"Gardasoft Search")
    assert P.is_discovery_request("Gardasoft Search\x00")
    assert P.is_discovery_request(b"Gardasoft Search" + b"\x00" * 8)
    assert not P.is_discovery_request(b"something else")


def test_discovery_reply_matches_the_manuals_worked_example():
    """manual: serial 012345, IP 192.168.1.103, MAC 00.0B.75.01.80.99 ->
    'Gardasoft,CC320,012345,000B75018099,C0A80167'. (The manual prints the
    IP with only 7 hex digits, which is a typo; 8 is correct.)"""
    reply = P.format_discovery_reply("CC320", 12345, "00:0B:75:01:80:99",
                                    "192.168.1.103")
    assert reply == "Gardasoft,CC320,012345,000B75018099,C0A80167"


def test_discovery_ports_are_asymmetric():
    assert P.DISCOVERY_PORT == 30311
    assert P.DISCOVERY_REPLY_PORT == 30310
    assert P.TCP_PORT == 30313


# ------------------------------------------------------------
# error codes differ from the RT/PP lighting range
# ------------------------------------------------------------

def test_err5_is_an_eeprom_failure_not_a_clamp_warning():
    """on RT/PP controllers Err 5 means 'warning, value clamped'. On the
    CC320 it means the EEPROM could not be read. Carrying the lighting
    meaning across would misreport a hard failure as a warning."""
    assert P.ERROR_REASONS[P.ERR_EEPROM_READ] == "Unable to read the EEPROM"


def test_the_four_command_errors_are_1_to_4():
    assert (P.ERR_PARAMETER, P.ERR_UNKNOWN_COMMAND,
            P.ERR_NUMERIC_FORMAT, P.ERR_PARAMETER_COUNT) == (1, 2, 3, 4)


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

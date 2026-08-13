# test_gardasoft_controller.py
# Offline tests for the emulated CC320. No sockets, no OpenGL.
#
# The centrepiece is test_real_rig_configuration_produces_the_expected_cycle,
# which feeds in the exact command sequence used on the real hardware and
# asserts the resulting waveform: five 1-second light windows back to back
# with a camera pulse at the top of each.
#
# run:  python tests/test_gardasoft_controller.py

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from NightEngine.Gardasoft import protocol as P
from NightEngine.Gardasoft.controller import NightGardasoftCC320

# the working "limp mode recovery" configuration from the real rig
RIG_CONFIG = """
RE0; RB1,0;
RS1,2,1,0,0; RT1,1s,0;    RR1,0;
RS2,2,1,0,0; RT2,1s,1s;   RR2,0;
RS3,2,1,0,0; RT3,1s,2s;   RR3,0;
RS4,2,1,0,0; RT4,1s,3s;   RR4,0;
RS5,2,1,0,0; RT5,1s,4s;   RR5,1ms;
RS6,8,6,5,0; RT6,50ms,0.1s; RR6,0;
RS7,8,7,5,0; RT7,50ms,0.1s; RR7,0;
RS8,8,1,5,0; RT8,0.5s,0.5s; RR8,0;
AW
"""


def load_rig(controller):
    for line in RIG_CONFIG.strip().splitlines():
        reply = controller.execute_line(line)
        assert "Err" not in reply, f"{line.strip()} -> {reply!r}"
    for index in range(9, 17):              # park the virtual channels
        controller.execute_line(f"RS{index},0,0,0,0;RT{index},0,0;RR{index},0")


def new_controller():
    controller = NightGardasoftCC320()
    controller.execute_line("CL")
    return controller


def levels_over_time(controller, outputs, duration, step=0.02, start_at=0.0):
    """samples output levels while marching time forward."""
    samples = []
    t = start_at
    while t <= start_at + duration + 1e-9:
        controller.evaluate(t)
        samples.append((round(t - start_at, 4),
                        {op: controller.output_level(op) for op in outputs}))
        t += step
    return samples


def windows(samples, output):
    """-> [(rise, fall)] for one output, from sampled levels."""
    result, start = [], None
    for t, levels in samples:
        if levels[output] and start is None:
            start = t
        elif not levels[output] and start is not None:
            result.append((start, t))
            start = None
    if start is not None:
        result.append((start, samples[-1][0]))
    return result


# ------------------------------------------------------------
# the real rig
# ------------------------------------------------------------

def test_real_rig_configuration_loads_without_error():
    controller = new_controller()
    load_rig(controller)
    assert controller.timer_period_ms == 0.0, "RB1,0 must disable the timer"
    assert controller.encoder_mode == 0, "RE0 must disable the encoder"
    for index in range(1, 6):
        channel = controller.channels[index]
        assert channel.mode == P.MODE_PULSE_TT
        assert channel.trigger == 1, "lights are all triggered by IP1"
        assert channel.width == 1000.0, "each light is lit for 1s"
        assert channel.second == (index - 1) * 1000.0, "delays ramp 0..4s"
    camera = controller.channels[8]
    assert camera.mode == P.MODE_BURST_T
    assert camera.trigger == 1
    assert camera.burst_count == 5, "g=5 is the pulse COUNT in burst mode"
    assert camera.width == 500.0
    assert camera.second == 500.0, "the second slot is the GAP in burst mode"


def test_real_rig_configuration_produces_the_expected_cycle():
    """one pulse on IP1 -> five 1s light windows, camera pulse at each top."""
    controller = new_controller()
    load_rig(controller)

    controller.evaluate(0.0)
    controller.mi(1, 1)                     # MI1,1 : assert IP1
    controller.evaluate(0.0)
    controller.mi(1, 0)                     # MI1,0 : release it

    samples = levels_over_time(controller, [1, 2, 3, 4, 5, 8], duration=5.5)

    for index in range(1, 6):
        found = windows(samples, index)
        assert len(found) == 1, f"light {index} should fire once, got {found}"
        rise, fall = found[0]
        expected_rise = (index - 1) * 1.0
        assert abs(rise - expected_rise) < 0.05, \
            f"light {index} rose at {rise}s, expected {expected_rise}s"
        assert abs((fall - rise) - 1.0) < 0.05, \
            f"light {index} was lit for {fall - rise}s, expected 1s"

    # the five windows must tile without gaps or overlap
    rises = [windows(samples, i)[0][0] for i in range(1, 6)]
    gaps = [round(rises[i + 1] - rises[i], 2) for i in range(4)]
    assert all(abs(gap - 1.0) < 0.05 for gap in gaps), f"window spacing {gaps}"

    camera_pulses = windows(samples, 8)
    assert len(camera_pulses) == 5, \
        f"camera should burst 5 times, got {len(camera_pulses)}"
    for index, (rise, fall) in enumerate(camera_pulses):
        assert abs(rise - index * 1.0) < 0.05, \
            f"camera pulse {index} at {rise}s, expected {index}.0s"
        assert abs((fall - rise) - 0.5) < 0.05, \
            f"camera pulse {index} lasted {fall - rise}s, expected 0.5s"

    # each camera pulse must land inside its own light's window
    for index in range(5):
        light_rise, light_fall = windows(samples, index + 1)[0]
        camera_rise, _ = camera_pulses[index]
        assert light_rise - 0.05 <= camera_rise < light_fall, \
            f"camera pulse {index} at {camera_rise}s is outside light " \
            f"{index + 1}'s window {light_rise}-{light_fall}s"


def test_only_one_light_is_lit_at_any_instant():
    """the whole point of the sweep: exactly one light per capture."""
    controller = new_controller()
    load_rig(controller)
    controller.evaluate(0.0)
    controller.mp(1)
    for _, levels in levels_over_time(controller, [1, 2, 3, 4, 5], 5.0):
        lit = sum(levels.values())
        assert lit <= 1, f"{lit} lights lit simultaneously"


def test_op6_and_op7_are_independent_of_the_photometric_cycle():
    """they are triggered by IP6/IP7, so a pulse on IP1 must not fire them."""
    controller = new_controller()
    load_rig(controller)
    controller.evaluate(0.0)
    controller.mp(1)
    samples = levels_over_time(controller, [6, 7], 5.0)
    assert not windows(samples, 6), "OP6 fired from IP1"
    assert not windows(samples, 7), "OP7 fired from IP1"

    # time only ever moves forward: continue from where the sweep ended,
    # otherwise pulses scheduled at t=5s are invisible when sampling from 0
    resume = 5.0
    controller.evaluate(resume)
    controller.mp(6)
    samples = levels_over_time(controller, [6, 7], 1.0, step=0.005,
                               start_at=resume)
    bursts = windows(samples, 6)
    assert len(bursts) == 5, f"OP6 should burst 5 times, got {len(bursts)}"
    assert abs((bursts[0][1] - bursts[0][0]) - 0.05) < 0.02, "50ms pulses"
    assert abs((bursts[1][0] - bursts[0][0]) - 0.15) < 0.02, \
        "period should be width 50ms + gap 100ms"


# ------------------------------------------------------------
# MI / MP semantics
# ------------------------------------------------------------

def test_mi_sets_a_level_and_a_rising_edge_triggers():
    controller = new_controller()
    controller.execute_line("RS1,2,1,0,0;RT1,100,0;RR1,0")
    controller.evaluate(0.0)
    assert controller.output_level(1) == 0
    controller.execute_line("MI1,1")
    controller.evaluate(0.0)
    assert controller.output_level(1) == 1, "rising edge on IP1 should fire OP1"


def test_mi_holding_high_does_not_retrigger():
    """level-triggered would fire repeatedly; the CC320 is edge-triggered."""
    controller = new_controller()
    controller.execute_line("RS1,2,1,0,0;RT1,50,0;RR1,0")
    controller.evaluate(0.0)
    controller.execute_line("MI1,1")
    samples = levels_over_time(controller, [1], 0.5, step=0.01)
    assert len(windows(samples, 1)) == 1, "held-high input must fire once only"


def test_mi_pulse_between_frames_is_not_lost():
    """the real failure this caused: a host sends MI1,1 immediately followed
    by MI1,0, both landing between two render frames. Comparing levels at
    evaluation time would see the input low both times and lose the trigger,
    so rising edges are latched instead."""
    controller = new_controller()
    load_rig(controller)
    controller.evaluate(0.0)

    controller.execute_line("MI1,1")
    controller.execute_line("MI1,0")        # no evaluate() in between
    controller.evaluate(0.01)

    assert controller.output_level(1) == 1, \
        "the latched rising edge should still have fired OP1"


def test_a_single_latched_edge_reaches_every_channel_sharing_the_input():
    """OP1..OP5 and OP8 all trigger from IP1, so one pulse must fire all of
    them -- the latch must not be consumed by whichever channel sees it
    first."""
    controller = new_controller()
    load_rig(controller)
    controller.evaluate(0.0)
    controller.execute_line("MI1,1;MI1,0")
    controller.evaluate(0.0)
    assert controller.output_level(1) == 1, "OP1 (light 1) did not fire"
    assert controller.output_level(8) == 1, "OP8 (camera) did not fire"
    for index in (2, 3, 4, 5):
        assert controller.channels[index].pulses, \
            f"OP{index} was not scheduled by the shared edge"


def test_mp_synthesises_a_complete_pulse():
    controller = new_controller()
    controller.execute_line("RS1,2,1,0,0;RT1,100,0;RR1,0")
    controller.evaluate(0.0)
    controller.execute_line("MP1")
    assert controller.input_level(1) == 0, "MP must leave the input released"
    controller.evaluate(0.0)                # the latched edge fires here
    assert controller.output_level(1) == 1


def test_a_real_edge_cancels_an_mi_override():
    """documented: 'The override is cancelled as soon as an edge is detected
    on the input.'"""
    controller = new_controller()
    controller.mi(3, 1)
    assert controller.input_level(3) == 1
    controller.set_physical_input(3, 1)
    assert 3 not in controller.input_overrides, "override should be cleared"


def test_mi_rejects_out_of_range_channels():
    controller = new_controller()
    assert "Err 1" in controller.execute_line("MI0,1")
    assert "Err 1" in controller.execute_line("MI9,1")


# ------------------------------------------------------------
# trigger graph
# ------------------------------------------------------------

def test_an_output_can_be_chained_from_another_output():
    """source 9..16 means OP1..OP8: 'the input channel number is calculated
    as 8 + the output channel number'."""
    controller = new_controller()
    controller.execute_line("RS1,2,1,0,0;RT1,100,0")
    controller.execute_line("RS2,2,9,0,0;RT2,100,200")   # OP2 from OP1, +200ms
    controller.evaluate(0.0)
    controller.mp(1)
    samples = levels_over_time(controller, [1, 2], 0.6, step=0.01)
    first = windows(samples, 1)
    second = windows(samples, 2)
    assert len(first) == 1 and len(second) == 1, f"{first} {second}"
    assert abs(second[0][0] - 0.2) < 0.03, \
        f"chained output should start 200ms later, got {second[0][0]}s"


def test_virtual_outputs_can_relay_triggers():
    """virtual channels 9-16 have no physical output but are otherwise
    identical, and are addressed as sources 17-24."""
    controller = new_controller()
    controller.execute_line("RS9,2,1,0,0;RT9,100,0")      # virtual relay
    controller.execute_line("RS1,2,17,0,0;RT1,100,100")   # OP1 from OP9
    controller.evaluate(0.0)
    controller.mp(1)
    samples = levels_over_time(controller, [1, 9], 0.5, step=0.01)
    assert len(windows(samples, 9)) == 1, "virtual channel did not fire"
    assert len(windows(samples, 1)) == 1, "relay through a virtual channel failed"


def test_a_trigger_cycle_does_not_hang():
    controller = new_controller()
    controller.execute_line("RS1,2,10,0,0;RT1,10,0")      # OP1 <- OP2
    controller.execute_line("RS2,2,9,0,0;RT2,10,0")       # OP2 <- OP1
    controller.evaluate(0.0)
    controller.evaluate(0.1)          # must simply return


def test_retrigger_time_blocks_a_second_trigger():
    controller = new_controller()
    controller.execute_line("RS1,2,1,0,0;RT1,10,0;RR1,200ms")
    controller.evaluate(0.0)
    controller.mp(1)
    controller.evaluate(0.0)          # first trigger fires: pulse 0..10ms
    controller.evaluate(0.05)         # ...and has ended by now
    assert controller.output_level(1) == 0, "the first pulse should be over"

    controller.mp(1)                  # 50ms in, inside the 200ms guard
    samples = levels_over_time(controller, [1], 0.1, step=0.005, start_at=0.05)
    assert len(windows(samples, 1)) == 0, "retriggered inside the guard window"

    # ...and is accepted again once the guard has elapsed
    controller.evaluate(0.30)
    controller.mp(1)
    controller.evaluate(0.31)
    assert controller.output_level(1) == 1, "should retrigger after the guard"


def test_invert_flag_inverts_the_output():
    controller = new_controller()
    controller.execute_line(f"RS1,1,0,0,{P.FLAG_O}")      # Set High, inverted
    controller.evaluate(0.0)
    assert controller.output_level(1) == 0


def test_internal_timer_fires_repeatedly():
    controller = new_controller()
    controller.execute_line("RS1,2,0,0,0;RT1,50,0;RB1,200ms")
    samples = levels_over_time(controller, [1], 1.0, step=0.01)
    fired = windows(samples, 1)
    assert 4 <= len(fired) <= 6, f"expected ~5 pulses at 5Hz, got {len(fired)}"


# ------------------------------------------------------------
# commands and framing
# ------------------------------------------------------------

def test_vr_reports_model_and_firmware():
    controller = new_controller()
    reply = controller.execute_line("VR")
    assert reply == "VRCC320 (HW02) V029\n\r>"


def test_st_reports_the_configured_channels():
    controller = new_controller()
    load_rig(controller)
    reply = controller.execute_line("ST")
    assert "No encoder, trigger period = 0.000s" in reply
    assert "OP1: MD=2, IP=1, GT=-, DL=0.00ms, PL=1000.00ms" in reply
    assert "OP5: MD=2, IP=1, GT=-, DL=4000.00ms, PL=1000.00ms" in reply
    assert "OP8: MD=8, IP=1, GT=5" in reply, "burst count shows in the GT slot"
    assert reply.endswith("\n\r>")


def test_st_single_channel():
    controller = new_controller()
    load_rig(controller)
    reply = controller.execute_line("ST3")
    assert reply.startswith("ST3OP3:")
    assert "OP4" not in reply


def test_unknown_command_errors_and_is_logged_verbatim():
    """Gardasoft ship undocumented commands, and MI itself was absent from
    an earlier manual revision. Log rather than guess."""
    controller = new_controller()
    reply = controller.execute_line("ZZ1,2")
    assert reply == "ZZ1,2Err 2\n\r>"
    assert "ZZ1,2" in controller.unknown_commands


def test_reads_return_vl_values():
    controller = new_controller()
    controller.mi(2, 1)
    assert controller.execute_line("RI2") == "RI2VL1\n\r>"
    assert controller.execute_line("RI1") == "RI1VL0\n\r>"
    assert controller.execute_line("RO1") == "RO1VL0\n\r>"


def test_encoder_count_wraps_at_2_to_the_32():
    controller = new_controller()
    assert controller.execute_line("EN") == "ENVL0\n\r>"
    controller.execute_line("EN1,25")
    assert controller.execute_line("EN") == "ENVL25\n\r>"
    controller.execute_line("EN0,10")
    assert controller.execute_line("EN") == "ENVL15\n\r>"
    controller.execute_line("EN0,40")
    assert controller.execute_line("EN") == f"ENVL{2**32 - 25}\n\r>"


def test_gr_reports_then_clears_the_last_error():
    controller = new_controller()
    controller.execute_line("ZZ")                # provoke Err 2
    assert "Err 2" in controller.execute_line("GR")
    assert "Err 0" in controller.execute_line("GR")


def test_aw_and_cl_manage_persistence():
    controller = new_controller()
    controller.execute_line("RS1,2,1,0,0;RT1,123,45;AW")
    controller.execute_line("CL")
    assert controller.channels[1].mode == P.MODE_SET_LOW
    controller.restore_saved()
    assert controller.channels[1].mode == P.MODE_PULSE_TT
    assert controller.channels[1].width == 123.0


def test_chained_line_yields_one_prompt():
    controller = new_controller()
    reply = controller.execute_line("RE0;RB1,0;AW")
    assert reply.count(">") == 1
    assert reply.endswith("\n\r>")


class LineRecorder:
    """a camera stand-in that records the effective level of one line over
    time, applying LineInverter the way NightGigEDevice does."""

    def __init__(self, inverter=0):
        self.inverter = inverter
        self.level = 0
        self.rises = []
        self.now = 0.0

    def set_line(self, line, level):
        level = 1 if level else 0
        effective = level ^ self.inverter
        if effective and not self.level:
            self.rises.append(round(self.now, 2))
        self.level = effective


def _line_rises(inverter):
    """runs the real rig with OP8 wired to a camera's Line2, and returns
    the times at which the camera sees a rising edge."""
    controller = new_controller()
    load_rig(controller)
    camera = LineRecorder(inverter=inverter)
    controller.bind_camera_line(output=8, device=camera, line=2)

    controller.evaluate(0.0)
    controller.mi(1, 1)
    controller.evaluate(0.0)
    controller.mi(1, 0)

    t = 0.0
    while t <= 5.5 + 1e-9:
        camera.now = t
        controller.evaluate(t)
        t += 0.02
    return camera.rises


def test_output_drives_a_camera_line_as_a_level_not_an_edge():
    """bind_camera_line mirrors OP8's level, so the camera does its own
    edge detection. Uninverted, it sees a rise at the start of each of the
    five 0.5s pulses."""
    rises = _line_rises(inverter=0)
    # the very first evaluate(0.0) happens before the trigger, so ignore a
    # rise recorded at exactly the same instant the pulse begins twice
    assert len(rises) == 5, f"expected 5 rising edges, got {rises}"
    for index, when in enumerate(rises):
        assert abs(when - index * 1.0) < 0.05, \
            f"edge {index} at {when}s, expected {index}.0s"


def test_line_inverter_moves_the_capture_to_mid_window():
    """the customer's script sets LineInverter 1 on Line2 with the default
    RisingEdge. Inverting means the camera's effective rise lands on OP8's
    FALLING edge -- 0.5s into each 1s light window rather than at its
    edge. That is very plausibly deliberate: the exposure sits where the
    light is stable. This is the concrete, checkable consequence."""
    inverted = _line_rises(inverter=1)
    plain = _line_rises(inverter=0)

    # an inverted, idle-low line reads high immediately, so the first
    # recorded rise is that initial state rather than a pulse
    pulses = [t for t in inverted if t > 0.05]
    assert len(pulses) == 5, f"expected 5 inverted edges, got {inverted}"
    for index, when in enumerate(pulses):
        expected = index * 1.0 + 0.5
        assert abs(when - expected) < 0.05, \
            f"inverted edge {index} at {when}s, expected {expected}s (mid-window)"

    # and clearing the inverter moves them back to the window edge
    for plain_t, inverted_t in zip(plain, pulses):
        assert abs((inverted_t - plain_t) - 0.5) < 0.05, \
            f"inversion shifted the capture by {inverted_t - plain_t}s, not 0.5s"


def test_cold_boot_matches_the_documented_defaults():
    controller = NightGardasoftCC320()
    assert controller.timer_period_ms == 1000.0, "IP0 at 1Hz"
    for index in range(1, 6):
        assert controller.channels[index].trigger == index
        assert controller.channels[index].second == 100.0
    assert controller.channels[7].second == 200.0
    assert controller.channels[8].second == 300.0
    assert controller.channels[6].trigger == 0, "OP6 comes from IP0"


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

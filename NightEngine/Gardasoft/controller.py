# controller.py
# An emulated Gardasoft CC320 Trigger Timing Controller: 16 output channels
# (8 physical + 8 virtual), 8 edge-detecting inputs, an internal free-running
# timer, and the trigger graph that connects them.
#
# No sockets and no OpenGL: server.py owns the sockets and calls evaluate()
# from the render thread. That keeps the whole thing unit-testable offline.
#
# TIMING MODEL (deliberate simplification): a real CC320 resolves pulses to
# ~1us, while our render loop advances in ~16ms frames. We model the LOGICAL
# state -- which outputs are asserted at each evaluation -- rather than the
# sub-frame waveform. For the photometric cycle this is ample: its light
# windows are 1s and its camera pulses 0.5s apart, so hundreds of frames fall
# inside each pulse.

import copy

from NightEngine.Gardasoft import protocol as P

MODEL = "CC320"
FIRMWARE = "(HW02) V029"


class Channel:
    """one output channel's configuration and runtime state."""

    __slots__ = ("index", "mode", "trigger", "gate", "width", "second",
                 "retrigger", "flags", "pulses", "level", "previous_level",
                 "last_trigger", "forced")

    def __init__(self, index):
        self.index = index
        self.mode = P.MODE_SET_LOW
        self.trigger = 0
        self.gate = 0
        self.width = 0.0        # ms, or encoder counts in encoder modes
        self.second = 0.0       # delay, or gap between pulses in burst modes
        self.retrigger = 0.0
        self.flags = 0
        self.pulses = []        # scheduled [start, end] windows, seconds
        self.level = 0
        self.previous_level = 0
        self.last_trigger = None
        self.forced = None      # RVc,v temporary override

    @property
    def is_physical(self):
        return self.index <= P.PHYSICAL_OUTPUTS

    @property
    def burst_count(self):
        """in burst modes the RS gate slot carries the pulse count."""
        if not P.gate_is_pulse_count(self.mode):
            return 1
        return max(1, min(int(self.gate), P.BURST_MAX_PULSES))

    def config_snapshot(self):
        return (self.mode, self.trigger, self.gate, self.width, self.second,
                self.retrigger, self.flags)

    def restore(self, snapshot):
        (self.mode, self.trigger, self.gate, self.width, self.second,
         self.retrigger, self.flags) = snapshot


class NightGardasoftCC320:
    """the emulated controller. Drive it by calling evaluate(now) regularly
    and injecting input activity with mi()/mp() or the ASCII commands."""

    def __init__(self, ip="127.0.0.1", serial=12345,
                 mac="00:0B:75:01:80:99", name="cc320"):
        self.ip = ip
        self.serial = serial
        self.mac = mac
        self.name = name

        self.channels = {index: Channel(index)
                         for index in range(1, P.TOTAL_OUTPUTS + 1)}
        self.input_levels = {index: 0 for index in range(1, P.INPUTS + 1)}
        self.input_overrides = {}          # set by MI, cleared by a real edge
        self.input_previous = dict(self.input_levels)
        self._input_edge_pending = set()   # rising edges awaiting evaluation

        self.timer_period_ms = 0.0         # RB1,p; 0 disables
        self.encoder_mode = 0              # REe
        self.encoder_count = 0
        self.keyboard_disabled = 0
        self.ethernet_messages = 0
        self.web_password = ""
        self.last_error = 0
        self.pass_fail = {}                # SNc,t,p

        self._timer_next = None
        self._timer_previous = 0
        self._saved = None
        self._now = 0.0
        self._order = None                 # cached topological order
        self.unknown_commands = []         # logged, never guessed at

        # scene bindings, filled in by the scene script
        self._light_bindings = {}          # output -> (light index, colour)
        self._camera_bindings = {}         # output -> callable
        # (output, device id, line) -> (output, device, line). Keyed so one
        # output can drive several cameras, and one camera several lines.
        self._line_bindings = {}
        self.output_edge_log = []          # (time, output) rising edges

        self.cold_boot()

    # ------------------------------------------------------------
    # power-on state (§7.6)
    # ------------------------------------------------------------

    def cold_boot(self):
        """the documented cold-boot configuration."""
        for channel in self.channels.values():
            channel.__init__(channel.index)
        self.timer_period_ms = 1000.0      # IP0 free-running at 1Hz
        self.encoder_mode = 0
        for index in range(1, 6):          # OP1..OP5 from IP1..IP5
            channel = self.channels[index]
            channel.mode = P.MODE_PULSE_TT
            channel.trigger = index
            channel.second = 100.0
            channel.width = 100.0
        for index, delay in ((6, 100.0), (7, 200.0), (8, 300.0)):
            channel = self.channels[index]
            channel.mode = P.MODE_PULSE_TT
            channel.trigger = 0            # from the internal timer
            channel.second = delay
            channel.width = 100.0
        self._order = None
        self.save()

    def clear(self):
        """CL: clear all configuration."""
        for channel in self.channels.values():
            channel.__init__(channel.index)
        self.timer_period_ms = 0.0
        self.encoder_mode = 0
        self._order = None

    def save(self):
        """AW: nothing survives a restart without this."""
        self._saved = {
            "channels": {i: c.config_snapshot() for i, c in self.channels.items()},
            "timer_period_ms": self.timer_period_ms,
            "encoder_mode": self.encoder_mode,
        }

    def restore_saved(self):
        if not self._saved:
            return
        for index, snapshot in self._saved["channels"].items():
            self.channels[index].restore(snapshot)
        self.timer_period_ms = self._saved["timer_period_ms"]
        self.encoder_mode = self._saved["encoder_mode"]
        self._order = None

    # ------------------------------------------------------------
    # scene bindings
    # ------------------------------------------------------------

    def bind_light(self, output, light_index, color=(1.0, 1.0, 1.0)):
        self._light_bindings[output] = (light_index, tuple(color))

    def bind_camera(self, output, callback):
        """callback() runs on each rising edge of that output."""
        self._camera_bindings[output] = callback

    def bind_camera_line(self, output, device, line):
        """wires an output to a camera's physical input line.

        Unlike bind_camera this mirrors the output's *level*, not its
        rising edge. That distinction matters: the camera does its own
        edge detection against its own TriggerActivation and LineInverter,
        so it must see the line go down as well as up. An edge-only
        binding cannot express FallingEdge, AnyEdge or the level modes."""
        self._line_bindings[(output, id(device), line)] = (output, device, line)

    # ------------------------------------------------------------
    # inputs
    # ------------------------------------------------------------

    def mi(self, channel, value):
        """MIc,v -- override an input's level. The manual: 'The override is
        cancelled as soon as an edge is detected on the input.'

        Rising edges are LATCHED rather than inferred by comparing levels at
        evaluation time. Real firmware samples inputs at microsecond
        resolution, so a host sending MI1,1 immediately followed by MI1,0
        produces a genuine edge; our evaluate() only runs once per rendered
        frame and would otherwise never observe the input high."""
        level = 1 if value else 0
        previous = self.input_level(channel)
        self.input_overrides[channel] = level
        if level == 1 and previous == 0:
            self._input_edge_pending.add(channel)

    def mp(self, channel):
        """MPi -- synthesise a pulse on an input (or on IP0, the
        free-running timer, when channel is 0)."""
        if channel == 0:
            self._fire_timer_edge()
            return
        self.mi(channel, 1)                 # latches the rising edge
        self.input_overrides[channel] = 0   # released immediately; the latch
                                            # survives until the next evaluate

    def set_physical_input(self, channel, level):
        """a real electrical edge, which cancels any MI override."""
        level = 1 if level else 0
        previous = self.input_level(channel)
        if self.input_levels[channel] != level:
            self.input_overrides.pop(channel, None)
        self.input_levels[channel] = level
        if level == 1 and previous == 0:
            self._input_edge_pending.add(channel)

    def input_level(self, channel):
        if channel in self.input_overrides:
            return self.input_overrides[channel]
        return self.input_levels.get(channel, 0)

    def _fire_timer_edge(self):
        self._timer_previous = 0
        self._timer_pulse = True

    # ------------------------------------------------------------
    # the trigger graph
    # ------------------------------------------------------------

    def _topological_order(self):
        """channels ordered so a channel is evaluated after any channel it
        depends on. Cycles are broken by leaving the offending channel last,
        so a bad configuration degrades rather than hanging."""
        if self._order is not None:
            return self._order
        dependencies = {}
        for index, channel in self.channels.items():
            needs = set()
            for source in (channel.trigger, channel.gate):
                kind, target = P.decode_source(source) if 0 <= source <= 24 else ("timer", 0)
                if kind == "output" and target != index:
                    needs.add(target)
            dependencies[index] = needs
        order, placed = [], set()
        remaining = set(self.channels)
        while remaining:
            ready = sorted(i for i in remaining if dependencies[i] <= placed)
            if not ready:                   # cycle: emit the rest in order
                order.extend(sorted(remaining))
                break
            order.extend(ready)
            placed.update(ready)
            remaining -= set(ready)
        self._order = order
        return order

    def _source_level(self, source):
        kind, index = P.decode_source(source)
        if kind == "timer":
            return getattr(self, "_timer_level", 0)
        if kind == "input":
            return self.input_level(index)
        return self.channels[index].level

    def _source_previous(self, source):
        kind, index = P.decode_source(source)
        if kind == "timer":
            return self._timer_previous
        if kind == "input":
            return self.input_previous.get(index, 0)
        return self.channels[index].previous_level

    def _source_edge(self, source):
        """did this source see a rising edge to act on?

        For inputs, a latched edge counts even if the level has already gone
        back down -- otherwise a MI1,1/MI1,0 pair sent between two frames
        would be missed entirely."""
        kind, index = P.decode_source(source)
        if kind == "input" and index in self._input_edge_pending:
            return True
        return (self._source_level(source) == 1
                and self._source_previous(source) == 0)

    def _gate_open(self, channel):
        """gate source 0 means ungated. NOTE: the manual's single gated
        example implies gate-HIGH blocks, but its wording is ambiguous and
        the rig's configuration never gates, so we take the plain reading
        (gate must be high to enable) and flag it as unverified."""
        if channel.gate == 0 or P.gate_is_pulse_count(channel.mode):
            return True
        try:
            return bool(self._source_level(channel.gate))
        except P.GardasoftError:
            return True

    def _schedule(self, channel, now):
        """queue this channel's pulse windows for a trigger at `now`."""
        if channel.retrigger and channel.last_trigger is not None:
            if (now - channel.last_trigger) * 1000.0 < channel.retrigger:
                return
        # burst modes cannot be retriggered mid-sequence
        if channel.pulses and P.gate_is_pulse_count(channel.mode):
            if now < channel.pulses[-1][1]:
                return
        channel.last_trigger = now
        width = max(channel.width, 0.0) / 1000.0

        if P.gate_is_pulse_count(channel.mode):
            gap = max(channel.second, 0.0) / 1000.0
            period = width + gap
            for pulse in range(channel.burst_count):
                start = now + pulse * period
                channel.pulses.append([start, start + width])
        else:
            delay = max(channel.second, 0.0) / 1000.0
            channel.pulses.append([now + delay, now + delay + width])

    def evaluate(self, now):
        """advances the controller to time `now` (seconds, monotonic)."""
        self._now = now

        # ---------------- internal timer (IP0) ---------------- #
        pulse_requested = getattr(self, "_timer_pulse", False)
        self._timer_pulse = False
        self._timer_previous = getattr(self, "_timer_level", 0)
        level = 0
        if self.timer_period_ms > 0:
            period = self.timer_period_ms / 1000.0
            if self._timer_next is None:
                self._timer_next = now + period
                level = 1
            elif now >= self._timer_next:
                self._timer_next = now + period
                level = 1
        if pulse_requested:
            level = 1
        self._timer_level = level

        # ---------------- channels, in dependency order ---------------- #
        for index in self._topological_order():
            channel = self.channels[index]
            channel.previous_level = channel.level

            if channel.mode == P.MODE_SET_LOW:
                channel.level = 0
            elif channel.mode == P.MODE_SET_HIGH:
                channel.level = 1
            elif channel.mode in P.PULSE_MODES:
                try:
                    fired = self._source_edge(channel.trigger)
                except P.GardasoftError:
                    fired = False
                if fired and self._gate_open(channel):
                    self._schedule(channel, now)
                channel.pulses = [w for w in channel.pulses if w[1] > now]
                channel.level = 1 if any(w[0] <= now < w[1]
                                         for w in channel.pulses) else 0
            elif channel.mode in (P.MODE_BUFFER_T, P.MODE_BUFFER_E):
                try:
                    channel.level = self._source_level(channel.trigger)
                except P.GardasoftError:
                    channel.level = 0
            else:
                channel.level = 0

            if channel.flags & P.FLAG_O:
                channel.level = 0 if channel.level else 1
            if channel.forced is not None:
                channel.level = channel.forced

            if channel.level and not channel.previous_level:
                self.output_edge_log.append((now, index))
                callback = self._camera_bindings.get(index)
                if callback:
                    callback()

        # mirror output levels onto camera input lines. Done after every
        # channel has settled, so a camera never samples a half-updated
        # controller. The camera detects its own edges from the level.
        for output, device, line in self._line_bindings.values():
            device.set_line(line, self.channels[output].level)

        # inputs are sampled once per evaluation for edge detection. Latched
        # edges are cleared only here, after every channel has had the chance
        # to see them -- OP1..OP5 and OP8 all trigger from IP1, so a single
        # pulse must reach all of them.
        self.input_previous = {i: self.input_level(i)
                               for i in range(1, P.INPUTS + 1)}
        self._input_edge_pending.clear()

    def apply_to_engine(self, engine):
        """maps output levels onto scene lights. Render thread only."""
        for output, (light_index, color) in self._light_bindings.items():
            if light_index >= len(engine.lights):
                continue
            on = bool(self.channels[output].level)
            engine.lights[light_index]["color"] = list(color) if on else [0.0, 0.0, 0.0]

    def output_level(self, output):
        return self.channels[output].level

    # ------------------------------------------------------------
    # ASCII command dispatch
    # ------------------------------------------------------------

    def execute_line(self, line):
        """handles one received line; returns the full framed reply."""
        parts = []
        try:
            commands = P.parse_line(line)
        except P.GardasoftError as error:
            self.last_error = error.code
            return P.format_line_reply([P.format_error("", error.code)])
        for command in commands:
            try:
                data = self._dispatch(command)
                parts.append(P.format_reply(command.echo, data))
            except P.GardasoftError as error:
                self.last_error = error.code
                parts.append(P.format_error(command.echo, error.code))
        return P.format_line_reply(parts)

    def _channel_arg(self, text, allow_virtual=True):
        limit = P.TOTAL_OUTPUTS if allow_virtual else P.PHYSICAL_OUTPUTS
        return P.parse_integer(text, minimum=1, maximum=limit)

    def _dispatch(self, command):
        handler = getattr(self, f"_cmd_{command.code.lower()}", None)
        if handler is None:
            self.unknown_commands.append(command.echo)
            raise P.GardasoftError(P.ERR_UNKNOWN_COMMAND)
        return handler(command)

    # -------------------- informational -------------------- #

    def _cmd_vr(self, command):
        command.arity(0)
        return f"{MODEL} {FIRMWARE}"

    def _cmd_st(self, command):
        command.arity(0, 1)
        if command.args:
            index = self._channel_arg(command.args[0])
            return self._format_channel(index)
        header = ("No encoder" if self.encoder_mode == 0
                  else f"Encoder mode {self.encoder_mode}")
        period = self.timer_period_ms / 1000.0
        lines = [f"{header}, trigger period = {period:.3f}s"]
        lines += [self._format_channel(i)
                  for i in range(1, P.TOTAL_OUTPUTS + 1)]
        return "\n".join(lines)

    def _format_channel(self, index):
        channel = self.channels[index]
        gate = "-" if channel.gate == 0 else str(channel.gate)
        return (f"OP{index}: MD={channel.mode}, IP={channel.trigger}, "
                f"GT={gate}, DL={P.format_time_ms(channel.second)}, "
                f"PL={P.format_time_ms(channel.width)}, "
                f"RT={P.format_time_ms(channel.retrigger)}, "
                f"{P.format_flags(channel.flags)}")

    def _cmd_gr(self, command):
        command.arity(0)
        code, self.last_error = self.last_error, 0
        return f"Err {code}"

    def _cmd_gt(self, command):
        command.arity(1)
        self.ethernet_messages = P.parse_integer(command.args[0], 0, 1)
        return ""

    def _cmd_ey(self, command):
        self.web_password = "".join(
            chr(P.parse_integer(a, 32, 126)) for a in command.args)
        return ""

    def _cmd_kb(self, command):
        command.arity(1)
        self.keyboard_disabled = P.parse_integer(command.args[0], 0, 1)
        return ""

    # -------------------- configuration -------------------- #

    def _cmd_cl(self, command):
        command.arity(0)
        self.clear()
        return ""

    def _cmd_aw(self, command):
        command.arity(0)
        self.save()
        return ""

    def _cmd_rs(self, command):
        command.arity(5)
        index = self._channel_arg(command.args[0])
        mode = P.parse_integer(command.args[1], P.MODE_MIN, P.MODE_MAX)
        trigger = P.parse_integer(command.args[2], 0, 24)
        gate_raw = P.parse_integer(command.args[3], 0,
                                   P.BURST_MAX_PULSES if mode in P.BURST_MODES else 24)
        flags = P.parse_integer(command.args[4], 0, 255)
        P.decode_source(trigger)
        if not P.gate_is_pulse_count(mode):
            P.decode_source(gate_raw)
        channel = self.channels[index]
        channel.mode = mode
        channel.trigger = trigger
        channel.gate = gate_raw
        channel.flags = flags
        channel.pulses = []
        channel.last_trigger = None
        self._order = None
        return ""

    def _cmd_rt(self, command):
        """RTc,p,d -- p is the pulse WIDTH, d the delay (or, in burst modes,
        the gap between pulses). The manual contradicts itself on the order;
        see protocol.rt_parameter_names for the evidence."""
        command.arity(3)
        index = self._channel_arg(command.args[0])
        channel = self.channels[index]
        encoder_width = channel.mode in P.ENCODER_WIDTH_MODES
        encoder_second = channel.mode in P.ENCODER_DELAY_MODES
        channel.width = (float(P.parse_count(command.args[1])) if encoder_width
                         else P.parse_time_ms(command.args[1]))
        channel.second = (float(P.parse_count(command.args[2])) if encoder_second
                          else P.parse_time_ms(command.args[2]))
        return ""

    def _cmd_rr(self, command):
        command.arity(2)
        index = self._channel_arg(command.args[0])
        self.channels[index].retrigger = P.parse_time_ms(command.args[1])
        return ""

    def _cmd_rb(self, command):
        command.arity(2)
        P.parse_integer(command.args[0], 1, 1)      # only timer 1 exists
        self.timer_period_ms = P.parse_time_ms(command.args[1])
        self._timer_next = None
        return ""

    def _cmd_re(self, command):
        command.arity(1)
        self.encoder_mode = P.parse_integer(command.args[0], 0, 2)
        return ""

    def _cmd_sn(self, command):
        command.arity(3)
        index = self._channel_arg(command.args[0])
        tag = P.parse_integer(command.args[1], 0)
        result = P.parse_integer(command.args[2], 0, 1)
        self.pass_fail[(index, tag)] = result
        return ""

    # -------------------- live control -------------------- #

    def _cmd_en(self, command):
        command.arity(0, 2)
        if not command.args:
            return P.format_value(self.encoder_count)
        direction = P.parse_integer(command.args[0], 0, 1)
        amount = P.parse_count(command.args[1])
        delta = amount if direction == 1 else -amount
        self.encoder_count = (self.encoder_count + delta) % (2 ** 32)
        return ""

    def _cmd_rv(self, command):
        command.arity(2)
        index = self._channel_arg(command.args[0])
        value = P.parse_integer(command.args[1], 0, 1)
        channel = self.channels[index]
        channel.forced = (0 if value else 1) if channel.flags & P.FLAG_O else value
        return ""

    def _cmd_ri(self, command):
        command.arity(1)
        index = P.parse_integer(command.args[0], 1, P.INPUTS)
        return P.format_value(self.input_level(index))

    def _cmd_ro(self, command):
        command.arity(1)
        index = self._channel_arg(command.args[0])
        return P.format_value(self.channels[index].level)

    def _cmd_mi(self, command):
        """MIc,v -- override an input's state. This is what a host uses to
        drive a cycle in software: MI1,1 then MI1,0 is a pulse on IP1."""
        command.arity(2)
        index = P.parse_integer(command.args[0], 1, P.INPUTS)
        value = P.parse_integer(command.args[1], 0, 1)
        self.mi(index, value)
        return ""

    def _cmd_mp(self, command):
        command.arity(1)
        index = P.parse_integer(command.args[0], 0, P.INPUTS)
        self.mp(index)
        return ""

    # ------------------------------------------------------------

    def info(self):
        return {"model": MODEL, "firmware": FIRMWARE, "ip": self.ip,
                "serial": self.serial, "mac": self.mac,
                "timer_period_ms": self.timer_period_ms,
                "encoder_mode": self.encoder_mode,
                "outputs": {i: c.level for i, c in self.channels.items()},
                "unknown_commands": list(self.unknown_commands)}

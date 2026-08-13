# hardware_trigger.py
#
# the smallest possible rig for testing an EMULATED HARDWARE TRIGGER.
#
# one GigE Vision camera watching a conveyor, and one key. pressing T
# pulses the camera's physical input Line2, exactly as a strobe
# controller's output would. there is no lighting controller, no light
# sweep and no calibration here on purpose: if a consumer configured for
# TriggerSource Line2 receives a frame, the hardware trigger works, and
# if it does not, there is nothing else in the picture to blame.
#
#   python hardware_trigger.py --ip 192.168.100.138 --log-gvcp
#
# then in HALCON (see hardware_trigger_halcon.txt):
#   TriggerSelector FrameStart / TriggerMode On / TriggerSource Line2
#   grab_image  ->  blocks until you press T
#
# the parts keep moving while the camera waits, so every triggered frame
# catches them further along the belt. two identical frames mean a stale
# buffer, not a fresh capture.

import os
import sys
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import glfw

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.Objects.NightObject import NightObject
from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Materials.NightMaterialLight import NightMaterialLight
from NightEngine.Meshes.MeshBox import MeshBox
from NightEngine.Meshes.MeshSphere import MeshSphere

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--ip", default="127.0.0.1",
                    help="local address to serve the camera on. must already "
                         "be assigned to an interface")
parser.add_argument("--resolution", default="640x480")
parser.add_argument("--no-bind-any", action="store_true",
                    help="bind the camera to --ip only, instead of 0.0.0.0. "
                         "binding 0.0.0.0 is what lets a consumer's discovery "
                         "broadcast reach us on any interface")
parser.add_argument("--log-gvcp", action="store_true",
                    help="print every GVCP command a consumer sends, with "
                         "register names resolved, plus why any frame was not "
                         "captured. Use this when a consumer connects but no "
                         "images arrive")
args = parser.parse_args()

# fail fast, before opening a window and a GL context, if the address
# cannot be bound -- the error is far clearer here than three services deep
from NightEngine.netutil import is_local_address, local_ipv4_addresses

if not is_local_address(args.ip):
    print(f"\n--ip {args.ip} is not assigned to any local interface, "
          f"so it cannot be bound.\n")
    print("Addresses this machine can bind right now:")
    for candidate in local_ipv4_addresses():
        print(f"    {candidate}")
    print('\nEither pass one of those, or add an alias:\n'
          f'    netsh interface ipv4 add address "Ethernet" {args.ip} '
          f'255.255.0.0\n'
          "(the adapter's primary address must be static, not DHCP)\n")
    sys.exit(2)

RESOLUTION = tuple(int(v) for v in args.resolution.lower().split("x"))

TRIGGER_KEY = glfw.KEY_T
TRIGGER_LINE = 2        # the camera input line our "controller" is wired to
PULSE_WIDTH = 0.05      # seconds the line is held high, like a strobe output

LIGHT_POSITION = [0, 12, 4]
LIGHT_COLOR = [1.6, 1.6, 1.5]   # steady: the trigger is the only variable here

BELT_SPEED = 4.0        # units/s the parts move at
BELT_END_X = 22.0       # parts recycle to the start once past this


class Example(NightBase):
    def setup(self):

        self.scene = self.create_scene()
        self.camera = NightCamera()
        self.camera.set_position([0, 22, 32])
        self.set_gravity(y=-20)

        self.light_directional["direction"] = [-0.3, -1.0, -0.4]

        # ------------------ floor and belt ------------------ #

        self.floor = NightObject(MeshBox(70, 2, 70, color=[0.45, 0.45, 0.48]),
                                 NightMaterialDefault())
        self.floor.set_position([0, -1, 0])
        self.scene.add(self.floor)

        self.belt = NightObject(MeshBox(50, 0.4, 8, color=[0.15, 0.15, 0.18]),
                                NightMaterialDefault())
        self.belt.set_position([0, 0.2, 0])
        self.scene.add(self.belt)

        # ------------------ parts on the belt ------------------ #

        self.parts = []
        specs = [
            (MeshBox(2.5, 2.5, 2.5, color=[0.75, 0.1, 0.1]), 2.0),
            (MeshSphere(1.4, 24,   color=[0.1, 0.55, 0.15]), 1.8),
            (MeshBox(3.0, 1.2, 2.0, color=[0.15, 0.25, 0.7]), 1.2),
            (MeshSphere(1.1, 24,   color=[0.8, 0.7, 0.1]), 1.5),
            (MeshBox(1.6, 3.2, 1.6, color=[0.6, 0.25, 0.6]), 2.4),
        ]
        spacing = (2 * BELT_END_X) / len(specs)
        for index, (mesh, y) in enumerate(specs):
            part = NightObject(mesh, NightMaterialDefault(), mass=1)
            part.set_position([-BELT_END_X + index * spacing, y, 0])
            self.scene.add(part)
            self.parts.append(part)

        # ------------------ a steady work light ------------------ #
        # deliberately NOT a strobe. the only thing that should vary
        # between triggered frames is where the parts are.

        self.light_point["position"] = list(LIGHT_POSITION)
        self.light_point["color"] = list(LIGHT_COLOR)

        bulb = NightObject(MeshSphere(0.5, 12, collision=False),
                           NightMaterialLight(color=[1.0, 1.0, 0.9]), 0)
        bulb.set_position(LIGHT_POSITION)
        bulb.cast_shadow = False
        self.scene.add(bulb)

        # ------------------ the camera under test ------------------ #

        self.inspect = self.add_vision_camera(
            name="inspect", position=[0, 18, 0], target=[0, 0, 0],
            resolution=RESOLUTION, fov=50,
            show_body=True, body_color=[0.9, 0.9, 0.2], frustum_depth=16)

        gige = self.start_gige_server([{
            "camera": self.inspect, "ip": args.ip,
            "mac": "02:00:" + ":".join(f"{int(o):02x}"
                                       for o in args.ip.split(".")),
            "vendor": "NightEngine", "model": "NightEngineCam",
            "serial": "NE00000200", "user_name": "inspect",
            "pixel_format": "Mono8", "frame_rate": 30.0,
        }], verbose=True, bind_any=not args.no_bind_any,
           log_gvcp=args.log_gvcp)

        self.device = gige.devices[0]
        self._pulse_until = 0.0
        self.pulses = 0

        # ------------------ what a consumer needs ------------------ #

        print("\nemulated hardware trigger ready.")
        print(f"  camera      : {args.ip}  (GigE Vision, control UDP 3956)")
        print(f"  device name : {self.device.halcon_name}")
        print(f"  trigger     : press T to pulse Line{TRIGGER_LINE} "
              f"({int(PULSE_WIDTH * 1000)} ms)")
        print("\nconfigure your consumer with:")
        print("  TriggerSelector    FrameStart")
        print("  TriggerMode        On")
        print(f"  TriggerSource      Line{TRIGGER_LINE}")
        print("  TriggerActivation  RisingEdge")
        print("\nthen grab: it blocks until you press T. one press, one frame.")
        if args.log_gvcp:
            print("\nGVCP logging is ON: every consumer command is printed "
                  "below, along with the reason any frame was not captured.")
        print()

    # ------------------------------------------------------------

    def physics_update(self, time_step):
        # drive the parts along the belt; recycle them at the end
        for part in self.parts:
            linear, _ = part.get_velocity()
            part.set_velocity(linear=[BELT_SPEED, linear[1], 0])
            if part.get_position()[0] > BELT_END_X:
                position = part.get_position()
                part.set_position([-BELT_END_X, position[1], position[2]])

    def update(self):
        # a momentary pulse, high then low, like a real strobe output. the
        # two halves land in different frames, which is fine and in fact
        # required: the device latches the edge itself, so even a stalled
        # render loop cannot lose the trigger.
        if self.key_just_pressed(TRIGGER_KEY):
            self.device.set_line(TRIGGER_LINE, 1)
            self._pulse_until = self.time + PULSE_WIDTH
            self.pulses += 1
            print(f"trigger {self.pulses}: Line{TRIGGER_LINE} high")
        if self._pulse_until and self.time >= self._pulse_until:
            self.device.set_line(TRIGGER_LINE, 0)
            self._pulse_until = 0.0

        self.camera.move(self.window, self.time_delta)
        self.draw_scene(self.camera)


if __name__ == "__main__":
    engine = Example()
    engine.run()

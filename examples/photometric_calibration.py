# photometric_calibration.py
#
# A photometric-calibration cell, entirely in software:
#
#   * a glossy hemisphere at scene centre
#   * a camera directly overhead looking down at it
#   * five lights on a ring around it
#   * an emulated Gardasoft CC320 trigger timing controller that fires the
#     lights one at a time AND triggers the camera once per light
#
# Each captured frame shows one specular highlight whose position on the dome
# encodes that light's direction, which is what light-source calibration
# solves for. Verified: the rendered highlight sits within 1px of both the
# Blinn-Phong maximum and the ideal mirror reflection point, so positions are
# geometrically exact.
#
# RUN IT
#   python photometric_calibration.py                  # loopback, local test
#   python photometric_calibration.py --ip 169.254.5.94 # on a real interface
#
# ADDRESSES
#   The camera and the controller can SHARE one address: GigE Vision uses UDP
#   3956 while the CC320 uses TCP/UDP 30313 and UDP 30311, so they do not
#   collide. --camera-ip is therefore optional and defaults to --ip; only pass
#   it if you specifically want them to look like two boxes on the network.
#
#   Whatever you pass must already be assigned to a local interface -- Windows
#   refuses to bind anything else (WinError 10049). Check with `ipconfig`, and
#   add an alias if you want a dedicated address:
#       netsh interface ipv4 add address "Ethernet" 169.254.5.60 255.255.0.0
#   (the adapter's primary address must be static, not DHCP, for that to be
#   accepted). Run with no arguments first: loopback always works.
#
#   DISCOVERABILITY: by default the camera listens on 0.0.0.0:3956 so a
#   consumer finds it whichever adapter it happens to scan, and --ip is what
#   the device ADVERTISES as its address. A socket bound to one specific
#   address only ever sees traffic sent to that address, which on a machine
#   with several adapters (very common with link-local 169.254.x) means a GUI
#   scanning a different adapter never sees the camera at all. The controller
#   already listens on all interfaces, so this keeps the two consistent. Pass
#   --no-bind-any to restrict the camera to --ip.
#
# START A CYCLE
#   Press T in the window (not SPACE -- that flies the camera up), or from
#   any machine:
#       printf 'MI1,1\r' | nc <ip> 30313      then   printf 'MI1,0\r' | nc <ip> 30313
#   or use examples/photometric_client.py, which does the whole sweep and
#   scores the result against ground truth.
#
# The controller is loaded with the same configuration used on the real rig,
# so one pulse on IP1 produces five 1-second light windows back to back with
# a camera pulse at the top of each.

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.Objects.NightObject import NightObject
from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Meshes.MeshBox import MeshBox
from NightEngine.Meshes.MeshSphere import MeshSphere
from NightEngine.Meshes.MeshHemisphere import MeshHemisphere
import glfw

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--ip", default="127.0.0.1",
                    help="local IP for the emulated CC320 controller")
parser.add_argument("--camera-ip", default=None,
                    help="separate local IP for the camera. Optional: the "
                         "camera (UDP 3956) and controller (30313) do not "
                         "collide, so by default they share --ip")
parser.add_argument("--lights", type=int, default=5,
                    help="lights on the ring; the real rig drives 5 (OP1..OP5)")
parser.add_argument("--resolution", default="1024x1024")
parser.add_argument("--no-http", action="store_true",
                    help="skip the controller's status web page")
parser.add_argument("--no-bind-any", action="store_true",
                    help="bind the camera and controller to --ip only, instead "
                         "of 0.0.0.0. The default listens on every interface so "
                         "a consumer reaches them whichever adapter it scans, "
                         "and is also immune to Windows dropping the route to "
                         "an APIPA address on a disconnected adapter")
parser.add_argument("--groundtruth", default="calibration_groundtruth.json")
# Identity. HALCON names a device "<mac><vendor>_<model>" with separators
# and spaces stripped, so setting these three makes an existing script's
# open_framegrabber call match the emulator without editing the script.
# Deliberately opt-in rather than default: an emulator should never be
# mistaken for real hardware by accident.
parser.add_argument("--vendor", default="NightEngine",
                    help="DeviceVendorName, e.g. 'Lucid Vision Labs'")
parser.add_argument("--model", default="NightEngineCam",
                    help="DeviceModelName, e.g. 'TRI050SM'")
parser.add_argument("--serial", default="NE00000100",
                    help="DeviceSerialNumber")
parser.add_argument("--mac", default=None,
                    help="MAC address, e.g. '1c:0f:af:7a:e6:bc'. Defaults to "
                         "one derived from the camera's IP")
parser.add_argument("--trigger-line", type=int, default=2, choices=(1, 2, 3, 4),
                    help="the camera input line OP8 is wired to. Set "
                         "TriggerSource to this line in your consumer to run "
                         "the camera in hardware-trigger mode (default: Line2, "
                         "matching the production rig)")
parser.add_argument("--log-gvcp", action="store_true",
                    help="print every GVCP command a consumer sends, with "
                         "register names resolved, plus why any frame was not "
                         "captured. Use this when a consumer connects but no "
                         "images arrive")
args = parser.parse_args()



RESOLUTION = tuple(int(v) for v in args.resolution.lower().split("x"))

# fail fast, before opening a window and a GL context, if an address cannot
# be bound -- the error is far clearer here than three services deep
from NightEngine.netutil import is_local_address, local_ipv4_addresses

for label, address in (("--ip", args.ip),
                       ("--camera-ip", args.camera_ip or args.ip)):
    if not is_local_address(address):
        print(f"\n{label} {address} is not assigned to any local interface, "
              f"so it cannot be bound.\n")
        print("Addresses this machine can bind right now:")
        for candidate in local_ipv4_addresses():
            print(f"    {candidate}")
        print('\nEither pass one of those, or add an alias:\n'
              f'    netsh interface ipv4 add address "Ethernet" {address} '
              f'255.255.0.0\n'
              "(the adapter's primary address must be static, not DHCP)\n")
        sys.exit(2)
DOME_RADIUS = 5.0
RING_RADIUS = 13.0
RING_HEIGHT = 9.0
CAMERA_HEIGHT = 22.0
CAMERA_FOV = 45.0

# the working configuration from the real rig: OP1..OP5 are the lights,
# each lit for 1s at delays 0..4s from one trigger on IP1; OP8 bursts five
# 0.5s camera pulses at 1s intervals, landing at the top of each window.
RIG_CONFIG = [
    "RE0", "RB1,0",
    "RS1,2,1,0,0", "RT1,1s,0", "RR1,0",
    "RS2,2,1,0,0", "RT2,1s,1s", "RR2,0",
    "RS3,2,1,0,0", "RT3,1s,2s", "RR3,0",
    "RS4,2,1,0,0", "RT4,1s,3s", "RR4,0",
    "RS5,2,1,0,0", "RT5,1s,4s", "RR5,1ms",
    "RS8,8,1,5,0", "RT8,0.5s,0.5s", "RR8,0",
] + [f"RS{i},0,0,0,0" for i in range(9, 17)] + ["AW"]

CAMERA_OUTPUT = 8
LIGHT_OUTPUTS = [1, 2, 3, 4, 5]


class Example(NightBase):
    def setup(self):
        self.scene = self.create_scene()

        # ------------------- viewing camera ------------------- #
        self.camera = NightCamera(fov=55)
        self.camera.look_at(position=[16, 16, 20], target=[0, 2, 0])

        # a dark cell: the ring lights should dominate, as in a real
        # light-tight enclosure
        self.light_directional["ambient"] = [0.04, 0.04, 0.04]
        self.light_directional["diffuse"] = [0.05, 0.05, 0.05]
        self.light_directional["specular"] = [0.0, 0.0, 0.0]
        self.light_directional["direction"] = [-0.3, -1.0, -0.2]
        self.shadows_enabled = False        # shadow maps track only the sun

        # ------------------- the cell ------------------- #
        floor_material = NightMaterialDefault()
        floor_material.specular = [0.0, 0.0, 0.0]      # matte, so the only
        floor_material.diffuse = [0.30, 0.30, 0.32]    # highlight is the dome's
        floor = NightObject(MeshBox(70, 2, 70, color=[0.3, 0.3, 0.32],
                                    collision=False), floor_material, 0)
        floor.set_position([0, -1, 0])
        self.scene.add(floor)

        # ------------------- the calibration target ------------------- #
        dome_material = NightMaterialDefault()
        dome_material.shininess = 250.0     # tight, well-localised highlight
        dome_material.diffuse = [0.12, 0.12, 0.12]
        dome_material.specular = [1.0, 1.0, 1.0]
        dome_material.ambient = [0.04, 0.04, 0.04]
        self.dome = NightObject(MeshHemisphere(DOME_RADIUS, 64,
                                              color=[0.85, 0.85, 0.85],
                                              collision=False),
                                dome_material, 0)
        self.scene.add(self.dome)

        # ------------------- ring lights ------------------- #
        self.set_light_count(args.lights)
        self.light_positions = []
        for index in range(args.lights):
            angle = 2 * math.pi * index / args.lights
            position = [RING_RADIUS * math.cos(angle), RING_HEIGHT,
                        RING_RADIUS * math.sin(angle)]
            light = self.lights[index]
            light["position"] = position
            light["color"] = [0.0, 0.0, 0.0]        # the controller drives these
            light["attenuation_linear"] = 0.0
            light["attenuation_quadratic"] = 0.0
            self.light_positions.append(position)

            # a small emissive marker so you can see where each light is
            marker = NightObject(MeshSphere(0.35, 12, color=[0.9, 0.9, 0.5],
                                            collision=False),
                                 NightMaterialDefault(lighting=False), 0)
            marker.set_position(position)
            marker.cast_shadow = False
            self.scene.add(marker)

        # ------------------- inspection camera ------------------- #
        self.inspection = self.add_vision_camera(
            name="photometric", position=[0, CAMERA_HEIGHT, 0.001],
            target=[0, 0, 0], resolution=RESOLUTION, fov=CAMERA_FOV,
            show_body=True, body_color=[0.3, 0.9, 0.9], frustum_depth=20)

        # ------------------- transports ------------------- #
        camera_ip = args.camera_ip or args.ip
        self.start_vision_server(host="127.0.0.1", port=8555)
        camera_mac = args.mac or (
            "02:00:" + ":".join(f"{int(o):02x}" for o in camera_ip.split(".")))
        self.start_gige_server([{
            "camera": self.inspection, "ip": camera_ip, "mac": camera_mac,
            "vendor": args.vendor, "model": args.model,
            "serial": args.serial, "user_name": "photometric",
            "pixel_format": "Mono8", "frame_rate": 30.0,
        }], verbose=True, bind_any=not args.no_bind_any,
           log_gvcp=args.log_gvcp)

        controller_server = self.start_gardasoft_controller(
            ip=args.ip, serial=680321, http_port=0 if args.no_http else 80,
            bind_any=not args.no_bind_any)
        controller = controller_server.controller

        # load the rig's own configuration
        for line in RIG_CONFIG:
            reply = controller.execute_line(line)
            if "Err" in reply:
                print(f"  controller rejected {line!r}: {reply!r}")

        # wire outputs to the scene
        for slot, output in enumerate(LIGHT_OUTPUTS[:args.lights]):
            controller.bind_light(output, slot, color=(3.0, 3.0, 3.0))
        # OP8 reaches the camera two ways, and which one fires depends on
        # how the consumer configured the camera:
        #   * as a wire onto a physical input line, for a consumer running
        #     the camera in hardware-trigger mode (TriggerSource LineN).
        #     The camera does its own edge detection, so LineInverter and
        #     TriggerActivation behave as they do on the real rig.
        #   * as a direct trigger, for a consumer using TriggerSource
        #     Software -- which is what our own photometric_client does.
        # The direct path stands down whenever a line is selected, so the
        # two can never double-trigger.
        for device in getattr(self.gige_server, "devices", []):
            controller.bind_camera_line(CAMERA_OUTPUT, device, args.trigger_line)
        controller.bind_camera(CAMERA_OUTPUT, self._fire_camera)
        self.trigger_line = args.trigger_line

        self.controller = controller
        self.captures = 0
        self._write_groundtruth()

        print("\nphotometric cell ready.")
        print(f"  controller : {args.ip}  (CC320, commands TCP/UDP 30313)")
        print(f"  camera     : {camera_ip}  (GigE Vision, control UDP 3956)")
        # the exact string HALCON forms for open_framegrabber, so a script
        # with a hardcoded device name can be checked without guessing at
        # the convention
        print(f"  device name: {self.gige_server.devices[0].halcon_name}")
        print(f"  lights     : {args.lights} on a ring r={RING_RADIUS} "
              f"h={RING_HEIGHT}")
        print(f"  ground truth written to {args.groundtruth}")
        # make the wiring visible: a missing binding used to be silent, and
        # "the controller runs but no images arrive" is indistinguishable from
        # "nothing is connected" without this
        print("\ncontroller output -> scene bindings:")
        for output in sorted(set(list(controller._light_bindings) +
                                 list(controller._camera_bindings))):
            targets = []
            if output in controller._light_bindings:
                targets.append(f"light {controller._light_bindings[output][0]}")
            if output in controller._camera_bindings:
                targets.append(f"camera trigger ({self.inspection.name}, "
                               f"Software) / Line{self.trigger_line} "
                               f"(hardware)")
            print(f"  OP{output} -> {', '.join(targets)}")

        print("\nPress T in the window to run one sweep, or send MI1,1 / "
              "MI1,0 to the controller.")
        if args.log_gvcp:
            print("GVCP logging is ON: every consumer command will be printed "
                  "below, along with the reason any frame was not captured.")
        print()

    # ------------------------------------------------------------

    def _fire_camera(self):
        """OP8 rising edge: trigger any camera whose consumer left
        TriggerSource on Software. A camera wired to a line is driven by
        bind_camera_line instead and must not be triggered here as well --
        doing so would ignore its LineInverter and TriggerActivation, and
        silently paper over a mis-configured rig."""
        fired = 0
        for device in getattr(self.gige_server, "devices", []):
            if device.trigger_line is None:
                device.trigger_pending = True
                fired += 1
        if fired:
            self.captures += 1

    def _write_groundtruth(self):
        """everything a calibration needs to be scored against.

        The camera basis is taken from matrix_view, which is what actually
        renders -- the camera's stored right vector used to disagree in sign,
        and deriving the basis from the transform would silently mirror the
        solution."""
        camera = self.inspection.camera
        camera.aspect_ratio = RESOLUTION[0] / RESOLUTION[1]
        camera.update()
        view = camera.matrix_view
        width, height = RESOLUTION
        focal_px = (height / 2.0) / math.tan(math.radians(CAMERA_FOV / 2.0))
        truth = {
            "sphere": {"centre": [0.0, 0.0, 0.0], "radius": DOME_RADIUS,
                       "kind": "hemisphere"},
            "camera": {
                "position": [float(v) for v in camera.get_position()],
                "fov_degrees": CAMERA_FOV,
                "resolution": [width, height],
                "intrinsics": {"fx": focal_px, "fy": focal_px,
                               "cx": (width - 1) / 2.0, "cy": (height - 1) / 2.0},
                "right": [float(v) for v in view[0, 0:3]],
                "up": [float(v) for v in view[1, 0:3]],
                "forward": [float(-v) for v in view[2, 0:3]],
            },
            "lights": [{"output": LIGHT_OUTPUTS[i], "index": i,
                        "position": [float(v) for v in self.light_positions[i]]}
                       for i in range(len(self.light_positions))],
            "cycle": {"trigger_input": 1, "camera_output": CAMERA_OUTPUT,
                      "window_seconds": 1.0, "camera_pulse_seconds": 0.5},
        }
        with open(args.groundtruth, "w", encoding="utf-8") as handle:
            json.dump(truth, handle, indent=2)
        self.groundtruth = truth

    # ------------------------------------------------------------

    def update(self):
        # T starts a sweep, exactly as MI1,1 / MI1,0 would. Deliberately not
        # SPACE: camera.move() below binds SPACE to "move up", so the old
        # binding fired a sweep and flew the camera upward at once.
        if self.key_just_pressed(glfw.KEY_T):
            self.gardasoft_server.trigger_input(1)
            print("sweep triggered (IP1)")

        self.camera.move(self.window, self.time_delta)
        self.draw_scene(self.camera)

    def vision_trigger(self, name, value):
        """lets the JSON client start a sweep too: {"cmd":"trigger",
        "name":"sweep"}"""
        if name == "sweep":
            self.gardasoft_server.trigger_input(1)
            return True
        return False


if __name__ == "__main__":
    engine = Example()
    engine.run()

# gige_camera.py
#
# exposes NightEngine's virtual inspection cameras as real GigE Vision
# devices. any standard consumer -- HALCON, Basler pylon Viewer, Pleora
# eBUS Player, Aravis -- can discover and stream from them with no
# plugin, because the device speaks GVCP and GVSP on the wire.
#
#   python gige_camera.py                          # loopback, for a local test
#   python gige_camera.py --ip 169.254.5.60        # one camera on a real NIC
#   python gige_camera.py --ip 169.254.5.60 --ip 169.254.5.61   # two cameras
#
# NETWORK SETUP (this is the part that actually matters)
#
# * The GigE Vision standard fixes the device control port at UDP 3956,
#   so consumers address devices by IP. Every camera therefore needs its
#   OWN local IP address. Add them on Windows with:
#       netsh interface ipv4 add address "Ethernet" 169.254.5.60 255.255.0.0
#   (the adapter's primary address must be static, not DHCP).
#
# * The consumer must be on ANOTHER MACHINE. When client and device share
#   a PC, Windows short-circuits the packets in the IP stack before a GigE
#   filter driver can see them, and HALCON's GigEVision2 interface uses a
#   filter driver by default. The symptom is nasty: discovery and open
#   both succeed and then zero frames arrive.
#
# * The device's advertised IP and subnet mask must be in the SAME SUBNET
#   as the consumer's NIC. HALCON will list a device outside its subnet
#   but mark it inaccessible and refuse to open it.
#
# * Firewall: allow inbound UDP 3956, and inbound UDP to the consumer
#   application on any port (the stream arrives on an ephemeral port).
#   Do it for all three profiles -- an unidentified network is classified
#   Public and silently drops broadcast.
#
# * Prefer MTU 9000 on both NICs. At a 1500-byte packet size a 1.3 MP mono
#   frame is ~900 UDP sends per frame; at 9000 it is ~160.

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.Objects.NightObject import NightObject
from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Meshes.MeshBox import MeshBox
from NightEngine.Meshes.MeshSphere import MeshSphere

BELT_SPEED = 3.0
BELT_END_X = 20.0

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--ip", action="append", default=None,
                    help="local IP for a camera; repeat for more cameras")
parser.add_argument("--mask", default=None,
                    help="subnet mask advertised by the devices")
parser.add_argument("--resolution", default="1280x1024")
parser.add_argument("--format", default="Mono8", choices=["Mono8", "RGB8"])
parser.add_argument("--fps", type=float, default=15.0)
parser.add_argument("--bind-any", action="store_true",
                    help="bind 0.0.0.0:3956 instead of the device address. "
                         "use only if the consumer never discovers the device; "
                         "single camera only")
args = parser.parse_args()

IPS = args.ip if args.ip else ["127.0.0.1"]
MASK = args.mask if args.mask else ("255.0.0.0" if IPS[0].startswith("127.")
                                    else "255.255.0.0")
RES = tuple(int(v) for v in args.resolution.lower().split("x"))


class Example(NightBase):
    def setup(self):

        self.scene = self.create_scene()
        self.camera = NightCamera()
        self.camera.set_position([0, 20, 30])
        self.set_gravity(y=-20)

        self.light_directional["direction"] = [-0.3, -1.0, -0.4]

        # ------------------- inspection cell ------------------- #

        self.floor = NightObject(MeshBox(70, 2, 70, color=[0.45, 0.45, 0.48]),
                                 NightMaterialDefault())
        self.floor.set_position([0, -1, 0])
        self.scene.add(self.floor)

        self.belt = NightObject(MeshBox(46, 0.4, 8, color=[0.15, 0.15, 0.18]),
                                NightMaterialDefault())
        self.belt.set_position([0, 0.2, 0])
        self.scene.add(self.belt)

        self.parts = []
        specs = [(MeshBox(2.5, 2.5, 2.5, color=[0.75, 0.1, 0.1]), 2.0),
                 (MeshSphere(1.4, 24, color=[0.1, 0.55, 0.15]), 1.8),
                 (MeshBox(3.0, 1.2, 2.0, color=[0.15, 0.25, 0.7]), 1.2),
                 (MeshSphere(1.1, 24, color=[0.8, 0.7, 0.1]), 1.5)]
        spacing = (2 * BELT_END_X) / len(specs)
        for i, (mesh, y) in enumerate(specs):
            part = NightObject(mesh, NightMaterialDefault(), mass=1)
            part.set_position([-BELT_END_X + i * spacing, y, 0])
            self.scene.add(part)
            self.parts.append(part)

        # -------------- cameras + GigE devices -------------- #

        poses = [([0, 16, 0], [0, 0, 0], 50, [0.9, 0.9, 0.2]),
                 ([13, 6, 15], [0, 1.5, 0], 55, [0.2, 0.9, 0.9])]

        devices = []
        for index, ip in enumerate(IPS):
            position, target, fov, color = poses[index % len(poses)]
            camera = self.add_vision_camera(name=f"cam{index}",
                                            position=position, target=target,
                                            resolution=RES, fov=fov,
                                            show_body=True, body_color=color,
                                            frustum_depth=16)
            devices.append({
                "camera": camera,
                "ip": ip,
                "subnet_mask": MASK,
                # locally-administered MAC (02:...) derived from the IP so
                # every device is distinct -- consumers key on MAC/serial
                "mac": "02:00:" + ":".join(f"{int(o):02x}" for o in ip.split(".")),
                "vendor": "NightEngine",
                "model": "NightEngineCam",
                "serial": f"NE{index:08d}",
                "user_name": f"cam{index}",
                "pixel_format": args.format,
                "frame_rate": args.fps,
            })

        try:
            self.start_gige_server(devices, bind_any=args.bind_any)
        except OSError as e:
            print(f"\n*** could not start the GigE server ***\n{e}\n")
            raise

        print("\nGigE Vision devices are live. Verify from the consumer machine:")
        print("  HALCON : info_framegrabber('GigEVision2', 'device', I, D)")
        print("  Aravis : arv-tool-0.8 ; arv-camera-test-0.8 -n <name>")
        for info in self.gige_server.info():
            print(f"  {info['name']}: {info['ip']}  "
                  f"{info['width']}x{info['height']}  {info['pixel_format']}  "
                  f"payload {info['payload_size']} B  XML {info['xml_bytes']} B")
        print()

    def physics_update(self, time_step):
        for part in self.parts:
            linear, _ = part.get_velocity()
            part.set_velocity(linear=[BELT_SPEED, linear[1], 0])
            if part.get_position()[0] > BELT_END_X:
                position = part.get_position()
                part.set_position([-BELT_END_X, position[1], position[2]])

    def vision_trigger(self, name, value):
        if name == "ring_light":
            v = float(value)
            self.light_directional["diffuse"] = [v, v, v]
            return True
        return False

    def update(self):
        self.camera.move(self.window, self.time_delta)
        self.draw_scene(self.camera)


if __name__ == "__main__":
    engine = Example()
    engine.run()

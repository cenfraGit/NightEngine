# machinevision.py
#
# an industrial-style inspection station: "parts" slide across an
# inspection area watched by two vision cameras (one top-down, one
# angled). a tcp server exposes the cameras to external programs.
#
# run this, then in another terminal:
#   python vision_client.py            # snap all cameras to png files
#   python vision_client.py --view     # live opencv view (q to quit)
#
# the "lighting controller" is simulated via triggers, e.g.:
#   client.trigger("ring_light", 0.2)  # dim the main light
#   client.trigger("ambient", 0.8)     # crank ambient
#   client.trigger("shadows", 0)       # disable shadows

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.Objects.NightObject import NightObject
from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Materials.NightMaterialTexture import NightMaterialTexture
from NightEngine.Materials.NightMaterialLight import NightMaterialLight
from NightEngine.Meshes.MeshBox import MeshBox
from NightEngine.Meshes.MeshSphere import MeshSphere

FLASH_POSITION = [0, 7, 5]     # strobe lamp over the inspection area
FLASH_COLOR = [2.5, 2.5, 2.2]  # slightly warm, deliberately overdriven

BELT_SPEED = 4.0       # units/s the parts move at
BELT_END_X = 22.0      # parts teleport back once past this


class Example(NightBase):
    def setup(self):

        self.scene = self.create_scene()
        self.camera = NightCamera()
        self.camera.set_position([0, 22, 32])
        self.set_gravity(y=-20)

        self.light_directional["direction"] = [-0.3, -1.0, -0.4]

        # ------------------ floor ------------------ #

        self.floor = NightObject(MeshBox(70, 2, 70, color=[0.45, 0.45, 0.48]),
                                 NightMaterialDefault())
        self.floor.set_position([0, -1, 0])
        self.scene.add(self.floor)

        # ------------- "conveyor" strip ------------- #

        self.belt = NightObject(MeshBox(50, 0.4, 8, color=[0.15, 0.15, 0.18]),
                                NightMaterialDefault())
        self.belt.set_position([0, 0.2, 0])
        self.scene.add(self.belt)

        # ------------------ parts ------------------ #
        # a mix of "good" and "defective" parts to inspect

        self.parts = []
        specs = [
            (MeshBox(2.5, 2.5, 2.5, color=[0.75, 0.1, 0.1]), 2.0),
            (MeshSphere(1.4, 24,   color=[0.1, 0.55, 0.15]), 1.8),
            (MeshBox(3.0, 1.2, 2.0, color=[0.15, 0.25, 0.7]), 1.2),
            (MeshSphere(1.1, 24,   color=[0.8, 0.7, 0.1]), 1.5),
            (MeshBox(1.6, 3.2, 1.6, color=[0.6, 0.25, 0.6]), 2.4),
        ]
        spacing = (2 * BELT_END_X) / len(specs)
        for i, (mesh, y) in enumerate(specs):
            part = NightObject(mesh, NightMaterialDefault(), mass=1)
            part.set_position([-BELT_END_X + i * spacing, y, 0])
            self.scene.add(part)
            self.parts.append(part)

        # --------------- flash strobe --------------- #
        # point light + a small emissive bulb gizmo, fired by the
        # "flash" trigger for a given duration

        self.flash_until = 0.0
        self.light_point["position"] = list(FLASH_POSITION)
        self.light_point["color"] = [0.0, 0.0, 0.0]

        self.flash_bulb = NightObject(MeshSphere(0.5, 12, collision=False),
                                      NightMaterialLight(color=[1.0, 1.0, 0.85]), 0)
        self.flash_bulb.set_position(FLASH_POSITION)
        self.flash_bulb.visible = False
        self.scene.add(self.flash_bulb)

        # -------------- vision cameras -------------- #

        # camera 0: top-down inspection camera over the belt center
        self.add_vision_camera(name="top",
                               position=[0, 18, 0],
                               target=[0, 0, 0],
                               resolution=(640, 480),
                               fov=50)
        # camera 1: angled side camera
        self.add_vision_camera(name="side",
                               position=[14, 6, 16],
                               target=[0, 1.5, 0],
                               resolution=(640, 480),
                               fov=60)

        # --------------- vision server --------------- #

        self.start_vision_server(host="127.0.0.1", port=8555)

    def vision_trigger(self, name, value):
        """simulated lighting controller channels."""
        if name == "ring_light":
            v = float(value)
            self.light_directional["diffuse"] = [v, v, v]
            self.light_directional["specular"] = [v, v, v]
            return True
        if name == "ambient":
            v = float(value)
            self.light_directional["ambient"] = [v, v, v]
            return True
        if name == "shadows":
            self.shadows_enabled = bool(value)
            return True
        if name == "flash":
            # strobe: on for `value` seconds (default 0.5)
            duration = float(value) if value else 0.5
            self.flash_until = self.time + duration
            return True
        return False

    def physics_update(self, time_step):
        # drive the parts along the belt; recycle them at the end
        for part in self.parts:
            linear, _ = part.get_velocity()
            part.set_velocity(linear=[BELT_SPEED, linear[1], 0])
            if part.get_position()[0] > BELT_END_X:
                position = part.get_position()
                part.set_position([-BELT_END_X, position[1], position[2]])

    def update(self):
        # flash strobe timer
        flashing = self.time < self.flash_until
        self.light_point["color"] = FLASH_COLOR if flashing else [0.0, 0.0, 0.0]
        self.flash_bulb.visible = flashing

        self.camera.move(self.window, self.time_delta)
        self.draw_scene(self.camera)


if __name__ == "__main__":
    engine = Example()
    engine.run()

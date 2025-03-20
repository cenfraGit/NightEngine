# test_drone.py

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.Objects.NightObject import NightObject
from NightEngine.Objects.NightLink import NightLink
from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Materials.NightMaterialLight import NightMaterialLight
from NightEngine.Meshes.MeshBox import MeshBox
from NightEngine.Meshes.MeshSphere import MeshSphere
from NightEngine.Objects.ObjectGrid import ObjectGrid
from NightEngine.Objects.ObjectAxes import ObjectAxes
import pybullet as p
import glfw

class Quadcopter(NightObject):
    def __init__(self, scene):
        
        # create drone base
        mesh = MeshBox(3, 1, 5)
        material = NightMaterialDefault()
        super().__init__(mesh, material, 5)

        # create rotors
        self.rot1 = NightLink(MeshSphere(0.6), material)
        self.rot1.set_position([3, 2, 1])
        self.rot1_id = self.add_link(self.rot1, p.JOINT_FIXED)
        scene.add(self.rot1)

    def move(self, window, time_delta: float):

        if self.check_pressed(window, glfw.KEY_I):
            p.applyExternalForce(self.physics_id,
                                 linkIndex=self.rot1_id,
                                 forceObj=[0, 50, 0],
                                 posObj=self.rot1.get_position(),
                                 flags=p.WORLD_FRAME)

class Example(NightBase):
    def setup(self):

        self.scene = self.create_scene()
        self.camera = NightCamera()
        self.camera.set_position([0, 0, 20])
        self.set_gravity()

        self.grid = ObjectGrid(width=100, divisions=20, color=[0.5, 0.5, 0.5])
        self.scene.add(self.grid)

        self.drone = Quadcopter(self.scene)
        self.drone.set_position([0, 3, 0])
        self.scene.add(self.drone)

    def update(self):
        self.drone.move(self.window, self.time_delta)
        self.draw_scene(self.camera)

if __name__ == "__main__":
    engine = Example()
    engine.run()

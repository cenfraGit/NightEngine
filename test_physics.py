# test_physics.py

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.NightObject import NightObject
from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Entities.MeshBox import MeshBox
from NightEngine.Entities.ObjectGrid import ObjectGrid
from NightEngine.Entities.ObjectAxes import ObjectAxes
import pybullet as p
import glfw

class Drone(NightObject):
    def __init__(self, camera):

        self.speed_movement = 4
        self.speed_rotation = 5

        self.current_vertical_force = 0

        mesh = MeshBox(3, 5, 1)
        material = NightMaterialDefault()
        self.camera = camera
        super().__init__(mesh, material, mass=5)

    def move(self, window, time_delta: float):

        # move forward/side respect to camera orientation

        force = 1

        forward = force * self.get_forward_vector()
        side = force * self.get_right_vector()

        p.applyExternalForce(self.physics_id, -1, [0, self.current_vertical_force, 0], self.get_position(True), p.WORLD_FRAME)

        if self.check_pressed(window, glfw.KEY_I):
            p.applyExternalForce(self.physics_id, -1, forward, self.get_position(), p.WORLD_FRAME)
        if self.check_pressed(window, glfw.KEY_K):
            p.applyExternalForce(self.physics_id, -1, -forward, self.get_position(), p.WORLD_FRAME)
        if self.check_pressed(window, glfw.KEY_L):
            p.applyExternalForce(self.physics_id, -1, side, self.get_position(), p.WORLD_FRAME)
        if self.check_pressed(window, glfw.KEY_J):
            p.applyExternalForce(self.physics_id, -1, -side, self.get_position(), p.WORLD_FRAME)
        if self.check_pressed(window, glfw.KEY_Y):
            self.current_vertical_force += self.speed_movement
        if self.check_pressed(window, glfw.KEY_H):
            self.current_vertical_force -= self.speed_movement

class Example(NightBase):
    def setup(self):

        self.scene = self.create_scene()
        self.camera = NightCamera()
        self.camera.set_position([0, 10, 30])
        # self.set_gravity(y=-9.8)

        self.grid = ObjectGrid(width=100, divisions=20, color=[0.5, 0.5, 0.5])
        self.scene.add(self.grid)

        self.axes = ObjectAxes()
        self.scene.add(self.axes)

        self.drone = Drone(self.camera)
        self.drone.set_position([0, 10, 0])
        self.scene.add(self.drone)

    def update(self):
        self.drone.move(self.window, self.time_delta)
        self.draw_scene(self.camera)

if __name__ == "__main__":
    engine = Example()
    engine.run()

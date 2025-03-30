# example_basic.py

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.Objects.NightObject import NightObject
from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Materials.NightMaterialTexture import NightMaterialTexture
from NightEngine.Meshes.MeshBox import MeshBox
from NightEngine.Meshes.MeshSphere import MeshSphere
from NightEngine.Objects.ObjectGrid import ObjectGrid
from NightEngine.Objects.ObjectAxes import ObjectAxes
import pybullet as p
import glfw
from math import cos, sin

class Box(NightObject):
    def __init__(self, material):
        mesh = MeshBox(3, 5, 1, color=[0.2, 0, 0.5])
        super().__init__(mesh, material, 5)

    def move(self, window, time_delta: float):

        amount = 10 * time_delta

        # ----------- lateral movement ----------- #

        if self.check_pressed(window, glfw.KEY_L):
            self.translate(amount, 0, 0, local=False)
        if self.check_pressed(window, glfw.KEY_J):
            self.translate(-amount, 0, 0, local=False)
        if self.check_pressed(window, glfw.KEY_I):
            self.translate(0, 0, -amount, local=False)
        if self.check_pressed(window, glfw.KEY_K):
            self.translate(0, 0, amount, local=False)
        if self.check_pressed(window, glfw.KEY_Y):
            self.translate(0, amount, 0, local=False)
        if self.check_pressed(window, glfw.KEY_H):
            self.translate(0, -amount, 0, local=False)

        # -------------- rotations -------------- #

        # x axis
        if self.check_pressed(window, glfw.KEY_O):
            self.rotate_x(amount)
        if self.check_pressed(window, glfw.KEY_U):
            self.rotate_x(-amount)

        # y axis
        if self.check_pressed(window, glfw.KEY_T):
            self.rotate_y(amount)
        if self.check_pressed(window, glfw.KEY_G):
            self.rotate_y(-amount)

        # z axis
        if self.check_pressed(window, glfw.KEY_R):
            self.rotate_z(amount)
        if self.check_pressed(window, glfw.KEY_F):
            self.rotate_z(-amount)

class Example(NightBase):
    def setup(self):

        self.scene = self.create_scene()
        self.camera = NightCamera()
        self.camera.set_position([0, 10, 15])
        self.set_gravity(y=-20)

        self.sky = NightObject(MeshSphere(200, 32, collision=False), NightMaterialTexture("images/milkyway.jpg", gl_culling=False, lighting=False), 0)
        self.sky.set_position([0, 0, 0])
        self.scene.add(self.sky)

        self.grid = ObjectGrid(width=100, divisions=20, color=[0.5, 0.5, 0.5])
        self.scene.add(self.grid)

        self.axes = ObjectAxes()
        self.scene.add(self.axes)

        self.box = Box(NightMaterialDefault())
        self.box.set_position([0, 10, 0])
        self.scene.add(self.box)

        self.container = NightObject(MeshBox(6, 6, 6), NightMaterialTexture("images/container.jpg"), 5)
        self.container.set_position([10, 10, 0])
        self.scene.add(self.container)

        self.sphere = NightObject(MeshSphere(5, 32, color=[0.3, 0, 0.05]), NightMaterialDefault(), 5)
        self.sphere.set_position([-10, 10, 0])
        self.scene.add(self.sphere)

    def update(self):
        self.box.move(self.window, self.time_delta)
        self.sky.rotate_x(-0.0002)
        self.light_directional["direction"][1] = sin(0.06*self.time)
        self.light_directional["direction"][2] = cos(0.06*self.time)
        self.draw_scene(self.camera)

if __name__ == "__main__":
    engine = Example()
    engine.run()

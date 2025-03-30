# test_basic.py

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.Objects.NightObject import NightObject
from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Materials.NightMaterialLight import NightMaterialLight
from NightEngine.Materials.NightMaterialTexture import NightMaterialTexture
from NightEngine.Meshes.MeshBox import MeshBox
from NightEngine.Meshes.MeshSphere import MeshSphere
from NightEngine.Objects.ObjectGrid import ObjectGrid
from NightEngine.Objects.ObjectAxes import ObjectAxes
import pybullet as p
import glfw

class Box(NightObject):
    def __init__(self, material):
        mesh = MeshBox(3, 5, 1)
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
        self.set_gravity()

        self.grid = ObjectGrid(width=100, divisions=20, color=[0.5, 0.5, 0.5])
        self.scene.add(self.grid)

        self.axes = ObjectAxes()
        self.scene.add(self.axes)

        self.box = Box(NightMaterialDefault())
        self.box.set_position([10, 10, 0])
        self.scene.add(self.box)

        self.box1 = Box(NightMaterialLight())
        self.box1.set_position([0, 10, 0])
        self.scene.add(self.box1)

        self.box2 = NightObject(MeshBox(10, 10, 10), NightMaterialTexture("images/container.jpg"), 5)
        self.box2.set_position([20, 10, 0])
        self.scene.add(self.box2)

        self.sphere = NightObject(MeshSphere(5, 32), NightMaterialDefault(), 5)
        self.sphere.set_position([-10, 10, 0])
        self.scene.add(self.sphere)

    def update(self):
        self.box.move(self.window, self.time_delta)
        self.draw_scene(self.camera)

if __name__ == "__main__":
    engine = Example()
    engine.run()

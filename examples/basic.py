# basic.py

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

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

    def move(self, window, time_step: float):
        """velocity-based control: the box is a dynamic body, so it is
        driven through the solver instead of being teleported. called
        once per physics step from physics_update."""

        speed = 10
        speed_rotation = 3

        linear, angular = self.get_velocity()
        vx, vy, vz = linear
        wx, wy, wz = angular
        controlled_linear = False
        controlled_angular = False

        # ----------- lateral movement ----------- #

        if self.check_pressed(window, glfw.KEY_L):
            vx = speed; controlled_linear = True
        if self.check_pressed(window, glfw.KEY_J):
            vx = -speed; controlled_linear = True
        if self.check_pressed(window, glfw.KEY_I):
            vz = -speed; controlled_linear = True
        if self.check_pressed(window, glfw.KEY_K):
            vz = speed; controlled_linear = True
        if self.check_pressed(window, glfw.KEY_Y):
            vy = speed; controlled_linear = True
        if self.check_pressed(window, glfw.KEY_H):
            vy = -speed; controlled_linear = True

        # -------------- rotations -------------- #

        # x axis
        if self.check_pressed(window, glfw.KEY_O):
            wx = speed_rotation; controlled_angular = True
        if self.check_pressed(window, glfw.KEY_U):
            wx = -speed_rotation; controlled_angular = True

        # y axis
        if self.check_pressed(window, glfw.KEY_T):
            wy = speed_rotation; controlled_angular = True
        if self.check_pressed(window, glfw.KEY_G):
            wy = -speed_rotation; controlled_angular = True

        # z axis
        if self.check_pressed(window, glfw.KEY_R):
            wz = speed_rotation; controlled_angular = True
        if self.check_pressed(window, glfw.KEY_F):
            wz = -speed_rotation; controlled_angular = True

        if controlled_linear:
            self.set_velocity(linear=[vx, vy, vz])
        if controlled_angular:
            self.set_velocity(angular=[wx, wy, wz])

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

    def physics_update(self, time_step):
        self.box.move(self.window, time_step)

    def update(self):
        self.sky.rotate_x(-0.0002)
        self.light_directional["direction"][1] = sin(0.06*self.time)
        self.light_directional["direction"][2] = cos(0.06*self.time)
        self.camera.move(self.window, self.time_delta)
        self.draw_scene(self.camera)

if __name__ == "__main__":
    engine = Example()
    engine.run()

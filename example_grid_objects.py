# example_grid_objects.py

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.NightObject import NightObject
from NightEngine.NightMaterial import NightMaterial
from NightEngine.Entities.MeshBox import MeshBox
from NightEngine.Entities.MeshSphere import MeshSphere
from NightEngine.Entities.ObjectGrid import ObjectGrid
from NightEngine.Entities.ObjectAxes import ObjectAxes
import pybullet as p
import glfw

class MyObject(NightObject):
    def __init__(self, camera):

        mesh = MeshSphere(4, 32, 32)
        material = NightMaterial(gl_wireframe=True)
        self.camera = camera
        super().__init__(mesh, material, mass=5)

    def move(self, window, time_delta: float):

        # move forward/side respect to camera orientation

        force = 400

        forward = self.camera.get_forward_vector()
        forward = [forward[0], forward[2], 0]
        forward = [force * v for v in forward]
        side = self.camera.get_right_vector()
        side = [side[0], side[2], 0]
        side = [force * v for v in side]

        if self.check_pressed(window, glfw.KEY_I):
            p.applyExternalForce(self.physics_id, -1, forward, self.get_position(True), p.WORLD_FRAME)
        if self.check_pressed(window, glfw.KEY_K):
            p.applyExternalForce(self.physics_id, -1, [-v for v in forward], self.get_position(True), p.WORLD_FRAME)
        if self.check_pressed(window, glfw.KEY_J):
            p.applyExternalForce(self.physics_id, -1, side, self.get_position(True), p.WORLD_FRAME)
        if self.check_pressed(window, glfw.KEY_L):
            p.applyExternalForce(self.physics_id, -1, [-v for v in side], self.get_position(True), p.WORLD_FRAME)
        if self.check_pressed(window, glfw.KEY_Y):
            p.applyExternalForce(self.physics_id, -1, [0, 0, 500], self.get_position(True), p.WORLD_FRAME)
        if self.check_pressed(window, glfw.KEY_H):
            p.applyExternalForce(self.physics_id, -1, [0, 0, -500], self.get_position(True), p.WORLD_FRAME)

class Example(NightBase):
    def setup(self):

        self.scene = self.create_scene()
        self.camera = NightCamera()
        self.camera.set_position([0, 10, -30])
        self.set_gravity(z=-40)

        self.grid = ObjectGrid(width=100, divisions=20, color=[0.5, 0.5, 0.5])
        self.scene.add(self.grid)

        self.axes = ObjectAxes(length=10, line_width=6)
        self.scene.add(self.axes)

        self.sphere = MyObject(self.camera)
        self.sphere.set_position([0, 10, -20])
        self.scene.add(self.sphere)

        w = 5
        hor = 5
        vert = 5
        for i in range(-hor, hor+1):
            for j in range(2, vert):
                cube = NightObject(MeshBox(w), NightMaterial(), mass=1)
                cube.set_position([i*w, j*w, 0])
                self.scene.add(cube)

    def update(self):
        self.sphere.move(self.window, self.time_delta)
        p.changeDynamics(self.sphere.physics_id, -1, restitution=0.9)
        p.changeDynamics(self.grid.physics_id, -1, restitution=0.8)
        self.draw_scene(self.camera)

if __name__ == "__main__":
    engine = Example()
    engine.run()

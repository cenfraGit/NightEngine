# gridobjects.py

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.Objects.NightObject import NightObject
from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Meshes.MeshBox import MeshBox
from NightEngine.Meshes.MeshSphere import MeshSphere
from NightEngine.Objects.ObjectGrid import ObjectGrid
from NightEngine.Objects.ObjectAxes import ObjectAxes
import pybullet as p
import glfw

class MyObject(NightObject):
    def __init__(self, camera):
        mesh = MeshSphere(4, 32)
        material = NightMaterialDefault(gl_wireframe=False)
        self.camera = camera
        super().__init__(mesh, material, mass=5)

    def move(self, window, time_delta: float):

        # move forward/side respect to camera orientation

        force = 2000

        forward = force * self.camera.get_forward_vector()
        side = force * self.camera.get_right_vector()

        if self.check_pressed(window, glfw.KEY_I):
            p.applyExternalForce(self.physics_id, -1, forward, self.get_position(), p.WORLD_FRAME)
        if self.check_pressed(window, glfw.KEY_K):
            p.applyExternalForce(self.physics_id, -1, -forward, self.get_position(), p.WORLD_FRAME)
        if self.check_pressed(window, glfw.KEY_J):
            p.applyExternalForce(self.physics_id, -1, side, self.get_position(), p.WORLD_FRAME)
        if self.check_pressed(window, glfw.KEY_L):
            p.applyExternalForce(self.physics_id, -1, -side, self.get_position(), p.WORLD_FRAME)
        if self.check_pressed(window, glfw.KEY_Y):
            p.applyExternalForce(self.physics_id, -1, [0, 4000, 0], self.get_position(), p.WORLD_FRAME)

class Example(NightBase):
    def setup(self):

        self.scene = self.create_scene()
        self.camera = NightCamera()
        self.camera.set_position([0, 10, 30])
        self.set_gravity(y=-50)

        self.light_directional = {
            "direction": [-0.5, -0.5, 0],
            "ambient": [0.3, 0.3, 0.3],
            "diffuse": [1.0, 1.0, 1.0],
            "specular": [1.0, 1.0, 1.0]
        }

        self.grid = ObjectGrid(width=100, divisions=20, color=[0.5, 0.5, 0.5])
        self.scene.add(self.grid)

        self.axes = ObjectAxes(length=10, line_width=6)
        self.scene.add(self.axes)

        self.sphere = MyObject(self.camera)
        self.sphere.set_position([0, 10, 20])
        self.scene.add(self.sphere)

        w = 5
        hor = 5
        for i in range(-hor, hor+1):
            for j in range(2, 6):
                cube = NightObject(MeshBox(w, w, w, color=[0.7, 0, 0]), NightMaterialDefault(), mass=1)
                cube.set_position([i*w+ i, j*w + 2*j, 0])
                self.scene.add(cube)

    def update(self):
        self.sphere.move(self.window, self.time_delta)
        p.changeDynamics(self.sphere.physics_id, -1, restitution=0.9)
        p.changeDynamics(self.grid.physics_id, -1, restitution=0.8)
        self.camera.move(self.window, self.time_delta)
        self.draw_scene(self.camera)

if __name__ == "__main__":
    engine = Example()
    engine.run()

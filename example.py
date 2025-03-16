# example.py

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.NightObject import NightObject
from NightEngine.NightMaterial import NightMaterial
from NightEngine.Entities.MeshBox import MeshBox
from NightEngine.Entities.MeshSphere import MeshSphere
from NightEngine.Entities.ObjectGrid import ObjectGrid
from NightEngine.Entities.ObjectAxes import ObjectAxes

class Example(NightBase):
    def setup(self):

        self.scene = NightObject()
        self.camera = NightCamera()
        self.camera.set_position([0, 0, -10])

        self.grid = ObjectGrid(width=100, divisions=100, color=[0.5, 0.5, 0.5])
        self.scene.add(self.grid)

        self.axes = ObjectAxes(length=10, line_width=6)
        self.scene.add(self.axes)
        
        # self.cube = NightObject(MeshBox(5), NightMaterial())
        # self.scene.add(self.cube)

        self.sphere = NightObject(MeshSphere(10, 32, 32), NightMaterial())
        self.scene.add(self.sphere)

    def update(self):
        self.draw_scene(self.scene, self.camera)

if __name__ == "__main__":
    engine = Example()
    engine.run()

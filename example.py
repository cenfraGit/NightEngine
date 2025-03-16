# example.py

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.NightObject import NightObject
from NightEngine.NightMaterial import NightMaterial
from NightEngine.entities.MeshBox import MeshBox

class Example(NightBase):
    def setup(self):

        self.scene = NightObject()
        self.camera = NightCamera()
        self.camera.set_position([0, 0, -10])
    
        material = NightMaterial(gl_culling=True,
                                 gl_wireframe=False,
                                 gl_line_width=5,
                                 gl_point_size=5)

        self.cube = NightObject(MeshBox(5), material)
        self.scene.add(self.cube)

    def update(self):
        self.draw_scene(self.scene, self.camera)

if __name__ == "__main__":
    engine = Example()
    engine.run()

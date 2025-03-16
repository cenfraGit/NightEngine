# example.py

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.NightObject import NightObject
from NightEngine.NightMaterial import NightMaterial
from NightEngine.NightUtils import NightUtils

class Example(NightBase):
    def setup(self):

        self.scene = NightObject()
        self.camera = NightCamera()
        
        positions = [
            -0.5, -0.5, -0.5,
             0.5, -0.5, -0.5,
             0.5,  0.5, -0.5,
            -0.5,  0.5, -0.5,

            -0.5, -0.5,  0.5,
             0.5, -0.5,  0.5,
             0.5,  0.5,  0.5,
            -0.5,  0.5,  0.5,

            -0.5, -0.5, -0.5,
            -0.5,  0.5, -0.5,
            -0.5,  0.5,  0.5,
            -0.5, -0.5,  0.5,

             0.5, -0.5, -0.5,
             0.5,  0.5, -0.5,
             0.5,  0.5,  0.5,
             0.5, -0.5,  0.5,

            -0.5, -0.5, -0.5,
             0.5, -0.5, -0.5,
             0.5, -0.5,  0.5,
            -0.5, -0.5,  0.5,

            -0.5,  0.5, -0.5,
             0.5,  0.5, -0.5,
             0.5,  0.5,  0.5,
            -0.5,  0.5,  0.5 
        ]

        colors = [
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,

            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,

            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,

            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,

            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,

            0.0, 1.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 1.0, 0.0
        ]


        material = NightMaterial()
        material.gl_culling = False
        material.gl_wireframe = True
        material.gl_line_width = 5
        material.gl_point_size = 5
        
        self.cube = NightObject(material)
        
        vbo_position = NightUtils.create_vbo(positions)
        vbo_color    = NightUtils.create_vbo(colors)
        NightUtils.set_attribute_pointer(material.program, vbo_position, "vertex_position", "vec3")
        NightUtils.set_attribute_pointer(material.program, vbo_color,    "vertex_color", "vec3")
        # self.cube.vertex_count = len(positions)

        self.cube.scale(10)
        self.cube.translate(0, 0, 4)
        self.scene.add(self.cube)

        self.camera.set_position([0, 0, -10])


    def update(self):
        self.draw_scene(self.scene, self.camera)

if __name__ == "__main__":
    engine = Example()
    engine.run()

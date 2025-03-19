# ObjectGrid.py

from NightEngine.NightObject import NightObject
from NightEngine.NightMesh import NightMesh
from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from OpenGL.GL import *

class ObjectAxes(NightObject):
    def __init__(self,
                 length=10,
                 line_width=1):

        positions = [[0, 0, 0], [length, 0, 0],
                     [0, 0, 0], [0, length, 0],
                     [0, 0, 0], [0, 0, length]]

        colors = [[1, 0, 0], [1, 0, 0],
                  [0, 1, 0], [0, 1, 0],
                  [0, 0, 1], [0, 0, 1]]

        mesh = NightMesh()
        mesh.add_attribute("vertex_position", "vec3", positions)
        mesh.add_attribute("vertex_color", "vec3", colors)
        mesh.vertex_count = len(positions)

        material = NightMaterialDefault(gl_draw_style=GL_LINES,
                                        gl_line_width=line_width,
                                        gl_culling=False,
                                        lighting=False)

        super().__init__(mesh, material, mass=0)

# ObjectGrid.py

from NightEngine.NightObject import NightObject
from NightEngine.NightMesh import NightMesh
from NightEngine.NightMaterial import NightMaterial
import pybullet as p
from OpenGL.GL import *

class ObjectGrid(NightObject):
    def __init__(self,
                 width=10,
                 divisions=10,
                 color=[1, 1, 1],
                 line_width=1):

        positions = []
        colors = []

        values = []
        for n in range(divisions + 1):
            values.append(-width/2 + n*(width / divisions))

        # ------------ vertical lines ------------ #

        for x in values:
            positions.append([x, 0, -width/2])
            positions.append([x, 0,  width/2])
            colors.append(color)
            colors.append(color)

        # ----------- horizontal lines ----------- #

        for y in values:
            positions.append([-width/2, 0, y])
            positions.append([ width/2, 0, y])
            colors.append(color)
            colors.append(color)
        
        mesh = NightMesh()
        mesh.add_attribute("vertex_position", "vec3", positions)
        mesh.add_attribute("vertex_color", "vec3", colors)
        mesh.vertex_count = len(positions)
        mesh.set_collision_shape(p.createCollisionShape(p.GEOM_BOX,
                                                        halfExtents=[width/2, width/2, 0]))
        
        material = NightMaterial(gl_draw_style=GL_LINES,
                                 gl_line_width=line_width,
                                 gl_culling=False)

        super().__init__(mesh, material)

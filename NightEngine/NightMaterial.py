# NightMaterial.py

from OpenGL.GL import *
from NightEngine.NightUtils import NightUtils

class NightMaterial:
    def __init__(self):

        # ------------------------------------------------------------
        # default shaders
        # ------------------------------------------------------------
        
        code_shader_vertex = """
        #version 330 core
        uniform mat4 matrix_projection;
        uniform mat4 matrix_view;
        uniform mat4 matrix_model;
        in vec3 vertex_position;
        in vec3 vertex_color;
        out vec3 color;
        void main() {
          gl_Position = matrix_projection * matrix_view * matrix_model * vec4(vertex_position, 1.0);
          color = vertex_color;
        }
        """

        code_shader_fragment = """
        #version 330 core
        in vec3 color;
        out vec4 frag_color;
        void main() {
          frag_color = vec4(color, 1.0);
        }
        """

        # ------------ create program ------------ #

        self.program = NightUtils.create_program(code_shader_vertex,
                                                 code_shader_fragment)

        # ------------------------------------------------------------
        # default draw values
        # ------------------------------------------------------------

        self.gl_draw_style = GL_TRIANGLES
        self.gl_line_width = 2
        self.gl_point_size = 2
        self.gl_culling = True
        self.gl_wireframe = False

    def update_draw_settings(self):

        glPointSize(self.gl_point_size)
        glLineWidth(self.gl_line_width)

        if self.gl_culling:
            glEnable(GL_CULL_FACE)
        else:
            glDisable(GL_CULL_FACE)

        if self.gl_wireframe:
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        else:
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        

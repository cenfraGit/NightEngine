# NightMaterial.py

from OpenGL.GL import *
from NightEngine.NightUtils import NightUtils

class NightMaterial:
    def __init__(self):

        code_shader_vertex = """
        #version 330 core
        uniform mat4 matrix_projection;
        uniform mat4 matrix_view;
        uniform mat4 matrix_model;
        in vec3 vertex_position;
        in vec3 vertex_color;
        out vec3 color;
        void main() {
          gl_Position = projection_matrix *
                        view_matrix *
                        model_matrix *
                        vec4(vertex_position, 1.0);
          color = vertex_color;
        }
        """

        code_shader_fragment = """
        #version 330 core
        uniform vec3 base_color;
        uniform bool use_vertex_colors;
        in vec3 color;
        out vec4 frag_color;
        void main() {
          vec4 temp_color = vec4(base_color, 1.0);
          if (use_vertex_colors)
            temp_color *= vec4(color, 1.0);
          frag_color = temp_color;
        }
        """

        self.program = NightUtils.create_program(code_shader_vertex,
                                                 code_shader_fragment)

        self.gl_draw_style = GL_LINES
        self.gl_line_width = 2
        self.gl_point_size = 3
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
        

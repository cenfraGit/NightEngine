# NightMaterialLight

from OpenGL.GL import *
from NightEngine.NightUtils import NightUtils

class NightMaterialLight:
    def __init__(self,
                 gl_draw_style=GL_TRIANGLES,
                 gl_line_width=2,
                 gl_point_size=2,
                 gl_culling=True,
                 gl_wireframe=False,
                 color=[1.0, 1.0, 1.0]):

        # ------------------------------------------------------------
        # shaders
        # ------------------------------------------------------------
        
        code_shader_vertex = """
        #version 330 core
        uniform mat4 matrix_projection;
        uniform mat4 matrix_view;
        uniform mat4 matrix_model;
        in vec3 vertex_position;
        void main() {
          gl_Position = matrix_projection * matrix_view * matrix_model * vec4(vertex_position, 1.0);
        }
        """
        
        code_shader_fragment = """
        #version 330 core
        uniform vec3 light_color;
        out vec4 frag_color;
        void main() {
          frag_color = vec4(light_color, 1.0);
        }
        """

        # ------------ create program ------------ #

        self.program = NightUtils.create_program(code_shader_vertex,
                                                 code_shader_fragment)

        # -------- set light source color -------- #

        NightUtils.set_uniform(self.program,
                               "light_color",
                               "vec3",
                               color)

        # ------------------------------------------------------------
        # default draw values
        # ------------------------------------------------------------

        self.gl_draw_style = gl_draw_style
        self.gl_line_width = gl_line_width
        self.gl_point_size = gl_point_size
        self.gl_culling = gl_culling
        self.gl_wireframe = gl_wireframe

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
        

# NightMaterialDefault.py

from OpenGL.GL import *
from NightEngine.NightUtils import NightUtils

class NightMaterialDefault:
    def __init__(self,
                 gl_draw_style=GL_TRIANGLES,
                 gl_line_width=2,
                 gl_point_size=2,
                 gl_culling=True,
                 gl_wireframe=False,
                 lighting=True):

        # ------------------------------------------------------------
        # shaders
        # ------------------------------------------------------------

        code_shader_vertex = """
        #version 330 core
        uniform mat4 matrix_projection;
        uniform mat4 matrix_view;
        uniform mat4 matrix_model;
        in vec3 vertex_position;
        in vec3 vertex_color;
        in vec3 vertex_normal;
        out vec3 normal;
        out vec3 color;
        out vec3 frag_pos;
        void main() {
          normal = mat3(transpose(inverse(matrix_model))) * vertex_normal;
          color = vertex_color;
          frag_pos = vec3(matrix_model * vec4(vertex_position, 1.0));
          gl_Position = matrix_projection * matrix_view * matrix_model * vec4(vertex_position, 1.0);
        }
        """

        code_shader_fragment = """
        #version 330 core
        in vec3 color;
        in vec3 normal;
        in vec3 frag_pos;
        out vec4 frag_color;
        uniform bool lighting;
        uniform vec3 view_pos;
        void main() {

          vec3 light_color = vec3(1.0, 1.0, 1.0);

          if (lighting) {

            // --------------- ambient ---------------
        
            vec3 ambient = 0.1 * light_color;

            // --------------- diffuse ---------------
        
            vec3 norm = normalize(normal);
            vec3 light_dir = normalize(vec3(2, 5, 4) - frag_pos);
            float diff = max(dot(norm, light_dir), 0.0);
            vec3 diffuse = diff * light_color;

            // --------------- specular ---------------
        
            vec3 view_dir = normalize(view_pos - frag_pos);
            vec3 reflect_dir = reflect(-light_dir, norm);
            float spec = pow(max(dot(view_dir, reflect_dir), 0.0), 32);
            vec3 specular = 0.5 * spec * light_color;

            vec3 result = (ambient + diffuse + specular) * color;
            frag_color = vec4(result, 1.0);
        
          } else {
        
            frag_color = vec4(color, 1.0);
        
          }
        }
        """
        
        # ------------ create program ------------ #

        self.program = NightUtils.create_program(code_shader_vertex,
                                                 code_shader_fragment)

        # ---------- configure lighting ---------- #

        NightUtils.set_uniform(self.program,
                               "lighting",
                               "bool",
                               lighting)

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
        

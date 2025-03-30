# NightMaterialTexture.py

from OpenGL.GL import *
from NightEngine.NightUtils import NightUtils
import numpy as np
from PIL import Image

class NightMaterialTexture:
    def __init__(self,
                 filename=None,
                 gl_draw_style=GL_TRIANGLES,
                 gl_line_width=2,
                 gl_point_size=2,
                 gl_culling=True,
                 gl_wireframe=False,
                 gl_wrap_s=GL_REPEAT,
                 gl_wrap_t=GL_REPEAT,
                 gl_min_filter=GL_LINEAR,
                 gl_mag_filter=GL_LINEAR,
                 lighting=True):

        # ------------------------------------------------------------
        # initialize texture
        # ------------------------------------------------------------

        self.surface = Image.open(filename).convert("RGBA") if filename else None

        self.gl_texture = glGenTextures(1)

        if self.surface:
            width, height = self.surface.size
            pixel_data = np.array(self.surface)
            glBindTexture(GL_TEXTURE_2D, self.gl_texture)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height,
                         0, GL_RGBA, GL_UNSIGNED_BYTE, pixel_data.tobytes())
            glGenerateMipmap(GL_TEXTURE_2D)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, gl_mag_filter)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, gl_min_filter)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, gl_wrap_s)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, gl_wrap_t)
            glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, [1, 1, 1, 1])

        # ------------------------------------------------------------
        # material attributes
        # ------------------------------------------------------------

        self.gl_draw_style = gl_draw_style
        self.gl_line_width = gl_line_width
        self.gl_point_size = gl_point_size
        self.gl_culling = gl_culling
        self.gl_wireframe = gl_wireframe
        self.lighting = lighting

        self.shininess = 32.0
        self.ambient = [1.0, 1.0, 1.0]
        self.diffuse = [1.0, 1.0, 1.0]
        self.specular = [1.0, 1.0, 1.0]

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
        in vec2 vertex_uv;

        uniform vec2 uv_repeat;
        uniform vec2 uv_offset;
        
        out vec3 normal;
        out vec3 color;
        out vec3 frag_pos;
        out vec2 uv;
        
        void main() {
          normal = mat3(transpose(inverse(matrix_model))) * vertex_normal;
          color = vertex_color;
          frag_pos = vec3(matrix_model * vec4(vertex_position, 1.0));
          uv = vertex_uv * uv_repeat + uv_offset;
          gl_Position = matrix_projection * matrix_view * matrix_model * vec4(vertex_position, 1.0);
        }
        """

        code_shader_fragment = """
        #version 330 core

        struct LightDirectional {
          vec3 direction;
          vec3 ambient;
          vec3 diffuse;
          vec3 specular;
        };

        struct Material {
          float shininess;
          vec3 ambient;
          vec3 diffuse;
          vec3 specular;
        };

        uniform bool bool_lighting;
        uniform vec3 view_pos;
        uniform sampler2D texture;
        
        uniform LightDirectional light_directional;
        uniform Material material;
        
        in vec3 color;
        in vec3 normal;
        in vec3 frag_pos;
        in vec2 uv;
        out vec4 frag_color;

        // prototypes
        vec3 CalcLightDir(LightDirectional light, vec3 normal, vec3 view_dir);
        
        void main() {
        
          vec3 result;
        
          if (bool_lighting) {
            vec3 norm = normalize(normal);
            vec3 view_dir = normalize(view_pos - frag_pos);
            result = CalcLightDir(light_directional, norm, view_dir);
          } else {
            result = color;
          }
          frag_color = vec4(result, 1.0) * texture2D(texture, uv);
        }

        vec3 CalcLightDir(LightDirectional light, vec3 normal, vec3 view_dir) {
          vec3 light_dir = normalize(-light.direction);
          float diff = max(dot(normal, light_dir), 0.0);
          vec3 reflect_dir = reflect(-light_dir, normal);
          float spec = pow(max(dot(view_dir, reflect_dir), 0.0), material.shininess);

          vec3 ambient = light.ambient * material.ambient;
          vec3 diffuse = light.diffuse * diff * material.diffuse;
          vec3 specular = light.specular * spec * material.specular;

          return (ambient + diffuse + specular) * color;
        }
        """
        
        # ------------ create program ------------ #

        self.program = NightUtils.create_program(code_shader_vertex,
                                                 code_shader_fragment)

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

        NightUtils.set_uniform(self.program, "bool_lighting", "bool", self.lighting)
        

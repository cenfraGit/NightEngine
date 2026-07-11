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
        uniform mat4 matrix_light;

        in vec3 vertex_position;
        in vec3 vertex_color;
        in vec3 vertex_normal;

        out vec3 normal;
        out vec3 color;
        out vec3 frag_pos;
        out vec4 frag_pos_light;

        void main() {
          normal = mat3(transpose(inverse(matrix_model))) * vertex_normal;
          color = vertex_color;
          frag_pos = vec3(matrix_model * vec4(vertex_position, 1.0));
          frag_pos_light = matrix_light * vec4(frag_pos, 1.0);
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

        struct LightPoint {
          vec3 position;
          vec3 color;      // rgb premultiplied by intensity; black = off
          float attenuation_linear;
          float attenuation_quadratic;
        };

        uniform bool bool_lighting;
        uniform bool bool_shadows;
        uniform vec3 view_pos;
        uniform sampler2D shadow_map;
        uniform float shadow_bias;

        uniform LightDirectional light_directional;
        uniform LightPoint light_point;
        uniform Material material;

        in vec3 color;
        in vec3 normal;
        in vec3 frag_pos;
        in vec4 frag_pos_light;
        out vec4 frag_color;

        // prototypes
        vec3 CalcLightDir(LightDirectional light, vec3 normal, vec3 view_dir);
        vec3 CalcLightPoint(vec3 normal, vec3 view_dir);
        float CalcShadow(vec3 normal, vec3 light_dir);

        void main() {

          vec3 result;

          if (bool_lighting) {
            vec3 norm = normalize(normal);
            vec3 view_dir = normalize(view_pos - frag_pos);
            result = CalcLightDir(light_directional, norm, view_dir);
            result += CalcLightPoint(norm, view_dir);
          } else {
            result = color;
          }
          frag_color = vec4(result, 1.0);
        }

        vec3 CalcLightPoint(vec3 normal, vec3 view_dir) {
          vec3 to_light = light_point.position - frag_pos;
          float dist = length(to_light);
          vec3 light_dir = to_light / max(dist, 0.0001);
          float diff = max(dot(normal, light_dir), 0.0);
          vec3 halfway_dir = normalize(light_dir + view_dir);
          float spec = pow(max(dot(normal, halfway_dir), 0.0), material.shininess);
          float attenuation = 1.0 / (1.0 + light_point.attenuation_linear * dist
                                         + light_point.attenuation_quadratic * dist * dist);
          return attenuation * light_point.color
                 * (diff * material.diffuse + spec * material.specular) * color;
        }

        vec3 CalcLightDir(LightDirectional light, vec3 normal, vec3 view_dir) {
          vec3 light_dir = normalize(-light.direction);
          float diff = max(dot(normal, light_dir), 0.0);
          // blinn-phong specular (softer highlights than plain phong)
          vec3 halfway_dir = normalize(light_dir + view_dir);
          float spec = pow(max(dot(normal, halfway_dir), 0.0), material.shininess);

          vec3 ambient = light.ambient * material.ambient;
          vec3 diffuse = light.diffuse * diff * material.diffuse;
          vec3 specular = light.specular * spec * material.specular;

          float shadow = CalcShadow(normal, light_dir);
          return (ambient + (1.0 - shadow) * (diffuse + specular)) * color;
        }

        float CalcShadow(vec3 normal, vec3 light_dir) {
          if (!bool_shadows) return 0.0;
          vec3 proj = frag_pos_light.xyz / frag_pos_light.w;
          proj = proj * 0.5 + 0.5;
          if (proj.z > 1.0) return 0.0;
          // slope-scaled bias against shadow acne
          float bias = max(shadow_bias * 4.0 * (1.0 - dot(normal, light_dir)), shadow_bias);
          // 3x3 pcf for soft edges
          float shadow = 0.0;
          vec2 texel = 1.0 / vec2(textureSize(shadow_map, 0));
          for (int x = -1; x <= 1; ++x) {
            for (int y = -1; y <= 1; ++y) {
              float closest = texture(shadow_map, proj.xy + vec2(x, y) * texel).r;
              shadow += (proj.z - bias > closest) ? 1.0 : 0.0;
            }
          }
          return shadow / 9.0;
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
        

# NightShadow.py
# shadow mapping for the directional light: a depth-only render pass
# from the light's point of view into a depth texture, which the
# materials then sample to darken occluded fragments.

from NightEngine.NightMatrix import NightMatrix
from NightEngine.NightUtils import NightUtils
from OpenGL.GL import *
import numpy as np


class NightShadow:
    def __init__(self, resolution=2048):

        self.resolution = resolution

        # ------------------------------------------------------------
        # depth texture + framebuffer
        # ------------------------------------------------------------

        self.texture = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self.texture)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24,
                     resolution, resolution, 0,
                     GL_DEPTH_COMPONENT, GL_FLOAT, None)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        # everything outside the shadow frustum reads depth 1.0 (lit)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER)
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, [1.0, 1.0, 1.0, 1.0])

        self.fbo = glGenFramebuffers(1)
        glBindFramebuffer(GL_FRAMEBUFFER, self.fbo)
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                               GL_TEXTURE_2D, self.texture, 0)
        glDrawBuffer(GL_NONE)
        glReadBuffer(GL_NONE)
        if glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE:
            raise Exception("NightShadow: shadow framebuffer incomplete.")
        glBindFramebuffer(GL_FRAMEBUFFER, 0)

        # ------------------------------------------------------------
        # depth-only shader
        # ------------------------------------------------------------

        code_shader_vertex = """
        #version 330 core
        uniform mat4 matrix_light;
        uniform mat4 matrix_model;
        in vec3 vertex_position;
        void main() {
          gl_Position = matrix_light * matrix_model * vec4(vertex_position, 1.0);
        }
        """

        code_shader_fragment = """
        #version 330 core
        void main() {}
        """

        self.program = NightUtils.create_program(code_shader_vertex,
                                                 code_shader_fragment)

    def get_light_matrix(self, direction, target=[0, 0, 0],
                         size=60.0, near=1.0, far=300.0, distance=100.0):
        """builds the light-space (projection @ view) matrix for a
        directional light looking at `target`. `size` is the half-width
        of the orthographic shadow box in world units."""
        d = np.array(direction, dtype=np.float64)
        norm = np.linalg.norm(d)
        if norm == 0:
            d = np.array([0.0, -1.0, 0.0])
            norm = 1.0
        d = d / norm
        # pick an up vector that is not parallel to the light direction
        up = [0, 1, 0] if abs(d[1]) < 0.99 else [0, 0, 1]
        target = np.array(target, dtype=np.float64)
        eye = target - d * distance
        view = NightMatrix.get_lookat(eye, target, up)
        projection = NightMatrix.get_orthographic(-size, size, -size, size, near, far)
        return projection @ view

    def begin(self, matrix_light):
        """starts the depth pass: render calls after this write depth
        from the light's point of view."""
        glBindFramebuffer(GL_FRAMEBUFFER, self.fbo)
        glViewport(0, 0, self.resolution, self.resolution)
        glClear(GL_DEPTH_BUFFER_BIT)
        glUseProgram(self.program)
        NightUtils.set_uniform(self.program, "matrix_light", "mat4", matrix_light)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        # front-face culling reduces shadow acne on closed meshes
        glEnable(GL_CULL_FACE)
        glCullFace(GL_FRONT)

    def draw(self, obj):
        """renders one object into the shadow map."""
        NightUtils.set_uniform(self.program, "matrix_model", "mat4", obj.get_world_matrix())
        glBindVertexArray(obj.vao)
        glDrawArrays(GL_TRIANGLES, 0, obj.mesh.vertex_count)

    def end(self):
        """ends the depth pass (caller rebinds its target framebuffer)."""
        glCullFace(GL_BACK)

# NightBase.py

from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Materials.NightMaterialLight import NightMaterialLight
from NightEngine.Objects.NightObject import NightObject
from NightEngine.NightUtils import NightUtils
from NightEngine.NightCamera import NightCamera
from scipy.spatial.transform import Rotation as R
from OpenGL.GL import *
import numpy as np
import pybullet as p
import glfw

class NightBase:
    def __init__(self,
                 width=900,
                 height=900,
                 title="NightEngine"):

        # ------------------------------------------------------------
        # initialize and configure glfw
        # ------------------------------------------------------------

        if not glfw.init():
            raise Exception("Problem initializing glfw.")

        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)

        self.window = glfw.create_window(width, height, title, None, None)
        if not self.window:
            glfw.terminate()
            raise Exception("Problem creating glfw window.")

        glfw.make_context_current(self.window)

        # ------------------------------------------------------------
        # callbacks and glfw config
        # ------------------------------------------------------------
        
        glfw.set_framebuffer_size_callback(self.window, self._callback_framebuffer_size)
        glfw.set_cursor_pos_callback      (self.window, self._callback_cursor_pos)
        glfw.set_scroll_callback          (self.window, self._callback_scroll)
        # glfw.set_input_mode               (self.window, glfw.CURSOR, glfw.CURSOR_HIDDEN)

        # ------------------------------------------------------------
        # variables
        # ------------------------------------------------------------

        # ----------------- time ----------------- #
        
        self.time = 0
        self.time_current = 0
        self.time_delta = 0
        self.time_last = 0

        self.width, self.height = width, height

        # ---------------- scene ---------------- #
        
        self._scene = None
        self.light_directional = {
            "direction": [0, -1, 0],
            "ambient": [0.3, 0.3, 0.3],
            "diffuse": [1.0, 1.0, 1.0],
            "specular": [1.0, 1.0, 1.0]
        }

        # ------------------------------------------------------------
        # opengl states
        # ------------------------------------------------------------

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_MULTISAMPLE)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glClearColor(0, 0.1, 0, 1)

        # ------------------------------------------------------------
        # init pybullet
        # ------------------------------------------------------------

        p.connect(p.DIRECT)

    def setup(self):
        # override
        pass

    def update(self):
        # override
        pass

    def run(self):
        """runs the setup and engine loop."""
        # run setup
        self.setup()
        if not self._scene:
            raise Exception("run: scene not created. run create_scene.")
        # run loop
        while not glfw.window_should_close(self.window):
            # calculate time
            self.time_current = glfw.get_time()
            self.time_delta = self.time_current - self.time_last
            self.time_last = self.time_current
            self.time += self.time_delta
            # step physics simulation
            p.setTimeStep(self.time_delta)
            p.stepSimulation()
            # process input
            glfw.poll_events()
            # update scene
            self.update()
            # draw
            glfw.swap_buffers(self.window)

    def draw_scene(self, camera: NightCamera):
        """draws a scene from a camera perspective."""

        # ------------------------------------------------------------
        # clear
        # ------------------------------------------------------------

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        # ------------------------------------------------------------
        # update camera
        # ------------------------------------------------------------

        camera.move(self.window, self.time_delta)
        camera.aspect_ratio = self.width / self.height
        camera.update()

        # ------------------------------------------------------------
        # draw objects
        # ------------------------------------------------------------

        descendants = self._scene.get_descendants(include_self=False)

        for obj in descendants:

            if not obj.visible:
                continue

            # ------------------------------------------------------------
            # update objects from physics
            # ------------------------------------------------------------

            if obj.physics_id != None:
                # update render based on object physical position and orientation
                pos, orn = p.getBasePositionAndOrientation(obj.physics_id)
                obj.set_position(pos, reset_base=False)
                obj.set_rotation(R.from_quat(orn).as_matrix(), reset_base=False)
                # update object link visual representations
                for i in range(p.getNumJoints(obj.physics_id)):
                    link_data = p.getLinkState(obj.physics_id, i)
                    link_pos = link_data[0]
                    link_orn = link_data[1]
                    obj.linkReferences[i].set_position(link_pos, reset_base=False)
                    obj.linkReferences[i].set_rotation(R.from_quat(link_orn).as_matrix(), reset_base=False)
                    
            glUseProgram(obj.material.program)
            glBindVertexArray(obj.vao)
            
            NightUtils.set_uniform(obj.material.program, "matrix_projection", "mat4", camera.matrix_projection)
            NightUtils.set_uniform(obj.material.program, "matrix_view",       "mat4", camera.matrix_view)
            NightUtils.set_uniform(obj.material.program, "matrix_model",      "mat4", obj.get_world_matrix())

            if isinstance(obj.material, NightMaterialDefault):
                # set directional light
                NightUtils.set_uniform(obj.material.program, "light_directional.direction", "vec3", self.light_directional["direction"])
                NightUtils.set_uniform(obj.material.program, "light_directional.ambient", "vec3", self.light_directional["ambient"])
                NightUtils.set_uniform(obj.material.program, "light_directional.diffuse", "vec3", self.light_directional["diffuse"])
                NightUtils.set_uniform(obj.material.program, "light_directional.specular", "vec3", self.light_directional["specular"])
                # set material qualities
                NightUtils.set_uniform(obj.material.program, "material.shininess", "float", obj.material.shininess)
                NightUtils.set_uniform(obj.material.program, "material.ambient", "vec3", obj.material.ambient)
                NightUtils.set_uniform(obj.material.program, "material.diffuse", "vec3", obj.material.diffuse)
                NightUtils.set_uniform(obj.material.program, "material.specular", "vec3", obj.material.specular)
                # camera pos for specular reflection
                NightUtils.set_uniform(obj.material.program, "view_pos", "vec3", camera.get_position())

            obj.material.update_draw_settings()

            glDrawArrays(obj.material.gl_draw_style, 0, obj.mesh.vertex_count)

    def create_scene(self):
        self._scene = NightObject()
        return self._scene

    def set_gravity(self, x=0.0, y=-9.8, z=0.0):
        """wrpper for pybullet setGravity"""
        p.setGravity(x, y, z)
        
    def _callback_framebuffer_size(self, window, width, height):
        """updates viewport and recalculates camera aspect ratio."""
        self.width, self.height = width, height
        glViewport(0, 0, self.width, self.height)

    def _callback_cursor_pos(self, window, xpos, ypos):
        pass

    def _callback_scroll(self, window, xoffset, yoffset):
        pass

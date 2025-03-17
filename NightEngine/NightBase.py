# NightBase.py

import glfw
from OpenGL.GL import *

from NightEngine.NightObject import NightObject
from NightEngine.NightUtils import NightUtils
from NightEngine.NightCamera import NightCamera
from scipy.spatial.transform import Rotation as R
import pybullet as p

class NightBase:
    def __init__(self,
                 width=1200,
                 height=800,
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

        # --------------- keyboard --------------- #

        self.keys_pressed = []

        # ---------------- camera ---------------- #

        self.fov = 70
        self.aspect_ratio = width / height
        self.near = 0.1
        self.far = 1000

        self._scene = None

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
        # init physics
        descendants = self._scene.get_descendants(include_self=False)
        for obj in descendants:
            obj.init_physics()
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
        camera.update(fov=self.fov, aspect_ratio=self.aspect_ratio,
                      near=self.near, far=self.far)

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
                # get pos and orientation
                pos, orn = p.getBasePositionAndOrientation(obj.physics_id)
                # position
                pos = [pos[0], pos[2], pos[1]]
                obj.set_position(pos)
                # rotation
                rotation = R.from_quat(orn)
                rotation_matrix = rotation.as_matrix()
                rotation_matrix = rotation_matrix[[0, 2, 1], :]
                rotation_matrix[:, 2] *= -1 # flip determinant so inside out
                obj.set_rotation(rotation_matrix)
                
            glUseProgram(obj.material.program)
            glBindVertexArray(obj.vao)
            
            NightUtils.set_uniform(obj.material.program, "matrix_projection", "mat4", camera.matrix_projection)
            NightUtils.set_uniform(obj.material.program, "matrix_view",       "mat4", camera.matrix_view)
            NightUtils.set_uniform(obj.material.program, "matrix_model",      "mat4", obj.get_world_matrix())

            obj.material.update_draw_settings()

            glDrawArrays(obj.material.gl_draw_style, 0, obj.mesh.vertex_count)

    def create_scene(self):
        self._scene = NightObject()
        return self._scene

    def set_gravity(self, x=0.0, y=0.0, z=-9.8):
        """wrpper for pybullet setGravity"""
        p.setGravity(x, y, z)
        
    def _callback_framebuffer_size(self, window, width, height):
        """updates viewport and recalculates camera aspect ratio."""
        glViewport(0, 0, width, height)
        self.aspect_ratio = width / height

    def _callback_cursor_pos(self, window, xpos, ypos):
        pass

    def _callback_scroll(self, window, xoffset, yoffset):
        """updates fov based on mouse scroll callback."""
        self.fov -= yoffset;
        if self.fov < 1.0:
            self.fov = 1.0
        if self.fov > 110.0:
            self.fov = 110

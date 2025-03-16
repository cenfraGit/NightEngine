# NightBase.py

import glfw
from OpenGL.GL import *

from NightEngine.NightObject import NightObject
from NightEngine.NightUtils import NightUtils
from NightEngine.NightCamera import NightCamera

class NightBase:
    def __init__(self,
                 width=800,
                 height=600,
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
        self.aspect_ratio = 1
        self.near = 0.1
        self.far = 1000

        # ------------------------------------------------------------
        # opengl states
        # ------------------------------------------------------------

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_MULTISAMPLE)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glClearColor(0, 0.1, 0, 1)

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
        # run loop
        while not glfw.window_should_close(self.window):
            # calculate time
            self.time_current = glfw.get_time()
            self.time_delta = self.time_current - self.time_last
            self.time_last = self.time_current
            self.time += self.time_delta
            # process input
            glfw.poll_events()
            self._process_keyboard_input()
            # update scene
            self.update()
            # draw
            glfw.swap_buffers(self.window)

    def draw_scene(self, scene: NightObject, camera: NightCamera):
        """draws a scene from a camera perspective."""

        # ------------------------------------------------------------
        # clear
        # ------------------------------------------------------------

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        # ------------------------------------------------------------
        # update camera
        # ------------------------------------------------------------

        camera.move(self.keys_pressed, self.time_delta)
        camera.update(fov=self.fov, aspect_ratio=self.aspect_ratio,
                      near=self.near, far=self.far)

        # ------------------------------------------------------------
        # draw objects
        # ------------------------------------------------------------

        descendants = scene.get_descendants(include_self=False)

        for obj in descendants:

            if not obj.visible:
                continue
            
            glUseProgram(obj.material.program)
            glBindVertexArray(obj.vao)
            
            NightUtils.set_uniform(obj.material.program, "matrix_projection", "mat4", camera.matrix_projection)
            NightUtils.set_uniform(obj.material.program, "matrix_view",       "mat4", camera.matrix_view)
            NightUtils.set_uniform(obj.material.program, "matrix_model",      "mat4", obj.get_world_matrix())

            obj.material.update_draw_settings()

            # glDrawArrays(obj.material.gl_draw_style, 0, 8)
            # glDrawArrays(GL_POINTS, 0, 8)
            glDrawArrays(GL_TRIANGLES, 0, 36)

    def _process_keyboard_input(self):
        """updates self.keys_pressed with currently pressed keys."""
        # scanned keys
        keys_to_check = [
            glfw.KEY_ESCAPE, glfw.KEY_W, glfw.KEY_S, glfw.KEY_A,
            glfw.KEY_D, glfw.KEY_SPACE, glfw.KEY_LEFT_SHIFT,
            glfw.KEY_RIGHT, glfw.KEY_LEFT, glfw.KEY_UP, glfw.KEY_DOWN]
        # start with empty list
        self.keys_pressed = []
        for key in keys_to_check:
            if glfw.get_key(self.window, key) == glfw.PRESS:
                if key == glfw.KEY_ESCAPE:
                    glfw.set_window_should_close(self.window, True)
                else:
                    self.keys_pressed.append(key)
        
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
        if self.fov > 90.0:
            self.fov = 90


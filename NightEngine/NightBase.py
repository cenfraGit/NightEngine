# NightBase.py

from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Materials.NightMaterialTexture import NightMaterialTexture
from NightEngine.Materials.NightMaterialLight import NightMaterialLight
from NightEngine.Objects.NightObject import NightObject
from NightEngine.Objects.NightLink import NightLink
from NightEngine.NightUtils import NightUtils
from NightEngine.NightCamera import NightCamera
from NightEngine.NightShadow import NightShadow
from NightEngine.NightVision import NightVisionCamera, NightVisionServer
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
        glClearColor(0.0, 0.0, 0.0, 1)

        # ------------------------------------------------------------
        # shadows (directional light shadow mapping)
        # ------------------------------------------------------------

        self.shadows_enabled = True
        self.shadow_target = [0, 0, 0]   # center of the shadow box
        self.shadow_size = 60.0          # half-width of the shadow box
        self.shadow_near = 1.0
        self.shadow_far = 300.0
        self.shadow_distance = 100.0     # light "position" distance from target
        self.shadow_bias = 0.0015
        self.shadow = NightShadow(resolution=2048)

        # ------------------------------------------------------------
        # machine vision (offscreen cameras + capture server)
        # ------------------------------------------------------------

        self.vision_cameras = []
        self.vision_server = None

        # ------------------------------------------------------------
        # init pybullet
        # ------------------------------------------------------------

        p.connect(p.DIRECT)

    def setup(self):
        # override
        pass

    def setup_physics(self):
        # override. called once after all physics bodies have been
        # created, before the loop starts. use it for changeDynamics,
        # initial velocities, etc.
        pass

    def physics_update(self, time_step: float):
        # override. called once per fixed physics step, right before
        # stepSimulation. apply forces/torques/velocities here so they
        # act on every substep regardless of framerate.
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
        # init multiobjects (not links)
        descendants = self._scene.get_descendants(include_self=False)
        for obj in descendants:
            if isinstance(obj, NightLink):
                continue
            obj.init_multibody()
        self.setup_physics()
        # set time step
        fixed_time_step = 1.0 / 240.0
        accumulated_time = 0.0
        p.setTimeStep(fixed_time_step)
        self._interp_alpha = 1.0
        # capture initial physics state so interpolation has a starting point
        for obj in self._get_physics_objects():
            obj._physics_state_curr = self._get_physics_state(obj)
            obj._physics_state_prev = obj._physics_state_curr
        # run loop
        while not glfw.window_should_close(self.window):
            # calculate time
            self.time_current = glfw.get_time()
            self.time_delta = self.time_current - self.time_last
            self.time_delta = min(self.time_delta, 1.0 / 30.0)
            self.time_last = self.time_current
            self.time += self.time_delta
            accumulated_time += self.time_delta
            # step physics simulation
            while accumulated_time >= fixed_time_step:
                physics_objects = self._get_physics_objects()
                for obj in physics_objects:
                    obj._physics_state_prev = obj._physics_state_curr
                self.physics_update(fixed_time_step)
                p.stepSimulation()
                for obj in physics_objects:
                    obj._physics_state_curr = self._get_physics_state(obj)
                accumulated_time -= fixed_time_step
            # how far we are between the last two physics steps (for
            # render interpolation)
            self._interp_alpha = accumulated_time / fixed_time_step
            # process input
            glfw.poll_events()
            # update scene
            self.update()
            # serve machine-vision clients (captures render here, on
            # the gl thread)
            if self.vision_server:
                self.vision_server.process()
            # draw
            glfw.swap_buffers(self.window)

    def _get_physics_objects(self):
        return [obj for obj in self._scene.get_descendants(include_self=False)
                if obj.physics_id is not None]

    def _get_physics_state(self, obj):
        """reads base and link positions/orientations from pybullet."""
        pos, orn = p.getBasePositionAndOrientation(obj.physics_id)
        links = []
        num_links = p.getNumJoints(obj.physics_id)
        if num_links:
            if hasattr(p, "getLinkStates"):
                states = p.getLinkStates(obj.physics_id, list(range(num_links)))
            else:
                states = [p.getLinkState(obj.physics_id, i) for i in range(num_links)]
            links = [(s[0], s[1]) for s in states]
        return {"pos": pos, "orn": orn, "links": links}

    @staticmethod
    def _interpolate_states(prev, curr, alpha):
        """blends two physics states for stutter-free rendering."""
        if prev is None or prev is curr or alpha >= 1.0:
            return curr
        pos = [c * alpha + q * (1.0 - alpha) for q, c in zip(prev["pos"], curr["pos"])]
        orn = p.getQuaternionSlerp(prev["orn"], curr["orn"], alpha)
        links = []
        for (pos_prev, orn_prev), (pos_curr, orn_curr) in zip(prev["links"], curr["links"]):
            link_pos = [c * alpha + q * (1.0 - alpha) for q, c in zip(pos_prev, pos_curr)]
            link_orn = p.getQuaternionSlerp(orn_prev, orn_curr, alpha)
            links.append((link_pos, link_orn))
        return {"pos": pos, "orn": orn, "links": links}

    def draw_scene(self, camera: NightCamera, width=None, height=None, framebuffer=0):
        """draws a scene from a camera perspective. by default renders
        to the window; pass width/height/framebuffer to render
        offscreen (used by vision cameras)."""

        width = width if width else self.width
        height = height if height else self.height

        # ------------------------------------------------------------
        # update camera
        # ------------------------------------------------------------

        camera.aspect_ratio = width / height
        camera.update()

        # ------------------------------------------------------------
        # update objects from physics
        # ------------------------------------------------------------

        descendants = self._scene.get_descendants(include_self=False)

        alpha = getattr(self, "_interp_alpha", 1.0)

        for obj in descendants:

            if not obj.visible or obj.physics_id is None:
                continue

            # update render transform from the captured physics
            # states, interpolated for stutter-free rendering
            state = self._interpolate_states(obj._physics_state_prev,
                                             obj._physics_state_curr,
                                             alpha)
            if state is None:
                state = self._get_physics_state(obj)
            obj.set_position(state["pos"], reset_base=False)
            obj.set_rotation(R.from_quat(state["orn"]).as_matrix(), reset_base=False)
            # update object link visual representations
            for i, (link_pos, link_orn) in enumerate(state["links"]):
                obj.linkReferences[i].set_position(link_pos, reset_base=False)
                obj.linkReferences[i].set_rotation(R.from_quat(link_orn).as_matrix(), reset_base=False)

        # ------------------------------------------------------------
        # shadow depth pass (scene from the light's point of view)
        # ------------------------------------------------------------

        matrix_light = None
        if self.shadows_enabled:
            matrix_light = self.shadow.get_light_matrix(
                self.light_directional["direction"],
                target=self.shadow_target,
                size=self.shadow_size,
                near=self.shadow_near,
                far=self.shadow_far,
                distance=self.shadow_distance)
            self.shadow.begin(matrix_light)
            for obj in descendants:
                if not obj.visible or not obj.mesh or not obj.material:
                    continue
                if not obj.cast_shadow:
                    continue
                # only lit triangle geometry casts shadows (skips sky
                # domes, grids, axes, light gizmos)
                if not getattr(obj.material, "lighting", False):
                    continue
                if obj.material.gl_draw_style != GL_TRIANGLES:
                    continue
                self.shadow.draw(obj)
            self.shadow.end()

        # ------------------------------------------------------------
        # bind target framebuffer and clear
        # ------------------------------------------------------------

        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer)
        glViewport(0, 0, width, height)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        # ------------------------------------------------------------
        # draw objects
        # ------------------------------------------------------------

        for obj in descendants:

            if not obj.visible:
                continue

            # transform-only nodes (cameras, group nodes) have nothing to draw
            if not obj.mesh or not obj.material:
                continue

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
                # shadows
                NightUtils.set_uniform(obj.material.program, "bool_shadows", "bool", self.shadows_enabled)
                if self.shadows_enabled:
                    NightUtils.set_uniform(obj.material.program, "matrix_light", "mat4", matrix_light)
                    NightUtils.set_uniform(obj.material.program, "shadow_map", "sampler2D", [self.shadow.texture, 7])
                    NightUtils.set_uniform(obj.material.program, "shadow_bias", "float", self.shadow_bias)

            if isinstance(obj.material, NightMaterialTexture):
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
                # shadows
                NightUtils.set_uniform(obj.material.program, "bool_shadows", "bool", self.shadows_enabled)
                if self.shadows_enabled:
                    NightUtils.set_uniform(obj.material.program, "matrix_light", "mat4", matrix_light)
                    NightUtils.set_uniform(obj.material.program, "shadow_map", "sampler2D", [self.shadow.texture, 7])
                    NightUtils.set_uniform(obj.material.program, "shadow_bias", "float", self.shadow_bias)
                # texture setup
                NightUtils.set_uniform(obj.material.program, "uv_repeat", "vec2", [1.0, 1.0])
                NightUtils.set_uniform(obj.material.program, "uv_offset", "vec2", [0.0, 0.0])
                NightUtils.set_uniform(obj.material.program, "texture_diffuse", "sampler2D", [obj.material.gl_texture, 1])

            obj.material.update_draw_settings()

            glDrawArrays(obj.material.gl_draw_style, 0, obj.mesh.vertex_count)

    def create_scene(self):
        self._scene = NightObject()
        return self._scene

    # ------------------------------------------------------------
    # machine vision
    # ------------------------------------------------------------

    def add_vision_camera(self, name=None, position=[0, 10, 10], target=[0, 0, 0],
                          resolution=(640, 480), fov=60.0, near=0.1, far=1000.0):
        """adds an offscreen inspection camera. returns the
        NightVisionCamera; its index (for the protocol) is the order
        of creation. attach vision_camera.camera to an object with
        obj.add(...) for a moving camera."""
        index = len(self.vision_cameras)
        vision_camera = NightVisionCamera(name=name if name else f"camera{index}",
                                          width=resolution[0],
                                          height=resolution[1],
                                          fov=fov, near=near, far=far)
        vision_camera.camera.look_at(position=position, target=target)
        self.vision_cameras.append(vision_camera)
        return vision_camera

    def start_vision_server(self, host="127.0.0.1", port=8555):
        """starts the tcp server that lets external programs (opencv
        scripts etc.) capture frames from the vision cameras."""
        self.vision_server = NightVisionServer(self, host, port)
        return self.vision_server

    def vision_trigger(self, name, value):
        """override to implement custom vision-triggered behavior
        (lighting controllers, actuators...). return True if the
        trigger was handled."""
        return False

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

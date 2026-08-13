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

# must match MAX_POINT_LIGHTS in the lit material shaders
MAX_POINT_LIGHTS = 8


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

        # previous state per key, for key_just_pressed
        self._key_states = {}

        # ---------------- scene ---------------- #
        
        self._scene = None
        self.light_directional = {
            "direction": [0, -1, 0],
            "ambient": [0.3, 0.3, 0.3],
            "diffuse": [1.0, 1.0, 1.0],
            "specular": [1.0, 1.0, 1.0]
        }
        # point lights (flash/strobe/work lights/inspection ring). color
        # is rgb premultiplied by intensity; black = off. lights[0] is
        # also reachable as self.light_point for backward compatibility.
        self.lights = [self.make_light()]

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
        self.gige_server = None
        self.gardasoft_server = None

        # ------------------------------------------------------------
        # init pybullet
        # ------------------------------------------------------------

        p.connect(p.DIRECT)

    # ------------------------------------------------------------
    # point lights
    # ------------------------------------------------------------

    @staticmethod
    def make_light(position=[0, 10, 0], color=[0.0, 0.0, 0.0],
                   attenuation_linear=0.02, attenuation_quadratic=0.002):
        """a point light's parameter dict, in the shape the shaders expect."""
        return {"position": list(position),
                "color": list(color),
                "attenuation_linear": attenuation_linear,
                "attenuation_quadratic": attenuation_quadratic}

    @property
    def light_point(self):
        """the first point light. kept so code written against the old
        single-light api keeps working, including in-place mutation such
        as light_point["color"] = [...] -- this returns the live dict."""
        return self.lights[0]

    @light_point.setter
    def light_point(self, value):
        self.lights[0] = value

    def set_light_count(self, count):
        """grows or shrinks the point light list. extra lights start off
        (black), so adding them changes nothing until they are given a
        colour."""
        if count > MAX_POINT_LIGHTS:
            raise ValueError(f"at most {MAX_POINT_LIGHTS} point lights are "
                             f"supported by the shaders (asked for {count})")
        while len(self.lights) < count:
            self.lights.append(self.make_light())
        del self.lights[count:]
        return self.lights

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
            if self.gardasoft_server:
                self.gardasoft_server.process()
            if self.gige_server:
                self.gige_server.process()
            # draw
            glfw.swap_buffers(self.window)

    def _set_point_light_uniforms(self, program):
        """uploads the active point lights as a shader array. only the
        active count is sent; the shader loops to point_light_count."""
        count = min(len(self.lights), MAX_POINT_LIGHTS)
        NightUtils.set_uniform(program, "point_light_count", "int", count)
        for index in range(count):
            light = self.lights[index]
            base = f"point_lights[{index}]"
            NightUtils.set_uniform(program, f"{base}.position", "vec3", light["position"])
            NightUtils.set_uniform(program, f"{base}.color", "vec3", light["color"])
            NightUtils.set_uniform(program, f"{base}.attenuation_linear", "float",
                                   light["attenuation_linear"])
            NightUtils.set_uniform(program, f"{base}.attenuation_quadratic", "float",
                                   light["attenuation_quadratic"])

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
                # point lights
                self._set_point_light_uniforms(obj.material.program)

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
                # point lights
                self._set_point_light_uniforms(obj.material.program)
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
                          resolution=(640, 480), fov=60.0, near=0.1, far=1000.0,
                          show_body=False, body_color=[0.85, 0.85, 0.2],
                          body_scale=1.0, frustum_depth=6.0):
        """adds an offscreen inspection camera. returns the
        NightVisionCamera; its index (for the protocol) is the order
        of creation. attach vision_camera.camera to an object with
        obj.add(...) for a moving camera.

        show_body draws a housing + wireframe frustum at the camera's
        pose so the camera is visible in the main view. the gizmos are
        hidden while that camera captures, so it never sees itself."""
        index = len(self.vision_cameras)
        vision_camera = NightVisionCamera(name=name if name else f"camera{index}",
                                          width=resolution[0],
                                          height=resolution[1],
                                          fov=fov, near=near, far=far)
        vision_camera.camera.look_at(position=position, target=target)
        if show_body:
            vision_camera.build_body(color=body_color, scale=body_scale,
                                     frustum_depth=frustum_depth)
            # the gizmos only render if the camera node is in the scene
            # graph; leave an already-attached camera where it is
            if vision_camera.camera.parent is None:
                self._scene.add(vision_camera.camera)
        self.vision_cameras.append(vision_camera)
        return vision_camera

    def start_vision_server(self, host="127.0.0.1", port=8555):
        """starts the tcp server that lets external programs (opencv
        scripts etc.) capture frames from the vision cameras."""
        self.vision_server = NightVisionServer(self, host, port)
        return self.vision_server

    def start_gige_server(self, devices, bind_broadcast=True, verbose=True,
                          bind_any=False, log_gvcp=False):
        """exposes vision cameras as GigE Vision devices, discoverable by
        any standard consumer (HALCON, pylon Viewer, eBUS Player, Aravis).

        each entry in `devices` is a dict of NightGigEDevice arguments,
        e.g. {"camera": cam0, "ip": "169.254.5.60",
              "mac": "02:00:00:05:00:3c", "model": "NightEngineCam",
              "serial": "NE0001", "pixel_format": "Mono8"}

        the standard fixes the control port at 3956, so every device
        needs its own local IP address.

        bind_any binds 0.0.0.0:3956 instead of the device address, as a
        fallback if a consumer's discovery broadcast never reaches us.
        single device only."""
        from NightEngine.GigE.server import NightGigEServer
        self.gige_server = NightGigEServer(self, devices=devices,
                                           bind_broadcast=bind_broadcast,
                                           verbose=verbose,
                                           bind_any=bind_any,
                                           log_gvcp=log_gvcp)
        return self.gige_server

    def start_gardasoft_controller(self, ip="127.0.0.1", serial=12345,
                                   mac="00:0B:75:01:80:99", http_port=80,
                                   verbose=True, bind_any=False):
        """exposes an emulated Gardasoft CC320 trigger timing controller on
        the network. Bind its outputs to scene lights and camera triggers
        with controller.bind_light(...) / controller.bind_camera(...)."""
        from NightEngine.Gardasoft.server import NightGardasoftServer
        self.gardasoft_server = NightGardasoftServer(
            self, ip=ip, serial=serial, mac=mac, http_port=http_port,
            verbose=verbose, bind_any=bind_any)
        return self.gardasoft_server

    def vision_trigger(self, name, value):
        """override to implement custom vision-triggered behavior
        (lighting controllers, actuators...). return True if the
        trigger was handled."""
        return False

    def set_gravity(self, x=0.0, y=-9.8, z=0.0):
        """wrpper for pybullet setGravity"""
        p.setGravity(x, y, z)

    def key_just_pressed(self, key):
        """True only on the frame a key goes down.

        Input here is polled rather than event-driven, so a held key reads
        as pressed every frame; anything that should happen once per press
        needs the previous state kept somewhere. Keeping it here rather
        than re-latching it in each example also means one key cannot be
        watched by two callers in the same frame -- the second would see
        the state already consumed."""
        down = glfw.get_key(self.window, key) == glfw.PRESS
        was_down = self._key_states.get(key, False)
        self._key_states[key] = down
        return down and not was_down
        
    def _callback_framebuffer_size(self, window, width, height):
        """updates viewport and recalculates camera aspect ratio."""
        self.width, self.height = width, height
        glViewport(0, 0, self.width, self.height)

    def _callback_cursor_pos(self, window, xpos, ypos):
        pass

    def _callback_scroll(self, window, xoffset, yoffset):
        pass

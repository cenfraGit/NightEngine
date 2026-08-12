# NightVision.py
# machine-vision support: offscreen "inspection" cameras that render
# into their own framebuffers, and a TCP server that lets external
# programs (e.g. opencv scripts) capture frames, re-aim cameras and
# trigger lighting -- similar to an industrial vision setup.
#
# protocol (one request per line, json):
#   {"cmd": "ping"}
#   {"cmd": "list_cameras"}
#   {"cmd": "capture", "camera": 0}
#   {"cmd": "set_camera", "camera": 0, "position": [x,y,z],
#    "look_at": [x,y,z], "fov": 60}
#   {"cmd": "set_light", "direction": [...], "ambient": [...],
#    "diffuse": [...], "specular": [...], "shadows": true}
#   {"cmd": "trigger", "name": "ring_light", "value": 1.0}
#
# every response is a json header line. capture responses are followed
# by exactly header["bytes"] of raw bgr8 pixel data (rows top to
# bottom), ready for numpy/opencv.

from NightEngine.NightCamera import NightCamera
from NightEngine.Objects.NightObject import NightObject
from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Meshes.NightMesh import NightMesh
from NightEngine.Meshes.MeshBox import MeshBox
from OpenGL.GL import *
import numpy as np
import threading
import socket
import queue
import json
import math


class NightVisionCamera:
    """an offscreen camera with its own framebuffer. capture() renders
    the scene from it and returns a (height, width, 3) bgr8 array."""

    def __init__(self, name="camera", width=640, height=480,
                 fov=60.0, near=0.1, far=1000.0):

        self.name = name
        self.width = width
        self.height = height

        self.camera = NightCamera(fov=fov,
                                  aspect_ratio=width / height,
                                  near=near,
                                  far=far)

        # visual gizmos (housing + frustum) parented to the camera
        # transform. hidden while this camera renders so it never
        # photographs itself.
        self.body_objects = []

        # ------------------------------------------------------------
        # offscreen framebuffer (color + depth renderbuffers)
        # ------------------------------------------------------------

        self.fbo = glGenFramebuffers(1)
        glBindFramebuffer(GL_FRAMEBUFFER, self.fbo)

        rb_color = glGenRenderbuffers(1)
        glBindRenderbuffer(GL_RENDERBUFFER, rb_color)
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB8, width, height)
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                  GL_RENDERBUFFER, rb_color)

        rb_depth = glGenRenderbuffers(1)
        glBindRenderbuffer(GL_RENDERBUFFER, rb_depth)
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height)
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                  GL_RENDERBUFFER, rb_depth)

        if glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE:
            raise Exception("NightVisionCamera: framebuffer incomplete.")
        glBindFramebuffer(GL_FRAMEBUFFER, 0)

    # ------------------------------------------------------------
    # visual body (so the camera is visible in the main view)
    # ------------------------------------------------------------

    def build_body(self, color=[0.85, 0.85, 0.2], scale=1.0, frustum_depth=6.0):
        """creates a housing box and a wireframe view frustum, parented
        to this camera's transform so they follow it. the frustum is
        built from the fov at call time; call again after changing fov."""

        # ---------------- housing ---------------- #

        housing = NightObject(MeshBox(0.7 * scale, 0.7 * scale, 1.1 * scale,
                                      color=color, collision=False),
                              NightMaterialDefault(), 0)
        # sit the body behind the lens origin (the camera looks down +z)
        housing.set_position([0, 0, -0.55 * scale])

        # ---------------- frustum ---------------- #

        half_h = math.tan(math.radians(self.camera.fov) / 2.0) * frustum_depth
        half_w = half_h * (self.width / self.height)
        d = frustum_depth
        corners = [[-half_w, -half_h, d], [half_w, -half_h, d],
                   [half_w, half_h, d], [-half_w, half_h, d]]
        origin = [0.0, 0.0, 0.0]

        positions = []
        for c in corners:                       # rays from the lens
            positions += [origin, c]
        for i in range(4):                      # rectangle at the far end
            positions += [corners[i], corners[(i + 1) % 4]]

        mesh = NightMesh()
        mesh.add_attribute("vertex_position", "vec3", positions)
        mesh.add_attribute("vertex_color", "vec3", [color] * len(positions))
        mesh.vertex_count = len(positions)

        frustum = NightObject(mesh,
                              NightMaterialDefault(gl_draw_style=GL_LINES,
                                                   gl_line_width=1,
                                                   gl_culling=False,
                                                   lighting=False), 0)

        for obj in (housing, frustum):
            obj.cast_shadow = False             # gizmos must not shadow the scene
            self.camera.add(obj)
            self.body_objects.append(obj)

        return self.body_objects

    def capture(self, engine):
        """renders the scene from this camera and reads the pixels
        back. must run on the render (main) thread."""

        # a camera must not appear in its own image
        hidden = [o for o in self.body_objects if o.visible]
        for obj in hidden:
            obj.visible = False
        try:
            engine.draw_scene(self.camera,
                              width=self.width,
                              height=self.height,
                              framebuffer=self.fbo)
            glBindFramebuffer(GL_FRAMEBUFFER, self.fbo)
            glPixelStorei(GL_PACK_ALIGNMENT, 1)
            data = glReadPixels(0, 0, self.width, self.height,
                                GL_BGR, GL_UNSIGNED_BYTE)
            glBindFramebuffer(GL_FRAMEBUFFER, 0)
        finally:
            for obj in hidden:
                obj.visible = True

        frame = np.frombuffer(data, dtype=np.uint8).reshape(self.height, self.width, 3)
        return np.ascontiguousarray(np.flipud(frame))  # gl rows are bottom-up

    # ------------------------------------------------------------
    # pixel format conversion (for transports that need mono/rgb)
    # ------------------------------------------------------------

    @staticmethod
    def to_mono8(frame_bgr):
        """bgr8 -> mono8 using itu-r 601 luma weights."""
        w = np.array([0.114, 0.587, 0.299], dtype=np.float32)   # b, g, r
        return (frame_bgr.astype(np.float32) @ w).astype(np.uint8)

    @staticmethod
    def to_rgb8(frame_bgr):
        return np.ascontiguousarray(frame_bgr[:, :, ::-1])


class NightVisionServer:
    """tcp server for external vision clients. socket threads only do
    i/o: every request is queued and executed on the render thread via
    process(), which NightBase calls once per frame."""

    def __init__(self, engine, host="127.0.0.1", port=8555):
        self.engine = engine
        self.host = host
        self.port = port
        self._requests = queue.Queue()
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((host, port))
        self._server.listen(4)
        threading.Thread(target=self._accept_loop, daemon=True).start()
        print(f"NightVisionServer: listening on {host}:{port}")

    # ------------------------------------------------------------
    # socket threads
    # ------------------------------------------------------------

    def _accept_loop(self):
        while True:
            try:
                conn, addr = self._server.accept()
            except OSError:
                break
            threading.Thread(target=self._client_loop, args=(conn,), daemon=True).start()

    def _client_loop(self, conn):
        stream = conn.makefile("rb")
        try:
            while True:
                line = stream.readline()
                if not line:
                    break
                try:
                    request = json.loads(line)
                except ValueError:
                    header, blob = {"ok": False, "error": "invalid json"}, b""
                else:
                    item = {"request": request,
                            "event": threading.Event(),
                            "response": None}
                    self._requests.put(item)
                    if item["event"].wait(timeout=5.0):
                        header, blob = item["response"]
                    else:
                        header, blob = {"ok": False, "error": "engine timeout"}, b""
                conn.sendall((json.dumps(header) + "\n").encode() + blob)
        except OSError:
            pass
        finally:
            conn.close()

    # ------------------------------------------------------------
    # render thread
    # ------------------------------------------------------------

    def process(self):
        """executes pending client requests. called by the engine loop
        once per frame, on the render thread."""
        while True:
            try:
                item = self._requests.get_nowait()
            except queue.Empty:
                break
            try:
                item["response"] = self._handle(item["request"])
            except Exception as e:
                item["response"] = ({"ok": False, "error": str(e)}, b"")
            item["event"].set()

    def _handle(self, request):
        engine = self.engine
        cmd = request.get("cmd")

        if cmd == "ping":
            return ({"ok": True, "time": engine.time}, b"")

        if cmd == "list_cameras":
            cameras = [{"index": i,
                        "name": vc.name,
                        "width": vc.width,
                        "height": vc.height,
                        "fov": vc.camera.fov}
                       for i, vc in enumerate(engine.vision_cameras)]
            return ({"ok": True, "cameras": cameras}, b"")

        if cmd == "capture":
            index = int(request.get("camera", 0))
            if not (0 <= index < len(engine.vision_cameras)):
                return ({"ok": False, "error": f"no camera {index}"}, b"")
            vc = engine.vision_cameras[index]
            blob = vc.capture(engine).tobytes()
            return ({"ok": True,
                     "camera": index,
                     "width": vc.width,
                     "height": vc.height,
                     "channels": 3,
                     "format": "bgr8",
                     "bytes": len(blob),
                     "time": engine.time}, blob)

        if cmd == "set_camera":
            index = int(request.get("camera", 0))
            if not (0 <= index < len(engine.vision_cameras)):
                return ({"ok": False, "error": f"no camera {index}"}, b"")
            vc = engine.vision_cameras[index]
            if "fov" in request:
                vc.camera.fov = float(request["fov"])
            if "position" in request or "look_at" in request:
                vc.camera.look_at(position=request.get("position"),
                                  target=request.get("look_at", [0, 0, 0]))
            return ({"ok": True}, b"")

        if cmd == "set_light":
            for key in ("direction", "ambient", "diffuse", "specular"):
                if key in request:
                    engine.light_directional[key] = list(request[key])
            if "shadows" in request:
                engine.shadows_enabled = bool(request["shadows"])
            return ({"ok": True}, b"")

        if cmd == "trigger":
            handled = engine.vision_trigger(request.get("name"),
                                            request.get("value"))
            return ({"ok": True, "handled": bool(handled)}, b"")

        return ({"ok": False, "error": f"unknown cmd: {cmd}"}, b"")

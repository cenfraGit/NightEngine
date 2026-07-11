# vision_client.py
#
# opencv client for the NightEngine vision server. run machinevision.py
# first, then:
#
#   python vision_client.py                # snap every camera -> png
#   python vision_client.py --view         # live view + edges (q quits)
#   python vision_client.py --lighting     # lighting controller demo
#
# the client class is self-contained: copy it into your own cv scripts.

import argparse
import socket
import json
import time

import numpy as np
import cv2


class NightVisionClient:
    """talks to a NightEngine vision server. frames come back as
    numpy bgr8 arrays, directly usable with opencv."""

    def __init__(self, host="127.0.0.1", port=8555):
        self._sock = socket.create_connection((host, port))
        self._stream = self._sock.makefile("rb")

    def close(self):
        self._sock.close()

    def _request(self, payload):
        self._sock.sendall((json.dumps(payload) + "\n").encode())
        header = json.loads(self._stream.readline())
        blob = b""
        n = header.get("bytes", 0)
        while len(blob) < n:
            chunk = self._stream.read(n - len(blob))
            if not chunk:
                raise ConnectionError("server closed connection")
            blob += chunk
        return header, blob

    def ping(self):
        header, _ = self._request({"cmd": "ping"})
        return header

    def list_cameras(self):
        header, _ = self._request({"cmd": "list_cameras"})
        if not header["ok"]:
            raise RuntimeError(header.get("error"))
        return header["cameras"]

    def capture(self, camera=0):
        """returns one frame as a (height, width, 3) bgr8 numpy array."""
        header, blob = self._request({"cmd": "capture", "camera": camera})
        if not header["ok"]:
            raise RuntimeError(header.get("error"))
        return np.frombuffer(blob, dtype=np.uint8).reshape(
            header["height"], header["width"], header["channels"])

    def set_camera(self, camera, position=None, look_at=None, fov=None):
        payload = {"cmd": "set_camera", "camera": camera}
        if position is not None:
            payload["position"] = list(position)
        if look_at is not None:
            payload["look_at"] = list(look_at)
        if fov is not None:
            payload["fov"] = fov
        header, _ = self._request(payload)
        return header

    def set_light(self, direction=None, ambient=None, diffuse=None,
                  specular=None, shadows=None):
        payload = {"cmd": "set_light"}
        for key, val in (("direction", direction), ("ambient", ambient),
                         ("diffuse", diffuse), ("specular", specular),
                         ("shadows", shadows)):
            if val is not None:
                payload[key] = val
        header, _ = self._request(payload)
        return header

    def trigger(self, name, value):
        """fires a named trigger (e.g. a lighting controller channel)."""
        header, _ = self._request({"cmd": "trigger", "name": name, "value": value})
        return header


# ------------------------------------------------------------
# demos
# ------------------------------------------------------------

def demo_snap(client):
    """single capture from every camera, saved to png."""
    for cam in client.list_cameras():
        frame = client.capture(cam["index"])
        filename = f"vision_{cam['name']}.png"
        cv2.imwrite(filename, frame)
        print(f"camera {cam['index']} ({cam['name']}): "
              f"{frame.shape[1]}x{frame.shape[0]} -> {filename}")


def demo_view(client, camera):
    """continuous capture with a simple cv pipeline (canny edges)."""
    print("live view: press q to quit")
    t_last, fps = time.time(), 0.0
    while True:
        frame = client.capture(camera)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.cvtColor(cv2.Canny(gray, 60, 160), cv2.COLOR_GRAY2BGR)
        view = np.hstack([frame, edges])

        now = time.time()
        fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_last, 1e-6))
        t_last = now
        cv2.putText(view, f"{fps:5.1f} fps", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("NightEngine vision", view)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()


def demo_lighting(client, camera):
    """lighting controller demo: capture under different lighting."""
    shots = [
        ("normal",      lambda: None),
        ("ring_dim",    lambda: client.trigger("ring_light", 0.25)),
        ("ring_bright", lambda: client.trigger("ring_light", 1.5)),
        ("no_shadows",  lambda: client.trigger("shadows", 0)),
    ]
    for name, action in shots:
        action()
        time.sleep(0.1)  # let a frame render with the new lighting
        frame = client.capture(camera)
        filename = f"vision_light_{name}.png"
        cv2.imwrite(filename, frame)
        print(f"{name}: mean brightness {frame.mean():.1f} -> {filename}")
    # restore defaults
    client.trigger("ring_light", 1.0)
    client.trigger("shadows", 1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8555)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--view", action="store_true", help="live view")
    parser.add_argument("--lighting", action="store_true", help="lighting demo")
    args = parser.parse_args()

    client = NightVisionClient(args.host, args.port)
    print("connected:", client.ping())

    if args.view:
        demo_view(client, args.camera)
    elif args.lighting:
        demo_lighting(client, args.camera)
    else:
        demo_snap(client)

    client.close()

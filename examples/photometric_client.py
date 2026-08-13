# photometric_client.py
#
# Drives a full photometric sweep against the emulated cell and scores the
# result against ground truth:
#
#   1. configures the emulated GigE Vision camera for triggered acquisition
#   2. sends MI1,1 / MI1,0 to the emulated Gardasoft CC320 on TCP 30313
#   3. receives one frame per light over GVSP as OP8 fires the camera
#   4. finds each specular highlight, back-projects it onto the dome, and
#      mirrors the view ray to recover that light's direction
#   5. compares the recovered directions with calibration_groundtruth.json
#
# This is a validation harness, not a replacement for your calibration: it
# exists to prove the rendered frames are calibratable.
#
#   python photometric_client.py
#   python photometric_client.py --ip 169.254.5.60 --camera-ip 169.254.5.61

import argparse
import json
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "tests"))

import numpy as np

from NightEngine.GigE import registers as R
from NightEngine.GigE import protocol as GP
from test_gige_device import Client as GigEClient   # reuse the tested client

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--ip", default="127.0.0.1", help="controller IP")
parser.add_argument("--camera-ip", default=None, help="camera IP")
parser.add_argument("--groundtruth", default="calibration_groundtruth.json")
parser.add_argument("--save", action="store_true", help="write frames as PNG")
args = parser.parse_args()
CAMERA_IP = args.camera_ip or args.ip

GARDASOFT_PORT = 30313


# ------------------------------------------------------------
# the controller
# ------------------------------------------------------------

class ControllerClient:
    """ASCII over TCP. Replies are <echo><data><LF><CR>'>'."""

    def __init__(self, ip, port=GARDASOFT_PORT):
        self.sock = socket.create_connection((ip, port), timeout=5.0)

    def command(self, text):
        self.sock.sendall((text + "\r").encode("ascii"))
        reply = b""
        while not reply.endswith(b">"):
            chunk = self.sock.recv(4096)
            if not chunk:
                break
            reply += chunk
        decoded = reply.decode("ascii", errors="replace")
        if "Err" in decoded:
            raise RuntimeError(f"{text!r} -> {decoded!r}")
        return decoded[len(text):].rstrip(">\r\n")

    def close(self):
        self.sock.close()


# ------------------------------------------------------------
# geometry
# ------------------------------------------------------------

def recover_light_direction(pixel, truth):
    """back-project a highlight pixel onto the dome, then mirror the view ray
    about the surface normal to get the direction toward the light.

    Returns (surface_point, light_direction) or None if the ray misses."""
    camera = truth["camera"]
    intrinsics = camera["intrinsics"]
    eye = np.array(camera["position"], dtype=float)
    right = np.array(camera["right"], dtype=float)
    up = np.array(camera["up"], dtype=float)
    forward = np.array(camera["forward"], dtype=float)
    centre = np.array(truth["sphere"]["centre"], dtype=float)
    radius = float(truth["sphere"]["radius"])

    u, v = pixel
    x = (u - intrinsics["cx"]) / intrinsics["fx"]
    y = (v - intrinsics["cy"]) / intrinsics["fy"]
    # image rows grow downward while the camera's up axis grows upward
    ray = forward + x * right - y * up
    ray /= np.linalg.norm(ray)

    offset = eye - centre
    b = 2.0 * float(np.dot(ray, offset))
    c = float(np.dot(offset, offset)) - radius * radius
    discriminant = b * b - 4.0 * c
    if discriminant < 0:
        return None
    t = (-b - math.sqrt(discriminant)) / 2.0
    if t <= 0:
        return None
    point = eye + t * ray
    normal = (point - centre) / radius
    view = eye - point
    view /= np.linalg.norm(view)
    light = 2.0 * float(np.dot(normal, view)) * normal - view
    return point, light / np.linalg.norm(light)


def highlight_centroid(image, truth):
    """intensity-weighted centroid of the brightest blob inside the dome."""
    height, width = image.shape
    camera, sphere = truth["camera"], truth["sphere"]
    focal = camera["intrinsics"]["fy"]
    distance = float(np.linalg.norm(np.array(camera["position"])
                                    - np.array(sphere["centre"])))
    radius_px = focal * sphere["radius"] / distance
    ys, xs = np.ogrid[:height, :width]
    inside = ((xs - camera["intrinsics"]["cx"]) ** 2
              + (ys - camera["intrinsics"]["cy"]) ** 2) <= radius_px ** 2
    masked = np.where(inside, image.astype(float), 0.0)
    if masked.max() <= 0:
        return None
    weights = np.where(masked >= masked.max() * 0.8, masked, 0.0)
    total = weights.sum()
    grid_y, grid_x = np.mgrid[:height, :width]
    return ((weights * grid_x).sum() / total, (weights * grid_y).sum() / total)


# ------------------------------------------------------------

def main():
    with open(args.groundtruth, encoding="utf-8") as handle:
        truth = json.load(handle)
    lights = truth["lights"]
    print(f"ground truth: {len(lights)} lights, dome r="
          f"{truth['sphere']['radius']}, camera at {truth['camera']['position']}")

    camera = GigEClient(CAMERA_IP)
    controller = ControllerClient(args.ip)
    try:
        print("controller:", controller.command("VR"))

        camera.discover()
        camera.write_reg(R.CCP, 1)
        # A sweep takes ~5s during which we send the camera nothing, so
        # without this the device's 3s heartbeat expires, control is dropped
        # and acquisition stops mid-sweep -- the same mechanism that drops a
        # camera when HALCON pauses at a breakpoint. A real consumer either
        # heartbeats on a timer or widens the window; GevHeartbeatTimeout is
        # writable for exactly this reason.
        camera.write_reg(R.HEARTBEAT_TIMEOUT, 30000)
        width = camera.read_reg(R.REG_WIDTH)
        height = camera.read_reg(R.REG_HEIGHT)
        payload = camera.read_reg(R.REG_PAYLOAD_SIZE)
        camera.write_reg(R.SCDA, R.ip_to_u32(CAMERA_IP))
        camera.write_reg(R.SCP, camera.stream_port)
        camera.write_reg(R.SCPS, R.SCPS_DO_NOT_FRAGMENT | 1500)
        # hardware-triggered acquisition: OP8 provides the trigger
        camera.write_reg(R.REG_TRIGGER_SELECTOR, 0)
        camera.write_reg(R.REG_TRIGGER_SOURCE, 0)
        camera.write_reg(R.REG_TRIGGER_MODE, 1)
        camera.write_reg(R.REG_ACQUISITION_COMMAND, 1)
        print(f"camera: {width}x{height}, payload {payload} B, "
              f"triggered acquisition armed")

        # The ground-truth file carries the intrinsics; the camera carries the
        # frames. If they disagree the file is stale, and mixing them yields
        # confident-looking nonsense rather than an obvious failure -- so
        # refuse rather than guess.
        truth_width, truth_height = truth["camera"]["resolution"]
        if (truth_width, truth_height) != (width, height):
            print(f"\nMISMATCH: {args.groundtruth} describes a "
                  f"{truth_width}x{truth_height} camera but the live camera is "
                  f"{width}x{height}.")
            print("The ground-truth file is written when the scene starts, so "
                  "it is stale.\nRestart photometric_calibration.py (with the "
                  "same --resolution) and retry.")
            return 2

        print("\nstarting the sweep (MI1,1 / MI1,0 on IP1)...")
        controller.command("MI1,1")
        controller.command("MI1,0")

        frames = []
        for index in range(len(lights)):
            leader, blob, _size_y, _pid = camera.receive_frame(timeout=8.0)
            image = np.frombuffer(blob, dtype=np.uint8).reshape(height, width)
            frames.append(image)
            print(f"  frame {index}: block={leader['block_id']} "
                  f"mean={image.mean():6.2f} peak={image.max():3d}")
        camera.write_reg(R.REG_ACQUISITION_COMMAND, 0)

        print("\nrecovered light directions vs ground truth:")
        errors = []
        for index, image in enumerate(frames):
            pixel = highlight_centroid(image, truth)
            if pixel is None:
                print(f"  light {index}: no highlight found")
                errors.append(float("nan"))
                continue
            recovered = recover_light_direction(pixel, truth)
            if recovered is None:
                print(f"  light {index}: highlight ray missed the dome")
                errors.append(float("nan"))
                continue
            point, direction = recovered
            true_position = np.array(lights[index]["position"], dtype=float)
            true_direction = true_position - point
            true_direction /= np.linalg.norm(true_direction)
            dot = float(np.clip(np.dot(direction, true_direction), -1.0, 1.0))
            error = math.degrees(math.acos(dot))
            errors.append(error)
            print(f"  light {index} (OP{lights[index]['output']}): "
                  f"highlight=({pixel[0]:7.2f},{pixel[1]:7.2f}) "
                  f"recovered={np.round(direction, 3).tolist()} "
                  f"error={error:5.2f}deg")
            if args.save:
                from PIL import Image
                Image.fromarray(image).save(f"photometric_{index}.png")

        finite = [e for e in errors if not math.isnan(e)]
        if len(finite) == len(lights):
            print(f"\nall {len(lights)} lights recovered; "
                  f"max angular error {max(finite):.2f}deg")
            if max(finite) < 5.0:
                print("PASS: the rendered frames are calibratable")
                return 0
            print("FAIL: angular error is larger than expected")
            return 1
        print(f"\nFAIL: only {len(finite)}/{len(lights)} lights recovered")
        return 1
    finally:
        controller.close()
        camera.close()


if __name__ == "__main__":
    sys.exit(main())

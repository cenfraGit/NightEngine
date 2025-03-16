# NightCamera.py

from NightEngine.NightObject import NightObject
from NightEngine.NightMatrix import NightMatrix
import numpy as np
import math
import glfw

class NightCamera(NightObject):
    def __init__(self,
                 fov=70,
                 aspect_ratio=1,
                 near=0.1,
                 far=1000):
        
        super().__init__()

        # -------------- properties -------------- #

        self.speed_movement = 10
        self.speed_rotation = 100

        self.fov = fov
        self.aspect_ratio = aspect_ratio
        self.near = near
        self.far = far

        self.pitch = 0
        self.yaw = 0

        self.matrix_projection = NightMatrix.get_perspective(fov, aspect_ratio, near, far)
        self.matrix_view = NightMatrix.get_identity()

    def update(self, fov=None, aspect_ratio=None, near=None, far=None):

        # ------------- update view ------------- #

        position = self.get_position()
        forward = self.get_forward_vector()
        target = position + forward
        self.matrix_view = NightMatrix.get_lookat(position, target, [0, 1, 0])

        # ---------- update perspective ---------- #

        self.fov = fov if fov else self.fov
        self.aspect_ratio = aspect_ratio if aspect_ratio else self.aspect_ratio
        self.near = near if near else self.near
        self.far = far if far else self.far
        self.matrix_projection = NightMatrix.get_perspective(self.fov,
                                                             self.aspect_ratio,
                                                             self.near,
                                                             self.far)

    def move(self, keys_pressed: list, time_delta: float):
        """default camera movement configuration."""

        amount_movement = self.speed_movement * time_delta
        amount_rotation = self.speed_rotation * time_delta

        for key in keys_pressed:
            # forward
            if key == glfw.KEY_W:
                self.translate(0, 0, amount_movement)
            # backward
            if key == glfw.KEY_S:
                self.translate(0, 0, -amount_movement)
            # left
            if key == glfw.KEY_A:
                self.translate(amount_movement, 0, 0)
            # right
            if key == glfw.KEY_D:
                self.translate(-amount_movement, 0, 0)
            # up
            if key == glfw.KEY_SPACE:
                self.translate(0, amount_movement, 0, local=False)
            # down
            if key == glfw.KEY_LEFT_SHIFT:
                self.translate(0, -amount_movement, 0, local=False)
            # turn right
            if key == glfw.KEY_RIGHT:
                self.yaw -= amount_rotation
            # turn left
            if key == glfw.KEY_LEFT:
                self.yaw += amount_rotation
            # turn up
            if key == glfw.KEY_UP:
                self.pitch += amount_rotation / 1.4
                if self.pitch > 89.0:
                    self.pitch = 89.0
            # turn down
            if key == glfw.KEY_DOWN:
                self.pitch -= amount_rotation / 1.4
                if self.pitch < -89.0:
                    self.pitch = -89.0

        front_x = math.cos(math.radians(self.yaw)) * math.cos(math.radians(self.pitch))
        front_y = math.sin(math.radians(self.pitch))
        front_z = math.sin(math.radians(self.yaw)) * math.cos(math.radians(self.pitch))
        # front = [front_x, front_y, front_z]
        front = [front_z, front_y, front_x] # ?
        # normalize
        front = [f / math.sqrt(sum([c**2 for c in front])) for f in front]
        self.transform[0:3, 2] = front

        # update right and up
        front = np.array(self.transform[0:3, 2])
        up = np.array([0, 1, 0])
        right = np.cross(up, front)
        right /= np.linalg.norm(right)
        true_up = np.cross(front, right)
        true_up /= np.linalg.norm(true_up)

        self.transform[0:3, 0] = right
        self.transform[0:3, 1] = true_up

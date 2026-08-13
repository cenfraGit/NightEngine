# NightCamera.py

from NightEngine.Objects.NightObject import NightObject
from NightEngine.NightMatrix import NightMatrix
import numpy as np
import math
import glfw

class NightCamera(NightObject):
    def __init__(self,
                 fov=85.0,
                 aspect_ratio=1.0,
                 near=0.1,
                 far=1000.0):

        # transform-only node: no mesh, no material, no physics body
        super().__init__()

        # -------------- properties -------------- #

        self.speed_movement = 10
        self.speed_rotation = 200

        self.fov = fov
        self.aspect_ratio = aspect_ratio
        self.near = near
        self.far = far

        self.pitch = 0
        self.yaw = -90

        self.matrix_projection = NightMatrix.get_perspective(fov, aspect_ratio, near, far)
        self.matrix_view = NightMatrix.get_identity()

    def look_at(self, position=None, target=[0, 0, 0]):
        """places (optional) and orients the camera to look at a world
        point. works for any direction, including straight down."""
        if position is not None:
            self.set_position(position)
        pos = np.array(self.get_position(), dtype=np.float64)
        forward = np.array(target, dtype=np.float64) - pos
        norm = np.linalg.norm(forward)
        if norm == 0:
            return
        forward /= norm
        world_up = np.array([0.0, 1.0, 0.0]) if abs(forward[1]) < 0.99 else np.array([0.0, 0.0, 1.0])
        # right = forward x up, matching NightMatrix.get_lookat, which builds
        # the matrix actually used to render. Using cross(up, forward) here
        # (as this did previously) stored a right vector pointing the
        # opposite way from on-screen right.
        right = np.cross(forward, world_up)
        right /= np.linalg.norm(right)
        true_up = np.cross(right, forward)
        true_up /= np.linalg.norm(true_up)
        self.transform[0:3, 0] = right
        self.transform[0:3, 1] = true_up
        self.transform[0:3, 2] = forward

    def update(self):

        # ------------- update view ------------- #

        # use the world matrix so the camera also works when attached
        # as a child of another object (e.g. a vehicle). the up vector
        # comes from the camera's own transform so straight-down views
        # (e.g. top-down inspection cameras) do not degenerate.
        world = self.get_world_matrix()
        position = np.array([world[0, 3], world[1, 3], world[2, 3]])
        forward = np.array(world[0:3, 2])
        up = np.array(world[0:3, 1])
        target = position + forward
        self.matrix_view = NightMatrix.get_lookat(position, target, up)

        # ---------- update perspective ---------- #

        self.matrix_projection = NightMatrix.get_perspective(self.fov,
                                                             self.aspect_ratio,
                                                             self.near,
                                                             self.far)

    def move(self, window, time_delta: float):
        """default camera movement configuration."""

        amount_movement = self.speed_movement * time_delta
        amount_rotation = self.speed_rotation * time_delta

        # exit
        if self.check_pressed(window, glfw.KEY_ESCAPE):
            glfw.set_window_should_close(window, True)
        # forward
        if self.check_pressed(window, glfw.KEY_W):
            self.translate(0, 0, amount_movement)
        # backward
        if self.check_pressed(window, glfw.KEY_S):
            self.translate(0, 0, -amount_movement)
        # left. translate along local X moves along the right vector, so
        # strafing left is negative. These signs were inverted while the
        # stored right vector pointed opposite to on-screen right.
        if self.check_pressed(window, glfw.KEY_A):
            self.translate(-amount_movement, 0, 0)
        # right
        if self.check_pressed(window, glfw.KEY_D):
            self.translate(amount_movement, 0, 0)
        # up
        if self.check_pressed(window, glfw.KEY_SPACE):
            self.translate(0, amount_movement, 0, local=False)
        # down
        if self.check_pressed(window, glfw.KEY_LEFT_SHIFT):
            self.translate(0, -amount_movement, 0, local=False)
        # turn right
        if self.check_pressed(window, glfw.KEY_RIGHT):
            self.yaw += amount_rotation
        # turn left
        if self.check_pressed(window, glfw.KEY_LEFT):
            self.yaw -= amount_rotation
        # turn up
        if self.check_pressed(window, glfw.KEY_UP):
            self.pitch += amount_rotation / 1.4
            if self.pitch > 89.0:
                self.pitch = 89.0
        # turn down
        if self.check_pressed(window, glfw.KEY_DOWN):
            self.pitch -= amount_rotation / 1.4
            if self.pitch < -89.0:
                self.pitch = -89.0

        front_x = math.cos(math.radians(self.yaw)) * math.cos(math.radians(self.pitch))
        front_y = math.sin(math.radians(self.pitch))
        front_z = math.sin(math.radians(self.yaw)) * math.cos(math.radians(self.pitch))
        front = np.array([front_x, front_y, front_z])
        front /= np.linalg.norm(front)
        
        self.transform[0:3, 2] = front

        # update right and up. right = forward x up to match
        # NightMatrix.get_lookat and therefore on-screen right; true_up is
        # algebraically the same either way (both reduce to
        # up - forward*(forward.up)), so the rendered view is unchanged.
        front = np.array(self.transform[0:3, 2])
        up = np.array([0, 1, 0])
        right = np.cross(front, up)
        right /= np.linalg.norm(right)
        true_up = np.cross(right, front)
        true_up /= np.linalg.norm(true_up)

        self.transform[0:3, 0] = right
        self.transform[0:3, 1] = true_up

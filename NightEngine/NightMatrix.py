# NightMatrix.py

import numpy as np
from math import sin, cos, tan, pi

class NightMatrix:

    @staticmethod
    def get_identity():
        return np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]], dtype=np.float32)

    @staticmethod
    def get_translation(x, y, z):
        return np.array([[1, 0, 0, x],
                         [0, 1, 0, y],
                         [0, 0, 1, z],
                         [0, 0, 0, 1]], dtype=np.float32)

    @staticmethod
    def get_rotation_x(angle):
        c = cos(angle)
        s = sin(angle)
        return np.array([[1, 0,  0, 0],
                         [0, c, -s, 0],
                         [0, s,  c, 0],
                         [0, 0,  0, 1]], dtype=np.float32)

    @staticmethod
    def get_rotation_y(angle):
        c = cos(angle)
        s = sin(angle)
        return np.array([[c,  0, s, 0],
                         [0,  1, 0, 0],
                         [-s, 0, c, 0],
                         [0,  0, 0, 1]], dtype=np.float32)

    @staticmethod
    def get_rotation_z(angle):
        c = cos(angle)
        s = sin(angle)
        return np.array([[c, -s, 0, 0],
                         [s,  c, 0, 0],
                         [0,  0, 1, 0],
                         [0,  0, 0, 1]], dtype=np.float32)

    @staticmethod
    def get_scale(s):
        return np.array([[s, 0, 0, 0],
                         [0, s, 0, 0],
                         [0, 0, s, 0],
                         [0, 0, 0, 1]], dtype=np.float32)

    @staticmethod
    def get_perspective(fov=60, aspect_ratio=1, near=0.1, far=1000):
        a = fov * pi/180.0
        d = 1.0 / tan(a/2)
        r = aspect_ratio
        b = (far + near) / (near - far)
        c = 2*far*near / (near - far)
        return np.array([[d/r, 0, 0,  0],
                         [0,   d, 0,  0],
                         [0,   0, b,  c],
                         [0,   0, -1, 0]], dtype=np.float32)

    @staticmethod
    def get_lookat(eye, target, up):
        eye = np.array(eye, dtype=np.float32)
        target = np.array(target, dtype=np.float32)
        up = np.array(up, dtype=np.float32)

        # ---------- calculate forward ---------- #
        
        forward = target - eye
        forward /= np.linalg.norm(forward)

        # ----------- calculate right ----------- #
        
        right = np.cross(forward, up)
        right /= np.linalg.norm(right)

        # ---------- calculate true up ---------- #
        
        true_up = np.cross(right, forward)

        # --------------- matrices --------------- #
        
        rotation = np.array([
            [right[0],    right[1],    right[2],    0],
            [true_up[0],  true_up[1],  true_up[2],  0],
            [-forward[0], -forward[1], -forward[2], 0],
            [0,           0,           0,           1]
        ], dtype=np.float32)

        translation = np.array([
            [1, 0, 0, -eye[0]],
            [0, 1, 0, -eye[1]],
            [0, 0, 1, -eye[2]],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        
        return rotation @ translation

# NightCamera.py

from NightEngine.NightObject import NightObject
from NightEngine.NightMatrix import NightMatrix

class NightCamera(NightObject):
    def __init__(self,
                 fov=70,
                 aspect_ratio=1,
                 near=0.1,
                 far=1000):
        
        super().__init__()

        self.matrix_projection = NightMatrix.get_perspective(fov, aspect_ratio, near, far)
        self.matrix_view = NightMatrix.get_identity()

    def update_view(self):
        position = self.get_position()
        forward = self.get_forward_vector()
        target = position + forward
        self.matrix_view = NightMatrix.get_lookat(position, target, [0, 1, 0])
        

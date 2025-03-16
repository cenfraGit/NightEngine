# NightObject.py

from NightEngine.NightMatrix import NightMatrix
from NightEngine.NightUtils import NightUtils
# from NightEngine.NightMesh import NightMesh
from OpenGL.GL import *
import numpy as np

class NightObject:
    def __init__(self, mesh=None, material=None):

        """initializes the object by locating the mesh attributes in
        the material program."""

        # ---------- initial transform ---------- #

        self.transform = NightMatrix.get_identity()

        # -------------- hierarchy -------------- #

        self.parent = None
        self.children = []

        # ------------ check if data ------------ #

        # if this object has no mesh or material (such as scene), exit
        # early.

        if not mesh or not material:
            return

        # -------------- properties -------------- #

        self.visible = True
        self.collisions = True
        
        self.mass = 1.0
        self.physics_id = None

        # -------------- appearance -------------- #

        self.material = material
        self.vertex_count = 0

        # ------------- vertex array ------------- #

        self.vao = NightUtils.create_vao()

        # ----------------- mesh ----------------- #

        # for each attrib in mesh, create vbo and set attrib pointer
        # on material program

        for variable_name, attribute_dict in mesh.attributes.items():
            vbo = NightUtils.create_vbo(attribute_dict["data"])
            NightUtils.set_attribute_pointer(material.program,
                                             vbo,
                                             variable_name,
                                             attribute_dict["data_type"])

    def move(self, keys_pressed: list, time_delta: float):
        # override
        pass

    def add(self, child):
        """adds child to object hierarchy."""
        self.children.append(child)
        child.parent = self

    def remove(self, child):
        """removes child to object hierarchy."""
        self.children.remove(child)
        child.parent = None

    def get_descendants(self, include_self=True):
        """returns a list of all descendants."""
        descendants = []
        nodes_to_process = [self] if include_self else self.children.copy()
        while nodes_to_process:
            node = nodes_to_process.pop()
            descendants.append(node)
            nodes_to_process.extend(reversed(node.children))
        return descendants

    def get_world_matrix(self):
        if self.parent == None:
            return self.transform
        else:
            return self.parent.get_world_matrix() @ self.transform

    def get_position(self, world=False):
        """returns the object's position (local or world)."""
        # ------------ world position ------------ #
        if world:
            world_transform = self.get_world_matrix()
            return [world_transform.item((0, 3)),
                    world_transform.item((1, 3)),
                    world_transform.item((2, 3))]
        # ------ local (relative to parent) ------ #
        else:
            return [self.transform.item((0, 3)),
                    self.transform.item((1, 3)),
                    self.transform.item((2, 3))]

    def get_forward_vector(self):
        """returns local z axis."""
        return self.transform[0:3, 2]

    def get_up_vector(self):
        """returns local y axis."""
        return self.transform[0:3, 1]

    def get_right_vector(self):
        """returns local x axis."""
        return self.transform[0:3, 0]

    def set_position(self, position:list):
        self.transform[0, 3] = position[0]
        self.transform[1, 3] = position[1]
        self.transform[2, 3] = position[2]

    def set_rotation(self, rotation_matrix:np.ndarray):
        self.transform[0, 0] = rotation_matrix[0, 0]
        self.transform[0, 1] = rotation_matrix[0, 1]
        self.transform[0, 2] = rotation_matrix[0, 2]

        self.transform[1, 0] = rotation_matrix[1, 0]
        self.transform[1, 1] = rotation_matrix[1, 1]
        self.transform[1, 2] = rotation_matrix[1, 2]

        self.transform[2, 0] = rotation_matrix[2, 0]
        self.transform[2, 1] = rotation_matrix[2, 1]
        self.transform[2, 2] = rotation_matrix[2, 2]

    def translate(self, x:float, y:float, z:float, local=True):
        m = NightMatrix.get_translation(x, y, z)
        self._apply_matrix(m, local)

    def scale(self, s:float, local=True):
        m = NightMatrix.get_scale(s)
        self._apply_matrix(m, local)

    def rotate_x(self, angle:float, local=True):
        m = NightMatrix.get_rotation_x(angle)
        self._apply_matrix(m, local)

    def rotate_y(self, angle:float, local=True):
        m = NightMatrix.get_rotation_y(angle)
        self._apply_matrix(m, local)

    def rotate_z(self, angle:float, local=True):
        m = NightMatrix.get_rotation_z(angle)
        self._apply_matrix(m, local)

    def _apply_matrix(self, matrix:np.ndarray, local=True):
        if local:
            self.transform = self.transform @ matrix
        else:
            self.transform = matrix @ self.transform

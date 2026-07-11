# NightObject.py

from NightEngine.NightMatrix import NightMatrix
from NightEngine.NightUtils import NightUtils
from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Materials.NightMaterialLight import NightMaterialLight
from scipy.spatial.transform import Rotation as R
from OpenGL.GL import *
import pybullet as p
import numpy as np
import glfw

class NightObject:
    def __init__(self, mesh=None, material=None, mass=0.0):

        """initializes the object by locating the mesh attributes in
        the material program."""

        # ---------- initial transform ---------- #

        self.transform = NightMatrix.get_identity()

        # -------------- hierarchy -------------- #

        self.parent = None
        self.children = []
        self._descendants_cache = {}

        # -------------- properties -------------- #

        self.visible = True

        self.mass = mass
        self.physics_id = None

        # physics states captured around each simulation step, used
        # for render interpolation (managed by NightBase)
        self._physics_state_prev = None
        self._physics_state_curr = None

        self.mesh = mesh
        self.material = material

        # ------------ check if data ------------ #

        # if this object has no mesh or material (such as scene), exit
        # early.

        if not mesh or not material:
            return

        # ------------- vertex array ------------- #

        self.vao = NightUtils.create_vao()

        # ----------------- mesh ----------------- #

        # for each attrib in mesh, create vbo and set attrib pointer
        # on material program

        for variable_name, attribute_dict in mesh.attributes.items():
            # --------- material point light --------- #
            if isinstance(self.material, NightMaterialLight) and variable_name not in ["vertex_position"]:
                continue
            # ----------- material default ----------- #
            if isinstance(self.material, NightMaterialDefault) and variable_name not in ["vertex_position", "vertex_color", "vertex_normal"]:
                continue
            # ----------- material texture ----------- #
            if isinstance(self.material, NightMaterialDefault) and variable_name not in ["vertex_position", "vertex_color", "vertex_normal", "vertex_uv"]:
                continue
            vbo = NightUtils.create_vbo(attribute_dict["data"])
            NightUtils.set_attribute_pointer(material.program,
                                             vbo,
                                             variable_name,
                                             attribute_dict["data_type"])

        self.linkMasses = []
        self.linkCollisionShapeIndices = []
        self.linkVisualShapeIndices = []
        self.linkPositions = []
        self.linkOrientations = []
        self.linkInertialFramePositions = []
        self.linkInertialFrameOrientations = []
        self.linkParentIndices = []
        self.linkJointTypes = []
        self.linkJointAxis = []
        self.linkReferences = []
        
    def init_multibody(self):
        if self.mesh and self.mesh.collision_shape != None:
            self.physics_id = p.createMultiBody(
                baseMass=self.mass,
                baseCollisionShapeIndex=self.mesh.collision_shape,
                basePosition=self.get_position(),
                baseOrientation=self.get_orientation(),
                linkMasses=self.linkMasses,
                linkCollisionShapeIndices=self.linkCollisionShapeIndices,
                linkVisualShapeIndices=self.linkVisualShapeIndices,
                linkPositions=self.linkPositions,
                linkOrientations=self.linkOrientations,
                linkInertialFramePositions=self.linkInertialFramePositions,
                linkInertialFrameOrientations=self.linkInertialFrameOrientations,
                linkParentIndices=self.linkParentIndices,
                linkJointTypes=self.linkJointTypes,
                linkJointAxis=self.linkJointAxis,
                useMaximalCoordinates=False)

    def add_link(self, obj, joint_type, inertial_frame_position=[0, 0, 0], inertial_frame_orientation=[0, 0, 0, 1], axis=[1, 0, 0]):
        link_index_new = len(self.linkParentIndices)
        self.linkMasses.append(obj.mass)
        self.linkCollisionShapeIndices.append(obj.mesh.collision_shape)
        self.linkVisualShapeIndices.append(-1)
        self.linkPositions.append(obj.get_position())
        self.linkOrientations.append(obj.get_orientation().tolist())
        self.linkInertialFramePositions.append(inertial_frame_position)
        self.linkInertialFrameOrientations.append(inertial_frame_orientation)
        self.linkParentIndices.append(0)
        self.linkJointTypes.append(joint_type)
        self.linkJointAxis.append(axis)
        self.linkReferences.append(obj)
        return link_index_new
        
    def check_pressed(self, window, glfw_key):
        return glfw.get_key(window, glfw_key) == glfw.PRESS

    def move(self, window, time_delta: float):
        # override
        pass

    def add(self, child):
        """adds child to object hierarchy."""
        self.children.append(child)
        child.parent = self
        self._invalidate_descendants_cache()

    def remove(self, child):
        """removes child to object hierarchy."""
        self.children.remove(child)
        child.parent = None
        self._invalidate_descendants_cache()

    def _invalidate_descendants_cache(self):
        node = self
        while node is not None:
            node._descendants_cache = {}
            node = node.parent

    def get_descendants(self, include_self=True):
        """returns a list of all descendants."""
        cached = self._descendants_cache.get(include_self)
        if cached is not None:
            return cached
        descendants = []
        nodes_to_process = [self] if include_self else self.children.copy()
        while nodes_to_process:
            node = nodes_to_process.pop()
            descendants.append(node)
            nodes_to_process.extend(reversed(node.children))
        self._descendants_cache[include_self] = descendants
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

    def get_rotation(self):
        return self.transform[0:3, 0:3]

    def get_orientation(self):
        """returns quaternion"""
        orn = R.from_matrix(self.get_rotation()).as_quat()
        return orn

    def get_yaw_pitch_roll(self):
        rotation_matrix = self.get_rotation()
        yaw = np.arctan2(rotation_matrix[2, 0], rotation_matrix[0, 0])
        pitch = -np.arcsin(np.clip(rotation_matrix[1, 0], -1.0, 1.0))
        roll = np.arctan2(rotation_matrix[1, 2], rotation_matrix[1, 1])
        return yaw, pitch, roll

    def get_forward_vector(self):
        """returns local z axis."""
        return self.transform[0:3, 2]

    def get_up_vector(self):
        """returns local y axis."""
        return self.transform[0:3, 1]

    def get_right_vector(self):
        """returns local x axis."""
        return self.transform[0:3, 0]

    def set_position(self, position:list, reset_base=True):
        self.transform[0, 3] = position[0]
        self.transform[1, 3] = position[1]
        self.transform[2, 3] = position[2]
        if reset_base:
            self._update_physics_pos_orn(force=True)

    def set_rotation(self, rotation_matrix:np.ndarray, reset_base=True):
        self.transform[0, 0] = rotation_matrix[0, 0]
        self.transform[0, 1] = rotation_matrix[0, 1]
        self.transform[0, 2] = rotation_matrix[0, 2]

        self.transform[1, 0] = rotation_matrix[1, 0]
        self.transform[1, 1] = rotation_matrix[1, 1]
        self.transform[1, 2] = rotation_matrix[1, 2]

        self.transform[2, 0] = rotation_matrix[2, 0]
        self.transform[2, 1] = rotation_matrix[2, 1]
        self.transform[2, 2] = rotation_matrix[2, 2]
        if reset_base:
            self._update_physics_pos_orn(force=True)

    def translate(self, x:float, y:float, z:float, local=True):
        m = NightMatrix.get_translation(x, y, z)
        self._apply_matrix(m, local)
        self._update_physics_pos_orn()

    def scale(self, s:float, local=True):
        m = NightMatrix.get_scale(s)
        self._apply_matrix(m, local)

    def rotate_x(self, angle:float, local=True):
        m = NightMatrix.get_rotation_x(angle)
        self._apply_matrix(m, local)
        self._update_physics_pos_orn()

    def rotate_y(self, angle:float, local=True):
        m = NightMatrix.get_rotation_y(angle)
        self._apply_matrix(m, local)
        self._update_physics_pos_orn()

    def rotate_z(self, angle:float, local=True):
        m = NightMatrix.get_rotation_z(angle)
        self._apply_matrix(m, local)
        self._update_physics_pos_orn()

    def _apply_matrix(self, matrix:np.ndarray, local=True):
        if local:
            self.transform = self.transform @ matrix
        else:
            self.transform = matrix @ self.transform

    def _update_physics_pos_orn(self, force=False):
        """syncs the physics body to the render transform.

        dynamic bodies (mass > 0) are owned by the physics engine:
        teleporting them every frame fights the solver (broken
        collisions, inconsistent velocities). they are only synced on
        explicit placement (set_position/set_rotation -> force=True).
        to move a dynamic body, use set_velocity/apply_force instead."""
        if self.physics_id is None:
            return
        if not force and self.mass > 0:
            return
        pos = self.get_position()
        orn = R.from_matrix(self.get_rotation()).as_quat()
        p.resetBasePositionAndOrientation(self.physics_id, pos, orn)
        # invalidate captured states so interpolation does not blend
        # across a teleport
        self._physics_state_prev = None
        self._physics_state_curr = None

    # ------------------------------------------------------------
    # dynamic body control (velocities, forces)
    # ------------------------------------------------------------

    def get_velocity(self):
        """returns (linear, angular) world-frame velocity."""
        if self.physics_id is None:
            return ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        return p.getBaseVelocity(self.physics_id)

    def set_velocity(self, linear=None, angular=None):
        """sets world-frame base velocity. call from physics_update."""
        if self.physics_id is None:
            return
        kwargs = {}
        if linear is not None:
            kwargs["linearVelocity"] = list(linear)
        if angular is not None:
            kwargs["angularVelocity"] = list(angular)
        p.resetBaseVelocity(self.physics_id, **kwargs)

    def apply_force(self, force, position=[0, 0, 0], link_index=-1, local=True):
        """applies a force for the next physics step only: call it
        from physics_update so it acts on every step."""
        if self.physics_id is None:
            return
        p.applyExternalForce(self.physics_id, link_index, list(force), list(position),
                             p.LINK_FRAME if local else p.WORLD_FRAME)

    def apply_torque(self, torque, link_index=-1, local=True):
        """applies a torque for the next physics step only: call it
        from physics_update so it acts on every step."""
        if self.physics_id is None:
            return
        p.applyExternalTorque(self.physics_id, link_index, list(torque),
                              p.LINK_FRAME if local else p.WORLD_FRAME)

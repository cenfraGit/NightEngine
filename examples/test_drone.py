# test_drone.py

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.Objects.NightObject import NightObject
from NightEngine.Objects.NightLink import NightLink
from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Materials.NightMaterialLight import NightMaterialLight
from NightEngine.Meshes.MeshBox import MeshBox
from NightEngine.Meshes.MeshSphere import MeshSphere
from NightEngine.Objects.ObjectGrid import ObjectGrid
from NightEngine.Objects.ObjectAxes import ObjectAxes
import pybullet as p
import numpy as np
import glfw

class ControllerPID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_previous = 0
        self.integral = 0

    def compute(self, target, process_variable, dt):
        error = target - process_variable
        self.integral += error * dt
        P = self.kp * error
        I = self.ki * self.integral
        D = self.kd * (error - self.error_previous) / dt
        self.error_previous = error
        return P + I + D

class Quadcopter(NightObject):
    def __init__(self, scene):

        self.base_force = 5

        self.target_altitude = 5
        self.target_pitch = 0
        
        self.rot1_force = 0
        self.rot2_force = 0
        self.rot3_force = 0
        self.rot4_force = 0

        # controllers
        self.pid_altitude = ControllerPID(kp=1.0, ki=1.0, kd=1.0)
        self.pid_pitch = ControllerPID(kp=0.3, ki=0.5, kd=1.0)
        
        # create drone base
        mesh = MeshBox(3, 1, 5)
        material = NightMaterialDefault()
        super().__init__(mesh, material, mass=0.06)

        # create rotors
        mesh_rotors = MeshSphere(0.6)
        
        self.rot1 = NightLink(mesh_rotors, material, mass=0.05)
        self.rot1.set_position([-1.7, 1, 2.7])
        self.rot2 = NightLink(mesh_rotors, material, mass=0.05)
        self.rot2.set_position([-1.7, 1, -2.7])
        self.rot3 = NightLink(mesh_rotors, material, mass=0.05)
        self.rot3.set_position([1.7, 1, -2.7])
        self.rot4 = NightLink(mesh_rotors, material, mass=0.05)
        self.rot4.set_position([1.7, 1, 2.7])
        
        self.rot1_id = self.add_link(self.rot1, p.JOINT_FIXED)
        self.rot2_id = self.add_link(self.rot2, p.JOINT_FIXED)
        self.rot3_id = self.add_link(self.rot3, p.JOINT_FIXED)
        self.rot4_id = self.add_link(self.rot4, p.JOINT_FIXED)

        scene.add(self.rot1)
        scene.add(self.rot2)
        scene.add(self.rot3)
        scene.add(self.rot4)

    def _update_rotor_forces(self):

        # convert forces from local to world
        rotation_matrix = np.array(p.getMatrixFromQuaternion(self.get_orientation())).reshape(3, 3)
        force1 = rotation_matrix @ np.array([0.0, self.rot1_force, 0.0])
        force2 = rotation_matrix @ np.array([0.0, self.rot2_force, 0.0])
        force3 = rotation_matrix @ np.array([0.0, self.rot3_force, 0.0])
        force4 = rotation_matrix @ np.array([0.0, self.rot4_force, 0.0])

        p.applyExternalForce(self.physics_id,
                             linkIndex=self.rot1_id,
                             forceObj=force1,
                             posObj=self.rot1.get_position(),
                             flags=p.WORLD_FRAME)
        p.applyExternalForce(self.physics_id,
                             linkIndex=self.rot2_id,
                             forceObj=force2,
                             posObj=self.rot2.get_position(),
                             flags=p.WORLD_FRAME)
        p.applyExternalForce(self.physics_id,
                             linkIndex=self.rot3_id,
                             forceObj=force3,
                             posObj=self.rot3.get_position(),
                             flags=p.WORLD_FRAME)
        p.applyExternalForce(self.physics_id,
                             linkIndex=self.rot4_id,
                             forceObj=force4,
                             posObj=self.rot4.get_position(),
                             flags=p.WORLD_FRAME)

    def _get_altitude(self):
        return self.get_position()[1]

    def _get_pitch(self):
        roll, pitch, yaw = p.getEulerFromQuaternion(self.get_orientation())
        return -roll
        
    def move(self, window, time_delta: float):

        # ------------------------------------------------------------
        # check key inputs
        # ------------------------------------------------------------

        # altitude
        if self.check_pressed(window, glfw.KEY_O):
            self.target_altitude += 0.2
        if self.check_pressed(window, glfw.KEY_U):
            self.target_altitude -= 0.2
        # movement
        if self.check_pressed(window, glfw.KEY_I):
            self.target_pitch = 0.5
        elif self.check_pressed(window, glfw.KEY_K):
            self.target_pitch = -0.5
        else:
            self.target_pitch = 0.0

        # ------------------------------------------------------------
        # control
        # ------------------------------------------------------------

        correction_altitude = self.pid_altitude.compute(self.target_altitude, self._get_altitude(), 1./240.)
        correction_pitch = self.pid_pitch.compute(self.target_pitch, self._get_pitch(), 1./240.)

        self.rot1_force = self.base_force + correction_altitude
        self.rot2_force = self.base_force + correction_altitude
        self.rot3_force = self.base_force + correction_altitude
        self.rot4_force = self.base_force + correction_altitude

        self.rot1_force += correction_pitch
        self.rot2_force -= correction_pitch
        self.rot3_force -= correction_pitch
        self.rot4_force += correction_pitch

        print(self.target_pitch, round(self._get_pitch(), 2), round(correction_pitch, 2))

        # --------- update rotor forces --------- #

        self._update_rotor_forces()

class Example(NightBase):
    def setup(self):

        self.scene = self.create_scene()
        self.camera = NightCamera()
        self.camera.set_position([0, 0, 20])
        self.set_gravity(y=-10)

        self.grid = ObjectGrid(width=100, divisions=20, color=[0.5, 0.5, 0.5])
        self.scene.add(self.grid)

        self.drone = Quadcopter(self.scene)
        self.drone.set_position([0, 5, 0])
        self.scene.add(self.drone)

    def update(self):
        self.drone.move(self.window, self.time_delta)
        self.draw_scene(self.camera)

if __name__ == "__main__":
    engine = Example()
    engine.run()

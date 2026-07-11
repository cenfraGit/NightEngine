# drone.py

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.Objects.NightObject import NightObject
from NightEngine.Objects.NightLink import NightLink
from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Meshes.MeshBox import MeshBox
from NightEngine.Meshes.MeshSphere import MeshSphere
import pybullet as p
import numpy as np
import glfw
import sys
import math
import signal

from multiprocessing import Process, Queue
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

data_queue = Queue()

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
        result = P + I + D
        limit = 10
        if result > limit:
            return limit
        elif result < -limit:
            return -limit
        return result

class Quadcopter(NightObject):
    def __init__(self, scene):

        # forces are now applied on every physics substep (previously
        # they only acted on 1 of ~4 substeps per frame), so base
        # thrust is set to the actual hover thrust per rotor:
        # total mass (0.06 + 4 * 0.05) * gravity 40 / 4 rotors
        self.base_force = 2.6

        self.target_altitude = 30
        self.target_pitch = 0
        self.target_roll = 0
        self.target_yaw = 0
        self.target_velocity_forward = 0
        self.target_velocity_right = 0
        
        self.rot1_force = 0
        self.rot2_force = 0
        self.rot3_force = 0
        self.rot4_force = 0
        self.rotational_force = 0

        # ------------------------------------------------------------
        # controllers
        # ------------------------------------------------------------

        # gains tuned against the fixed per-substep simulation with a
        # headless search (ITAE cost on step responses, then a combined
        # climb/forward/lateral/yaw validation flight)

        # --------------- altitude --------------- #

        self.pid_altitude = ControllerPID(kp=12.0, ki=0.0, kd=1.5)

        # ---------------- pitch ---------------- #

        self.pid_pitch = ControllerPID(kp=6.0, ki=0.0, kd=1.5)

        # ----------------- roll ----------------- #

        self.pid_roll = ControllerPID(kp=6.0, ki=0.0, kd=0.8)

        # ----------------- yaw ----------------- #

        self.pid_yaw = ControllerPID(kp=240.0, ki=3.0, kd=0.0)

        # ------------- vel forward ------------- #

        self.pid_velocity_forward = ControllerPID(kp=0.08, ki=0.01, kd=0.0)

        # -------------- vel right -------------- #

        self.pid_velocity_right = ControllerPID(kp=0.08, ki=0.01, kd=0.0)

        # -------------- vel right -------------- #
        
        # create drone base
        mesh = MeshBox(3, 1, 5, color=[1.0, 1.0, 1.0])
        material = NightMaterialDefault()
        super().__init__(mesh, material, mass=0.06)

        # create rotors
        mesh_rotors = MeshSphere(0.6, 8, color=[0.2, 0.2, 0.2])

        self.rot1_pos_local = [-1.7, 1, 2.7]
        self.rot2_pos_local = [-1.7, 1, -2.7]
        self.rot3_pos_local = [1.7, 1, -2.7]
        self.rot4_pos_local = [1.7, 1, 2.7]
        
        self.rot1 = NightLink(mesh_rotors, material, mass=0.05)
        self.rot1.set_position(self.rot1_pos_local)
        self.rot2 = NightLink(mesh_rotors, material, mass=0.05)
        self.rot2.set_position(self.rot2_pos_local)
        self.rot3 = NightLink(mesh_rotors, material, mass=0.05)
        self.rot3.set_position(self.rot3_pos_local)
        self.rot4 = NightLink(mesh_rotors, material, mass=0.05)
        self.rot4.set_position(self.rot4_pos_local)
        
        self.rot1_id = self.add_link(self.rot1, p.JOINT_FIXED)
        self.rot2_id = self.add_link(self.rot2, p.JOINT_FIXED)
        self.rot3_id = self.add_link(self.rot3, p.JOINT_FIXED)
        self.rot4_id = self.add_link(self.rot4, p.JOINT_FIXED)

        # the camera is a transform-only child of the drone (not a
        # physics link): it follows the base through the hierarchy
        self.camera = NightCamera()
        self.camera.set_position([0, 1, 2.7])
        self.camera.rotate_x(1)
        self.add(self.camera)

        scene.add(self.rot1)
        scene.add(self.rot2)
        scene.add(self.rot3)
        scene.add(self.rot4)

    def _update_rotor_forces(self):

        force1 = np.array([0.0, self.rot1_force, 0.0])
        force2 = np.array([0.0, self.rot2_force, 0.0])
        force3 = np.array([0.0, self.rot3_force, 0.0])
        force4 = np.array([0.0, self.rot4_force, 0.0])

        # posObj is relative to the LINK frame, whose origin already
        # sits at the rotor: [0,0,0] applies thrust at the rotor
        # itself (the old rotor offsets doubled the torque arm)
        p.applyExternalForce(self.physics_id,
                             linkIndex=self.rot1_id,
                             forceObj=force1,
                             posObj=[0, 0, 0],
                             flags=p.LINK_FRAME)
        p.applyExternalForce(self.physics_id,
                             linkIndex=self.rot2_id,
                             forceObj=force2,
                             posObj=[0, 0, 0],
                             flags=p.LINK_FRAME)
        p.applyExternalForce(self.physics_id,
                             linkIndex=self.rot3_id,
                             forceObj=force3,
                             posObj=[0, 0, 0],
                             flags=p.LINK_FRAME)
        p.applyExternalForce(self.physics_id,
                             linkIndex=self.rot4_id,
                             forceObj=force4,
                             posObj=[0, 0, 0],
                             flags=p.LINK_FRAME)
        
        p.applyExternalTorque(self.physics_id,
                              linkIndex=-1,
                              torqueObj=[0.0, self.rotational_force, 0.0],
                              flags=p.WORLD_FRAME)

    def _get_altitude(self):
        return self.get_position()[1]

    def _get_pitch(self):
        yaw, pitch, roll = self.get_yaw_pitch_roll()
        return roll # ?

    def _get_roll(self):
        yaw, pitch, roll = self.get_yaw_pitch_roll()
        return pitch # ?

    def _get_yaw(self):
        yaw, pitch, roll = self.get_yaw_pitch_roll()
        return -yaw # ?

    def _get_yaw_rate(self):
        _, angular_velocity = p.getBaseVelocity(self.physics_id)
        return angular_velocity[1] # ?
        
    def move(self, window, time_step: float, time_total):
        """called once per physics step from physics_update: forces
        applied with applyExternalForce only last one step, and the
        PIDs integrate with the real step size."""

        # ------------------------------------------------------------
        # check key inputs
        # ------------------------------------------------------------

        # altitude
        if self.check_pressed(window, glfw.KEY_Y):
            self.target_altitude += 0.2
        if self.check_pressed(window, glfw.KEY_H):
            self.target_altitude -= 0.2
        # forward movement
        if self.check_pressed(window, glfw.KEY_I):
            self.target_velocity_forward = 4
        elif self.check_pressed(window, glfw.KEY_K):
            self.target_velocity_forward = -4
        else:
            self.target_velocity_forward = 0
        # lateral movement
        if self.check_pressed(window, glfw.KEY_L):
            self.target_velocity_right = 4
        elif self.check_pressed(window, glfw.KEY_J):
            self.target_velocity_right = -4
        else:
            self.target_velocity_right = 0
        # yaw
        if self.check_pressed(window, glfw.KEY_O):
            self.target_yaw_rate = -0.8
        elif self.check_pressed(window, glfw.KEY_U):
            self.target_yaw_rate = 0.8
        else:
            self.target_yaw_rate = 0.0

        # ------------------------------------------------------------
        # control
        # ------------------------------------------------------------

        # first apply corrections to pitch and roll values

        linear_velocity, _ = p.getBaseVelocity(self.physics_id)
        # linear velocity is global, need to transform to local
        _, orientation_quaternion = p.getBasePositionAndOrientation(self.physics_id)
        rotation_matrix = np.array(p.getMatrixFromQuaternion(orientation_quaternion)).reshape(3, 3)
        local_velocity = np.dot(rotation_matrix.T, linear_velocity)
        velocity_forward = -local_velocity[2]
        velocity_right = -local_velocity[0]
        
        # note: +pitch target accelerates forward, so the error is
        # simply target - current (the old -target here made the loop
        # positive feedback)
        velocity_forward_correction = self.pid_velocity_forward.compute(self.target_velocity_forward, velocity_forward, time_step)
        velocity_right_correction = self.pid_velocity_right.compute(self.target_velocity_right, velocity_right, time_step)

        self.target_pitch = velocity_forward_correction
        self.target_roll = -velocity_right_correction

        # then, based on current values, apply corrections to motors

        correction_altitude = self.pid_altitude.compute(self.target_altitude, self._get_altitude(), time_step)
        correction_pitch = self.pid_pitch.compute(self.target_pitch, self._get_pitch(), time_step)
        correction_roll = self.pid_roll.compute(self.target_roll, self._get_roll(), time_step)
        correction_yaw = self.pid_yaw.compute(self.target_yaw_rate, self._get_yaw_rate(), time_step)

        self.rot1_force = self.base_force + correction_altitude
        self.rot2_force = self.base_force + correction_altitude
        self.rot3_force = self.base_force + correction_altitude
        self.rot4_force = self.base_force + correction_altitude

        self.rot1_force += correction_pitch
        self.rot2_force -= correction_pitch
        self.rot3_force -= correction_pitch
        self.rot4_force += correction_pitch

        self.rot1_force += correction_roll
        self.rot2_force += correction_roll
        self.rot3_force -= correction_roll
        self.rot4_force -= correction_roll

        self.rotational_force = correction_yaw 

        # sample the plot at ~60hz instead of every 240hz substep
        self._plot_counter = getattr(self, "_plot_counter", 0) + 1
        if self._plot_counter % 4 == 0:
            data_queue.put([time_total,
                            self.target_altitude, self._get_altitude(),
                            self.target_pitch, self._get_pitch(),
                            self.target_roll, self._get_roll(),
                            self.target_yaw_rate, self._get_yaw_rate()])

        # --------- update rotor forces --------- #

        self._update_rotor_forces()

class Example(NightBase):
    def setup(self):

        self.scene = self.create_scene()
        # self.camera = NightCamera()
        # self.camera.set_position([0, 20, 20])
        self.set_gravity(y=-40)

        self.light_directional["direction"] = [-0.5, -1, 0]

        # thin-but-solid ground box (a zero-thickness collider is
        # degenerate and lets fast bodies tunnel through)
        self.plane = NightObject(MeshBox(100, 2, 100, color=[0.5, 0.2, 0.1]), NightMaterialDefault())
        self.plane.set_position([0, -1, 0])
        self.scene.add(self.plane)

        self.drone = Quadcopter(self.scene)
        self.drone.set_position([0, 30, 0])
        self.scene.add(self.drone)

        # ------- lemniscate of bernoulli ------- #

        path_color = [0.5, 0.5, 0.0]

        box_width = 3
        box_height = 0.4
        box_depth = 5
        num_boxes = 40
        radius = 30
        for i in range(num_boxes):
            angle = i / num_boxes * 2 * math.pi
            # parametric equations
            x = radius * math.cos(angle) / (1 + math.sin(angle) ** 2)
            y = box_height / 2
            z = radius * math.sin(angle) * math.cos(angle) / (1 + math.sin(angle) ** 2)

            path = NightObject(MeshBox(box_width, box_height, box_depth, color=path_color), NightMaterialDefault())
            path.set_position([x, y, z])

            # tangent
            dz = radius * (-math.sin(angle) * (1 + math.sin(angle) ** 2) - math.cos(angle) * 2 * math.sin(angle) * math.cos(angle)) / ((1 + math.sin(angle) ** 2) ** 2)
            dx = radius * (math.cos(2 * angle) * (1 + math.sin(angle) ** 2) - math.sin(angle) * math.cos(angle) * 2 * math.sin(angle) * math.cos(angle)) / ((1 + math.sin(angle) ** 2) ** 2)

            if dx == 0 and dz == 0:
                rotation_matrix = np.eye(3)
            else:
                tangent_angle = math.atan2(dx, dz)
                rotation_matrix = np.array([
                    [math.cos(-tangent_angle - math.pi / 2), 0, math.sin(-tangent_angle - math.pi / 2)],
                    [0, 1, 0],
                    [-math.sin(-tangent_angle - math.pi / 2), 0, math.cos(-tangent_angle - math.pi / 2)]
                ])

            path.set_rotation(rotation_matrix)
            self.scene.add(path)

        
    def physics_update(self, time_step):
        self.drone.move(self.window, time_step, self.time)

    def update(self):
        # self.draw_scene(self.camera)
        self.draw_scene(self.drone.camera)

# module-level so windows multiprocessing (spawn) can import it in the
# child process
def start_plotting(queue):
    app = pg.mkQApp("dataplot")

    win = pg.GraphicsLayoutWidget(show=True, title="drone data")
    win.resize(900, 700)
    win.setWindowTitle("drone data")
    pg.setConfigOptions(antialias=True)

    plot_altitude = win.addPlot(title="Altitude")
    curve_altitude_target = plot_altitude.plot(pen='r')
    curve_altitude_current = plot_altitude.plot(pen='y')
    win.nextRow()
    plot_pitch = win.addPlot(title="Pitch")
    curve_pitch_target = plot_pitch.plot(pen='r')
    curve_pitch_current = plot_pitch.plot(pen='y')
    win.nextRow()
    plot_roll = win.addPlot(title="Roll")
    curve_roll_target = plot_roll.plot(pen='r')
    curve_roll_current = plot_roll.plot(pen='y')
    win.nextRow()
    plot_yaw = win.addPlot(title="Yaw Rate")
    curve_yaw_target = plot_yaw.plot(pen='r')
    curve_yaw_current = plot_yaw.plot(pen='y')

    x_data = []
    y_altitude_target = []
    y_altitude_current = []
    y_pitch_target = []
    y_pitch_current = []
    y_roll_target = []
    y_roll_current = []
    y_yaw_target = []
    y_yaw_current = []

    def update():
        while not queue.empty():
            data = queue.get()
            x_data.append(data[0])
            y_altitude_target.append(data[1])
            y_altitude_current.append(data[2])
            y_pitch_target.append(data[3])
            y_pitch_current.append(data[4])
            y_roll_target.append(data[5])
            y_roll_current.append(data[6])
            y_yaw_target.append(data[7])
            y_yaw_current.append(data[8])

            if len(x_data) > 500:
                x_data.pop(0)
                y_altitude_target.pop(0)
                y_altitude_current.pop(0)
                y_pitch_target.pop(0)
                y_pitch_current.pop(0)
                y_roll_target.pop(0)
                y_roll_current.pop(0)
                y_yaw_target.pop(0)
                y_yaw_current.pop(0)

            curve_altitude_target.setData(x_data, y_altitude_target)
            curve_altitude_current.setData(x_data, y_altitude_current)
            curve_pitch_target.setData(x_data, y_pitch_target)
            curve_pitch_current.setData(x_data, y_pitch_current)
            curve_roll_target.setData(x_data, y_roll_target)
            curve_roll_current.setData(x_data, y_roll_current)
            curve_yaw_target.setData(x_data, y_yaw_target)
            curve_yaw_current.setData(x_data, y_yaw_current)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)
    app.exec_()

if __name__ == "__main__":

    def signal_handler(sig, frame):
        plotting_process.terminate()
        plotting_process.join()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    plotting_process = Process(target=start_plotting, args=(data_queue,))
    plotting_process.start()
    
    engine = Example()
    engine.run()

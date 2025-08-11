import argparse
import atexit
import os
import sys
import select
import time
import threading
import numpy as np
from scipy.spatial.transform import Rotation

import mevita_utils
import parameters as P
from tmotor_lib import CanMotorController

import rospy
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Imu, Joy, JointState
from nav_msgs.msg import Odometry
from mevita.msg import MevitaLog
rospy.init_node("mevita")


# TODO add terminal display of thermometer, etc.

np.set_printoptions(precision=3, suppress=True)

class RobotState:
    def __init__(self):
        self.angle = [0.0] * P.N_JOINTS
        self.velocity = [0.0] * P.N_JOINTS
        self.torque = [0.0] * P.N_JOINTS
        self.temperature = [0.0] * P.N_JOINTS
        self.lock = threading.Lock()

class PeripheralState:
    def __init__(self):
        self.imu_last_time = None
        self.body_vel = [0.0] * 3
        self.body_quat = [0.0] * 4
        self.body_gyro = [0.0] * 3
        self.body_acc = [0.0] * 3
        self.ps4joy_enable = False
        self.ps4joy_axes = [0.0] * 14
        self.ps4joy_buttons = [0.0] * 14
        self.spacenav_enable = False
        self.spacenav = [0.0] * 8
        self.virtual_enable = False
        self.virtual = [0.0] * 4
        self.lock = threading.Lock()

class RobotCommand:
    def __init__(self):
        self.angle = [0.0] * P.N_JOINTS
        self.velocity = [0.0] * P.N_JOINTS
        self.torque = [0.0] * P.N_JOINTS
        self.kp = P.KP_GAIN[:]
        self.kd = P.KD_GAIN[:]
        assert len(self.kp) == P.N_JOINTS
        assert len(self.kd) == P.N_JOINTS
        self.command = "STANDBY"
        self.initial_angle = [0.0] * P.N_JOINTS
        self.final_angle = [0.0] * P.N_JOINTS
        self.interpolating_time = 0.0
        self.remaining_time = 0.0
        self.initialized = False
        self.lock = threading.Lock()

class Mevita:
    def __init__(self):
        self.xml_fullpath = os.path.abspath('models/scene.xml')
        self.urdf_fullpath = os.path.abspath('models/mevita_dae.urdf')
        self.joint_params = mevita_utils.get_urdf_joint_params(self.urdf_fullpath, P.JOINT_NAME)

        self.main_thread = None
        self.is_main_setup = False

        self.sim_thread = None
        self.is_mujoco_setup = False

        self.can_thread = None
        self.is_can_setup = False
        self.motors = [None]*P.N_JOINTS

        self.robot_state = RobotState()
        self.peripheral_state = PeripheralState()
        self.robot_command = RobotCommand()

        rospy.Subscriber("/mevita_command", String, self.ros_command_callback, queue_size=1)
        rospy.Subscriber("/livox/imu_filtered", Imu, self.imu_callback, queue_size=1)
        rospy.Subscriber("/ps4joy/joy", Joy, self.ps4joy_joy_callback, queue_size=1)
        rospy.Subscriber("/spacenav/joy", Joy, self.spacenav_joy_callback, queue_size=1)
        rospy.Subscriber("/virtual/joy", Joy, self.virtual_joy_callback, queue_size=1)
        atexit.register(self.disable_all_motors)

    def disable_all_motors(self):
        for motor in self.motors:
            if motor is not None:
                pos, vel, tau, tem = motor.disable_motor()
                print("Disabling Motor {} [Status] Pos: {:.3f}, Vel: {:.3f}, Tau: {:.3f}, Temp: {:.3f}".format(motor.motor_id, pos, vel, tau, tem))

    def imu_callback(self, msg):
        with self.peripheral_state.lock:
            self.peripheral_state.body_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            self.peripheral_state.body_gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            self.peripheral_state.body_acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            self.peripheral_state.imu_last_time = time.time()

    def vel_callback(self, msg):
        with self.peripheral_state.lock:
            self.peripheral_state.body_vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]

    def virtual_joy_callback(self, msg):
        with self.peripheral_state.lock:
            self.peripheral_state.virtual_enable = True
            self.peripheral_state.virtual = [msg.axes[0], msg.axes[1], msg.buttons[0], msg.buttons[1]]
            one_pushed = self.peripheral_state.virtual[2]
            two_pushed = self.peripheral_state.virtual[3]

        if one_pushed == 1:
            self.command_callback("STANDBY-STANDUP")
        elif two_pushed == 1:
            self.command_callback("STANDUP-WALK")

    def ros_command_callback(self, msg):
        print("Received ROS Command: {}".format(msg.data))
        self.command_callback(msg.data)

    def ps4joy_joy_callback(self, msg):
        with self.peripheral_state.lock:
            self.peripheral_state.ps4joy_enable = True
            self.peripheral_state.ps4joy = msg.axes[:] + msg.buttons[:] # 14 + 14
            cross_pushed = self.peripheral_state.ps4joy[14+1]
            circle_pushed = self.peripheral_state.ps4joy[14+2]

        if cross_pushed == 1:
            self.command_callback("STANDBY-STANDUP")
        elif circle_pushed == 1:
            self.command_callback("STANDUP-WALK")

    def spacenav_joy_callback(self, msg):
        with self.peripheral_state.lock:
            self.peripheral_state.spacenav_enable = True
            self.peripheral_state.spacenav = [msg.axes[0], msg.axes[1], msg.axes[2], msg.axes[3], msg.axes[4], msg.axes[5], msg.buttons[0], msg.buttons[1]]
            left_pushed = self.peripheral_state.spacenav[6]
            right_pushed = self.peripheral_state.spacenav[7]

        if left_pushed == 1:
            self.command_callback("STANDBY-STANDUP")
        elif right_pushed == 1:
            self.command_callback("STANDUP-WALK")

    def command_callback(self, command):
        with self.robot_command.lock:
            prev_command = self.robot_command.command
            if not self.robot_command.initialized:
                pass
        if command == "STANDBY-STANDUP":
            with self.robot_command.lock:
                if self.robot_command.remaining_time < 0.1:
                    if prev_command == "STANDBY":
                        self.robot_command.command = "STANDBY2"
                        with self.robot_state.lock:
                            self.robot_command.initial_angle = self.robot_command.angle[:]
                            self.robot_command.final_angle = P.STANDBY2_ANGLE[:]
                            self.robot_command.interpolating_time = 5.0
                            self.robot_command.remaining_time = self.robot_command.interpolating_time
                    elif prev_command == "STANDBY2":
                        self.robot_command.command = "STANDUP"
                        with self.robot_state.lock:
                            self.robot_command.initial_angle = self.robot_command.angle[:]
                            self.robot_command.final_angle = P.DEFAULT_ANGLE[:]
                            self.robot_command.interpolating_time = 1.0
                            self.robot_command.remaining_time = self.robot_command.interpolating_time
                    elif prev_command == "STANDUP":
                        self.robot_command.command = "STANDBY"
                        with self.robot_state.lock:
                            self.robot_command.initial_angle = self.robot_command.angle[:]
                            self.robot_command.final_angle = P.STANDBY_ANGLE[:]
                            self.robot_command.interpolating_time = 5.0
                            self.robot_command.remaining_time = self.robot_command.interpolating_time
        elif command == "STANDUP-WALK":
            with self.robot_command.lock:
                if self.robot_command.remaining_time < 1.0:
                    if prev_command == "STANDUP":
                        self.robot_command.command = "WALK"
                        self.robot_command.interpolating_time = 3.0
                        self.robot_command.remaining_time = self.robot_command.interpolating_time
                    elif prev_command == "WALK":
                        self.robot_command.command = "STANDUP"
                        with self.robot_state.lock:
                            self.robot_command.initial_angle = self.robot_state.angle[:]
                            self.robot_command.final_angle = P.DEFAULT_ANGLE[:]
                            self.robot_command.interpolating_time = 3.0
                            self.robot_command.remaining_time = self.robot_command.interpolating_time
        elif command == "STANDBY":
            with self.robot_command.lock:
                self.robot_command.command = "STANDBY"
                with self.robot_state.lock:
                    self.robot_command.initial_angle = self.robot_state.angle[:]
                    self.robot_command.final_angle = P.STANDBY_ANGLE[:]
                    self.robot_command.interpolating_time = 5.0
                    self.robot_command.remaining_time = self.robot_command.interpolating_time
        elif command == "STANDUP":
                self.robot_command.command = "STANDUP"
                with self.robot_state.lock:
                    self.robot_command.initial_angle = self.robot_state.angle[:]
                    self.robot_command.final_angle = P.DEFAULT_ANGLE[:]
                    self.robot_command.interpolating_time = 5.0
                    self.robot_command.remaining_time = self.robot_command.interpolating_time
        elif prev_command == "STANDUP" and command == "WALK":
                self.robot_command.command = "WALK"

        with self.robot_command.lock:
            print("Command changed from {} to {}".format(prev_command, self.robot_command.command))

    def setup_main(self):
        if self.is_main_setup:
            return
        self.main_thread = threading.Thread(target=self.main_thread_func)
        self.main_thread.daemon = True
        self.main_thread.start()
        self.is_main_setup = True

    def main_thread_func(self):
        policy_path = os.path.join(os.path.dirname(__file__), "../models/policy.pt")
        policy = mevita_utils.read_torch_policy(policy_path).to("cpu")

        is_safe = True

        rate = rospy.Rate(P.CONTROL_HZ)
        while not rospy.is_shutdown():
            with self.robot_command.lock:
                command = self.robot_command.command
            if command in ["STANDBY", "STANDBY2", "STANDUP"]:
                with self.robot_command.lock:
                    self.robot_command.remaining_time -= 1.0/P.CONTROL_HZ
                    self.robot_command.remaining_time = max(0, self.robot_command.remaining_time)
                    if self.robot_command.remaining_time <= 0:
                        pass
                    else:
                        ratio = 1 - self.robot_command.remaining_time / self.robot_command.interpolating_time
                        self.robot_command.angle = [a + (b-a)*ratio for a, b in zip(self.robot_command.initial_angle, self.robot_command.final_angle)]
            elif command in ["WALK"]:
                with self.robot_command.lock:
                    self.robot_command.remaining_time -= 1.0/P.CONTROL_HZ
                    self.robot_command.remaining_time = max(0, self.robot_command.remaining_time)

                with self.peripheral_state.lock:
                    base_quat = self.peripheral_state.body_quat[:]
                    base_lin_vel = self.peripheral_state.body_vel[:]
                    base_ang_vel = self.peripheral_state.body_gyro[:]

                    if self.peripheral_state.ps4joy_enable:
                        nav = self.peripheral_state.ps4joy[:]
                        max_command = 1.0
                        commands_ = [nav[1], nav[0], nav[2], nav[2]]
                        commands = [[min(max(-1.0, command / max_command), 1.0) for command in commands_]]
                    elif self.peripheral_state.spacenav_enable:
                        nav = self.peripheral_state.spacenav[:]
                        max_command = 0.6835
                        commands_ = [nav[0], nav[1], nav[5], nav[5]]
                        commands = [[min(max(-1.0, command / max_command), 1.0) for command in commands_]]
                    elif self.peripheral_state.virtual_enable:
                        nav = self.peripheral_state.virtual[:]
                        max_command = 1.0
                        commands_ = [nav[1], nav[0], 0, 0]
                        commands = [[min(max(-1.0, command / max_command), 1.0) for command in commands_]]
                    else:
                        commands = [[0.0, 0.0, 0.0, 0.0]]

            # for safety
            if command in ["WALK"]:
                # no imu
                with self.peripheral_state.lock:
                    if self.peripheral_state.imu_last_time is None:
                        is_safe = False
                        print("No Connection to IMU. PD gains become 0.")
                    if (self.peripheral_state.imu_last_time is not None) and (time.time() - self.peripheral_state.imu_last_time > 0.1):
                        print("IMU data is too old. PD gains become 0.")
                        is_safe = False
                # falling down
                if is_safe and (Rotation.from_quat(base_quat).as_matrix()[2, 2] < 0.6):
                    is_safe = False
                    print("Robot is almost fell down. PD gains become 0.")

                if not is_safe:
                    print("Robot is not safe. Please reboot the robot.")
                    with self.robot_command.lock:
                        self.robot_command.kp = [0.0] * P.N_JOINTS
                        self.robot_command.kd = [0.0] * P.N_JOINTS
                        with self.robot_state.lock:
                            self.robot_command.angle = self.robot_state.angle[:]
                    rate.sleep()
                    continue

            if command in ["WALK"]:
                with self.robot_state.lock:
                    dof_pos = self.robot_state.angle[:]
                    dof_vel = self.robot_state.velocity[:]
                obs = mevita_utils.get_policy_observation(base_quat, base_lin_vel, base_ang_vel, commands, dof_pos, dof_vel)
                actions = mevita_utils.get_policy_output(policy, obs)
                scaled_actions = P.ACTION_SCALE * actions

                ref_angle = [a + b for a, b in zip(scaled_actions, P.DEFAULT_ANGLE[:])]
                with self.robot_state.lock:
                    for i in range(len(ref_angle)):
                        if self.robot_state.angle[i]  < self.joint_params["lower"][i] or self.robot_state.angle[i] > self.joint_params["upper"][i]:
                            ref_angle[i] = max(self.joint_params["lower"][i]+0.1, min(ref_angle[i], self.joint_params["upper"][i]-0.1))
                            print("# Joint {} out of range: {:.3f}".format(P.JOINT_NAME[i], self.robot_state.angle[i]))
                with self.robot_command.lock:
                    self.robot_command.angle = ref_angle

            rate.sleep()

    def setup_mujoco(self):
        if self.is_mujoco_setup:
            return
        self.sim_thread = threading.Thread(target=self.mujoco_thread_func)
        self.sim_thread.daemon = True
        self.sim_thread.start()
        self.is_mujoco_setup = True

    def mujoco_thread_func(self):
        import mujoco
        import mujoco_viewer

        model = mujoco.MjModel.from_xml_path(self.xml_fullpath)
        data = mujoco.MjData(model)
        viewer = mujoco_viewer.MujocoViewer(model, data)

        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_step(model, data)

        mujoco_joint_names = [model.joint(i).name for i in range(model.njnt)]
        print("mujoco joints: ", mujoco_joint_names)
        with self.robot_state.lock:
            for i, name in enumerate(P.JOINT_NAME):
                joint_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) # mujoco
                qpos_idx = model.jnt_qposadr[joint_idx]
                qvel_idx = model.jnt_dofadr[joint_idx]
                self.robot_state.angle[i] = data.qpos[qpos_idx]
                self.robot_state.velocity[i] = data.qvel[qvel_idx]
                self.robot_state.torque[i] = 0.0
                self.robot_state.temperature[i] = 25.0

        mujoco_actuator_names = [model.actuator(i).name for i in range(model.nu)]
        print("mujoco actuators: ", mujoco_actuator_names)
        data.ctrl[:] = 0.0

        with self.robot_state.lock:
            self.robot_state.angle = P.STANDBY_ANGLE[:]

        with self.robot_command.lock:
            self.robot_command.angle = P.STANDBY_ANGLE[:]

        jointstate_pub = rospy.Publisher("joint_states", JointState, queue_size=2)
        mevita_pub = rospy.Publisher("mevita_log", MevitaLog, queue_size=2)

        prev_actions = []
        with self.robot_command.lock:
            p_ref = self.robot_command.angle[:]
        for i in range(3): # 4/200 = 0.02 sec
            prev_actions.append(p_ref)

        print("start mujoco thread...")
        rate = rospy.Rate(P.SIM_HZ)
        viewer_count = 0
        while viewer.is_alive and (not rospy.is_shutdown()):

            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

            with self.robot_command.lock:
                p_ref = self.robot_command.angle[:]
                v_ref = self.robot_command.velocity[:]
                kp_ref = self.robot_command.kp[:]
                kd_ref = self.robot_command.kd[:]
                tau_ref = self.robot_command.torque[:]

            p_ref_delayed = prev_actions.pop(0)

            for i, name in enumerate(P.JOINT_NAME): # mevita
                if name in mujoco_actuator_names: # mujoco
                    actuator_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) # mujoco
                    joint_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) # mujoco
                    qpos_idx = model.jnt_qposadr[joint_idx]
                    qvel_idx = model.jnt_dofadr[joint_idx]
                    error = p_ref_delayed[i] - data.qpos[qpos_idx]
                    derivative = v_ref[i] - data.qvel[qvel_idx]
                    data.ctrl[actuator_idx] = kp_ref[i]*error + kd_ref[i]*derivative + tau_ref[i]

            prev_actions.append(p_ref)

            mujoco.mj_step(model, data)

            # total_mass = 0.0
            # cog = np.zeros(3)
            # print("-----------------")
            # for i in range(model.nbody):
            #     if i == 0 or i == model.nbody-1: # world link or terrain object
            #         continue
            #     mass = model.body_mass[i]
            #     pos = data.xipos[i]  # body i's world coordinate
            #     print(mass, pos)
            #     cog += mass * pos
            #     total_mass += mass
            # cog /= total_mass
            # print(cog, total_mass)

            # for i in range(model.nbody):
            #     print(data.xpos[i])

            with self.robot_state.lock:
                for i, name in enumerate(P.JOINT_NAME):
                    if name in mujoco_actuator_names:
                        actuator_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                        joint_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                        qpos_idx = model.jnt_qposadr[joint_idx]
                        qvel_idx = model.jnt_dofadr[joint_idx]
                        self.robot_state.angle[i] = data.qpos[qpos_idx]
                        self.robot_state.velocity[i] = data.qvel[qvel_idx]
                        self.robot_state.torque[i] = data.ctrl[actuator_idx]
                        self.robot_state.temperature[i] = 25.0

            gyro_data = data.sensor("body_gyro_sensor").data.copy()
            acc_data = data.sensor("body_acc_sensor").data.copy()
            vel_data = data.sensor("body_vel_sensor").data.copy()
            quat_data = data.sensor("body_quat_sensor").data.copy()

            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.orientation.x = quat_data[1]
            imu_msg.orientation.y = quat_data[2]
            imu_msg.orientation.z = quat_data[3]
            imu_msg.orientation.w = quat_data[0]
            imu_msg.angular_velocity.x = gyro_data[0]
            imu_msg.angular_velocity.y = gyro_data[1]
            imu_msg.angular_velocity.z = gyro_data[2]
            imu_msg.linear_acceleration.x = acc_data[0]
            imu_msg.linear_acceleration.y = acc_data[1]
            imu_msg.linear_acceleration.z = acc_data[2]
            self.imu_callback(imu_msg)

            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.twist.twist.linear.x = vel_data[0]
            odom_msg.twist.twist.linear.y = vel_data[1]
            odom_msg.twist.twist.linear.z = vel_data[2]
            self.vel_callback(odom_msg)

            jointstate_msg = JointState()
            jointstate_msg.header.stamp = rospy.Time.now()
            jointstate_msg.name = P.JOINT_NAME
            with self.robot_state.lock:
                jointstate_msg.position = self.robot_state.angle
                jointstate_msg.velocity = self.robot_state.velocity
                jointstate_msg.effort = self.robot_state.torque
            jointstate_pub.publish(jointstate_msg)

            mevita_msg = MevitaLog()
            mevita_msg.header.stamp = rospy.Time.now()
            with self.robot_state.lock:
                mevita_msg.angle = self.robot_state.angle[:]
                mevita_msg.velocity = self.robot_state.velocity[:]
                mevita_msg.torque = self.robot_state.torque[:]
                mevita_msg.temperature = self.robot_state.temperature[:]
            with self.peripheral_state.lock:
                mevita_msg.body_vel = self.peripheral_state.body_vel[:]
                mevita_msg.body_quat = self.peripheral_state.body_quat[:]
                mevita_msg.body_gyro = self.peripheral_state.body_gyro[:]
                mevita_msg.body_acc = self.peripheral_state.body_acc[:]
            with self.peripheral_state.lock:
                mevita_msg.ref_angle = self.robot_command.angle[:]
                mevita_msg.ref_velocity = self.robot_command.velocity[:]
                mevita_msg.ref_kp = self.robot_command.kp[:]
                mevita_msg.ref_kd = self.robot_command.kd[:]
                mevita_msg.ref_torque = self.robot_command.torque[:]
            mevita_pub.publish(mevita_msg)

            if viewer_count == 0: # render takes time...
                viewer.render()
            viewer_count = (viewer_count + 1) % 4

            rate.sleep()

    def setup_can(self):
        if self.is_can_setup:
            return
        self.can_bus_list = list(set(P.DEVICE))  # ["can0", "can1"]
        self.can_threads = []

        for bus_name in self.can_bus_list:
            motor_indices = [i for i, dev in enumerate(P.DEVICE) if dev == bus_name]
            t = threading.Thread(target=self.can_thread_func, args=(bus_name, motor_indices))
            t.daemon = True
            t.start()
            self.can_threads.append(t)

        self.is_can_setup = True

    def can_thread_func(self, bus_name, motor_indices):
        print(f"[{bus_name}] Setting up {len(motor_indices)} motors...")
        for i in motor_indices:
            self.motors[i] = CanMotorController(bus=P.DEVICE[i], motor_id=P.CAN_ID[i], motor_type=P.MOTOR_TYPE[i], motor_dir=P.MOTOR_DIR[i])

        print(f"[{bus_name}] Enabling Motors...")
        for i in motor_indices:
            motor = self.motors[i]
            pos, vel, tau, tem = motor.enable_motor()
            print(f"[{bus_name}] Enabling Motor {P.JOINT_NAME[i]} [Status] Pos: {pos:.3f}, Vel: {vel:.3f}, Tau: {tau:.3f}, Temp: {tem:.3f}")
            if (tem > 70.0) or (tem < 0.0) or (abs(tau) > 3.0) or (abs(vel) > 1.0) or (abs(pos) > 10.0):
                print(f"################ [{bus_name}] Something is wrong. Cannot continue the operation.")
                sys.exit()
            with self.robot_state.lock:
                self.robot_state.angle[i] = pos
                self.robot_state.velocity[i] = vel
                self.robot_state.torque[i] = tau
                self.robot_state.temperature[i] = tem

        print(f"[{bus_name}] Setting Initial Offset...")
        for i in motor_indices:
            motor = self.motors[i]
            motor.set_angle_offset(P.STANDBY_ANGLE[i], deg=False)
            motor.set_angle_range(self.joint_params["lower"][i], self.joint_params["upper"][i], deg=False)

        with self.robot_command.lock:
            self.robot_command.angle = P.STANDBY_ANGLE[:]

        with self.robot_state.lock:
            self.robot_state.angle = P.STANDBY_ANGLE[:]

        print(f"[{bus_name}] Gradually Setting Motors to Initial Position...")
        start_time = time.time()
        interpolation_time = 3.0
        while True:
            t = (time.time() - start_time) / interpolation_time
            if t >= 1.0:
                break
            with self.robot_command.lock:
                p_ref = self.robot_command.angle
            for i in motor_indices:
                motor = self.motors[i]
                kp_ref = mevita_utils.interpolate_linear(0.0, P.KP_GAIN[i], t)
                kd_ref = mevita_utils.interpolate_linear(0.0, P.KD_GAIN[i], t)
                try:
                    pos, vel, tau, tem = motor.send_rad_command(p_ref[i], 0, kp_ref, kd_ref, 0)
                except:
                    print(f"################ [{bus_name}] Can Receiver Failed for {P.JOINT_NAME[i]}")
                    continue
                with self.robot_state.lock:
                    self.robot_state.angle[i] = pos
                    self.robot_state.velocity[i] = vel
                    self.robot_state.torque[i] = tau
                    self.robot_state.temperature[i] = tem
            time.sleep(1.0 / P.CAN_HZ)

        error = 0.0
        for i in motor_indices:
            error += (self.robot_state.angle[i] - P.STANDBY_ANGLE[i])**2
        error = np.sqrt(error)
        if error > 0.3: # [rad]
            print(f"################ [{bus_name}] The initial pose is too different from the reset pose ({error:.3f}). Cannot continue the operation.")
            sys.exit()
        else:
            print(f"[{bus_name}] Position Error from reset pose, OK: {error:.3f}")

        if bus_name == "can0":
            jointstate_pub = rospy.Publisher("joint_states", JointState, queue_size=2)
            mevita_pub = rospy.Publisher("mevita_log", MevitaLog, queue_size=2)

        print(f"[{bus_name}] CAN thread started")
        rate = rospy.Rate(P.CAN_HZ)
        error_count = [0] * P.N_JOINTS
        while not rospy.is_shutdown():

            with self.robot_command.lock:
                p_ref = self.robot_command.angle[:]
                v_ref = self.robot_command.velocity[:]
                kp_ref = self.robot_command.kp[:]
                kd_ref = self.robot_command.kd[:]
                tau_ref = self.robot_command.torque[:]
                # p_ref = [0.0]*P.N_JOINTS
                # v_ref = [0.0]*P.N_JOINTS
                # kp_ref = [0.0]*P.N_JOINTS
                # kd_ref = [0.0]*P.N_JOINTS
                # tau_ref = [0.0]*P.N_JOINTS

            pos_list = [0]*P.N_JOINTS
            vel_list = [0]*P.N_JOINTS
            tau_list = [0]*P.N_JOINTS
            tem_list = [0]*P.N_JOINTS

            for i in motor_indices:
                motor = self.motors[i]
                try:
                    pos, vel, tau, tem = motor.send_rad_command(p_ref[i], v_ref[i], kp_ref[i], kd_ref[i], tau_ref[i])
                except:
                    error_count[i] += 1
                    print(f"################ [{bus_name}] Can Receiver Failed for {P.JOINT_NAME[i]} ({error_count[i]})")
                    continue
                if (pos is None) or (vel is None) or (tau is None) or (tem is None):
                    sys.exit()
                pos_list[i] = pos
                vel_list[i] = vel
                tau_list[i] = tau
                tem_list[i] = tem
                # print(f"[{bus_name}] Motor {P.JOINT_NAME[i]} [Status] Pos: {pos:.3f}, Vel: {vel:.3f}, Tau: {tau:.3f}, Temp: {tem:.3f}")
                if tem > 80.0:
                    print(f"[{bus_name}] Motor {P.JOINT_NAME[i]} is Overheated!!!")
                    pos, vel, tau, tem = motor.disable_motor()
                    print(f"################ [{bus_name}] Disabling Motor {P.JOINT_NAME[i]} [Status] Pos: {pos:.3f}, Vel: {vel:.3f}, Tau: {tau:.3f}, Temp: {tem:.3f}")

            ave_scale = 1.0
            with self.robot_state.lock:
                for i in motor_indices:
                    self.robot_state.angle[i] = pos_list[i]
                    self.robot_state.velocity[i] = (1.0-ave_scale)*self.robot_state.velocity[i] + ave_scale*vel_list[i]
                    self.robot_state.torque[i] = (1.0-ave_scale)*self.robot_state.torque[i] + ave_scale*tau_list[i]
                    self.robot_state.temperature[i] = (1.0-ave_scale)*self.robot_state.temperature[i] + ave_scale*tem_list[i]

            if bus_name == "can0":
                jointstate_msg = JointState()
                jointstate_msg.header.stamp = rospy.Time.now()
                jointstate_msg.name = P.JOINT_NAME
                with self.robot_state.lock:
                    jointstate_msg.position = self.robot_state.angle
                    jointstate_msg.velocity = self.robot_state.velocity
                    jointstate_msg.effort = self.robot_state.torque
                jointstate_pub.publish(jointstate_msg)

                mevita_msg = MevitaLog()
                mevita_msg.header.stamp = rospy.Time.now()
                with self.robot_state.lock:
                    mevita_msg.angle = self.robot_state.angle[:]
                    mevita_msg.velocity = self.robot_state.velocity[:]
                    mevita_msg.torque = self.robot_state.torque[:]
                    mevita_msg.temperature = self.robot_state.temperature[:]
                with self.peripheral_state.lock:
                    mevita_msg.body_vel = self.peripheral_state.body_vel[:]
                    mevita_msg.body_quat = self.peripheral_state.body_quat[:]
                    mevita_msg.body_gyro = self.peripheral_state.body_gyro[:]
                    mevita_msg.body_acc = self.peripheral_state.body_acc[:]
                with self.peripheral_state.lock:
                    mevita_msg.ref_angle = self.robot_command.angle[:]
                    mevita_msg.ref_velocity = self.robot_command.velocity[:]
                    mevita_msg.ref_kp = self.robot_command.kp[:]
                    mevita_msg.ref_kd = self.robot_command.kd[:]
                    mevita_msg.ref_torque = self.robot_command.torque[:]
                mevita_pub.publish(mevita_msg)

            rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true", help="do simulation")
    args = parser.parse_args()

    mevita = Mevita()
    if args.sim:
        mevita.setup_mujoco()
    else:
        mevita.setup_can()
    mevita.main_thread_func()

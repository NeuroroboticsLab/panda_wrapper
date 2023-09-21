#!/usr/bin/env python

import threading

import actionlib
import json
import rospy
import cv2
from pytransform3d import transformations as pt
import numpy as np
from scipy.spatial.transform import Rotation as R


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from franka_gripper.msg import *
from franka_msgs.msg import FrankaState

import franka_gripper.msg
import franka_gripper

from panda_wrapper.srv import *


def transform_to_pose(transform):
    pose = PoseStamped()
    pose.pose.position.x = transform[0, 3]
    pose.pose.position.y = transform[1, 3]
    pose.pose.position.z = transform[2, 3]
    r = R.from_matrix(transform[:3, :3])
    quat = r.as_quat()
    pose.pose.orientation.x = quat[2]
    pose.pose.orientation.y = -quat[3]
    pose.pose.orientation.z = -quat[0]
    pose.pose.orientation.w = quat[1]
    return pose


class RobotCart:
    def __init__(self, rate=10):
        self.rate = rospy.Rate(rate)
        self.pose = None

        threading.Thread(target=self.sender).start()
        # Current robot end-effector pose expressed relative to the base frame.
        self.O_T_EE = []

        self.sub = rospy.Subscriber(
            "/franka_state_controller/franka_states", FrankaState, self.joint_callback)

    def joint_callback(self, franka_state):
        # print("Franka State: ", franka_state)
        self.O_T_EE = np.reshape(franka_state.O_T_EE, (4, 4), order='F')
        self.O_T_EE[:3, :3] = np.linalg.qr(
            self.O_T_EE[:3, :3], mode='complete')[0]

        self.pose = transform_to_pose(self.O_T_EE)
        print(self.pose)
        self.sub.unregister()

    def move_tcp_delta(self, delta_pos, delta_ori=[0, 0, 0], degrees=True):
        self.pose.pose.position.x += delta_pos[0]
        self.pose.pose.position.y += delta_pos[1]
        self.pose.pose.position.z += delta_pos[2]

        if (delta_ori[0] == 0 and delta_ori[1] == 0 and delta_ori[2] == 0):
            return  # do not chage orientation

        r_delta = R.from_euler('xyz', delta_ori, degrees)
        r = R.from_quat([self.pose.pose.orientation.x, self.pose.pose.orientation.y,
                         self.pose.pose.orientation.z, self.pose.pose.orientation.w])
        quat_old = r.as_quat()

        r_new = r * r_delta
        quat_new = r_new.as_quat()
        self.pose.pose.orientation.x = quat_new[0]
        self.pose.pose.orientation.y = quat_new[1]
        self.pose.pose.orientation.z = quat_new[2]
        self.pose.pose.orientation.w = quat_new[3]

    def move_tcp(self, pos, ori=None, degrees=True):
        self.pose.pose.position.x = pos[0]
        self.pose.pose.position.y = pos[1]
        self.pose.pose.position.z = pos[2]

        if ori is None:
            return

        q = R.from_euler('xyz', ori, degrees).as_quat()
        self.pose.pose.orientation.x = q[0]
        self.pose.pose.orientation.y = q[1]
        self.pose.pose.orientation.z = q[2]
        self.pose.pose.orientation.w = q[3]

    def sender(self):
        pub = rospy.Publisher(
            '/cartesian_impedance_controller/equilibrium_pose', PoseStamped, queue_size=10)

        while not rospy.is_shutdown():
            # print(pose)
            if self.pose is not None:
                pub.publish(self.pose)
                # print("init")
            # rospy.loginfo(self.pose)
            self.rate.sleep()


class RobotJoint:
    def __init__(self, rate=10):
        self.rate = rospy.Rate(rate)
        self.real_state = JointState()
        self.sub = rospy.Subscriber(
            "/joint_states", JointState, self.state_callback)
        rospy.sleep(0.5)
        self.desired_state = self.real_state

        threading.Thread(target=self.sender).start()

    def sender(self):
        pub = rospy.Publisher(
            '/desired_joints', JointState, queue_size=10)

        while not rospy.is_shutdown():
            pub.publish(self.desired_state)
            # rospy.loginfo(self.desired_state)
            self.rate.sleep()

    def state_callback(self, joint_data):
        self.real_state = joint_data

    def set_target(self, joints):
        print("Set target: ", joints.position)
        self.desired_state = joints

    def stop(self):
        self.desired_state = self.real_state
        rospy.sleep(2)


class Gripper:
    def __init__(self, rate=10):
        self.rate = rospy.Rate(rate)
        self.grasp_client = actionlib.SimpleActionClient(
            '/franka_gripper/grasp', GraspAction)
        self.homing_client = actionlib.SimpleActionClient(
            '/franka_gripper/homing', HomingAction)
        self.moving_client = actionlib.SimpleActionClient(
            '/franka_gripper/move', MoveAction)
        self.stop_client = actionlib.SimpleActionClient(
            '/franka_gripper/stop', StopAction)
        self.grasp_client.wait_for_server()
        self.homing_client.wait_for_server()
        self.moving_client.wait_for_server()
        self.stop_client.wait_for_server()
        print("Gripper ready")

    def grasp(self, width_in, speed_in=0.1, force_in=40.0):
        goal = franka_gripper.msg.GraspGoal(
            width=width_in, speed=speed_in, force=force_in)
        goal.epsilon.inner = 0.03
        goal.epsilon.outer = 0.03
        self.grasp_client.send_goal(goal)
        self.grasp_client.wait_for_result()
        print("Grasp result: ", self.grasp_client.get_result())

    def move(self, width_in, speed_in=0.1):
        goal = franka_gripper.msg.MoveGoal(width=width_in, speed=speed_in)
        self.moving_client.send_goal(goal)
        self.moving_client.wait_for_result()

    def homing(self):
        self.homing_client.send_goal(franka_gripper.msg.HomingGoal())
        self.homing_client.wait_for_result()

    def stop(self):
        self.stop_client.send_goal(franka_gripper.msg.StopAction())
        self.stop_client.wait_for_result()


class StateViewer:
    def __init__(self, rate=10, print_data=False):
        self.rate = rospy.Rate(rate)
        self.print_data = print_data
        self.joints = []
        # Measured link-side joint torque sensor signals
        self.tau_J = []
        # Desired link-side joint torque sensor signals without gravity.
        self.tau_J_d = []
        # External torque, filtered.
        self.tau_ext_hat_filtered = []
        # Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame.
        self.O_F_ext_hat_K = []
        # Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the stiffness frame.
        self.K_F_ext_hat_K = []
        # Current robot end-effector pose expressed relative to the base frame.
        self.base_to_ee = []

        self.sub = rospy.Subscriber(
            "/franka_state_controller/franka_states", FrankaState, self.joint_callback)

    def joint_callback(self, franka_state):
        # print("Franka State: ", franka_state)
        self.joints = franka_state.q
        self.tau_J = franka_state.tau_J
        self.tau_J_d = franka_state.tau_J_d
        self.tau_ext_hat_filtered = franka_state.tau_ext_hat_filtered
        self.O_F_ext_hat_K = franka_state.O_F_ext_hat_K
        self.K_F_ext_hat_K = franka_state.K_F_ext_hat_K

        self.base_to_ee = np.reshape(franka_state.O_T_EE, (4, 4), order='F')
        self.base_to_ee[:3, :3] = np.linalg.qr(
            self.base_to_ee[:3, :3], mode='complete')[0]

    def get_state(self):
        return [self.joints, self.tcp]

    def print_state(self):
        print("Translation: ", self.base_to_ee.transform.translation)
        print("Rotation: ", self.base_to_ee.transform.rotation)

        print("Joint Values: ", self.joints)
        print("Measured link-side joint torque sensor signals: ", self.tau_J)
        print(
            "Desired link-side joint torque sensor signals without gravity: ", self.tau_J_d)
        print("Ext torque, filtered: ", self.tau_ext_hat_filtered)
        print("Estimated ext wrench on stiffness frame, to base frame: ",
              self.O_F_ext_hat_K)
        print("Estimated ext wrench on stiffness frame, to stiffness frame: ",
              self.K_F_ext_hat_K)
        print("\n")

    def save_state(self, path="/home/sascha/catkin_ws/data/transform.xml"):
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
        cv_file.write("base_to_ee", self.base_to_ee)
        cv_file.write("ee_to_base", pt.invert_transform(self.base_to_ee))
        cv_file.write("joints", self.joints)

        cv_file.release()


def load_state(path, print_data=False):
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    base_to_ee = cv_file.getNode("base_to_ee").mat()
    ee_to_base = cv_file.getNode("ee_to_base").mat()
    joints = cv_file.getNode("joints").mat()

    if print_data:
        print("base_to_ee: \n", base_to_ee)
        print("ee_to_base: \n", ee_to_base)

    return base_to_ee, ee_to_base, joints

    with open(path) as json_file:
        data = json.load(json_file)

        joints = data['joints']

        tcp = tf2_ros.TransformStamped()
        translation = data['translation']
        quat = data['rotation']
        tcp.transform.translation = translation
        tcp.transform.rotation = quat

        if print_data:
            print("joint Positions: ", joints)
            print("tcp: ", tcp)
        return joints, tcp

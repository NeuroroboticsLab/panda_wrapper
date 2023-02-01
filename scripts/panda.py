#!/usr/bin/env python

import threading

import actionlib
import json
import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from franka_gripper.msg import *
import tf2_ros
import franka_gripper.msg
import franka_gripper

from move_robot.srv import *


def transform_to_pose(trans):
    pose = PoseStamped()
    pose.pose.position = trans.transform.translation
    pose.pose.orientation = trans.transform.rotation
    pose.header.stamp = trans.header.stamp
    return pose


class RobotCart:
    def __init__(self, rate=10):
        self.rate = rospy.Rate(rate)
        self.pose = PoseStamped()
        self.tf_buffer = tf2_ros.Buffer()

        listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(1)

        self.start_trans = self.tf_buffer.lookup_transform(
            'fr3_link0', 'fr3_EE', rospy.Time())
        self.pose = transform_to_pose(self.start_trans)

        threading.Thread(target=self.sender).start()

    def sender(self):
        pub = rospy.Publisher(
            '/cartesian_impedance_controller/equilibrium_pose', PoseStamped, queue_size=10)

        while not rospy.is_shutdown():
            # print(pose)
            pub.publish(self.pose)
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
        self.joint_state = JointState()
        self.sub = rospy.Subscriber(
            "/joint_states", JointState, self.joint_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(0.1)
        self.tcp = self.tf_buffer.lookup_transform(
            'fr3_link0', 'fr3_EE', rospy.Time())
        threading.Thread(target=self.tcp_thread).start()

    def joint_callback(self, joint_data):
        self.joint_state = joint_data

    def tcp_thread(self):
        while not rospy.is_shutdown():
            self.tcp = self.tf_buffer.lookup_transform(
                'fr3_link0', 'fr3_EE', rospy.Time())
            if self.print_data:
                self.print_state()
            self.rate.sleep()

    def get_state(self):
        return [self.joint_state, self.tcp]

    def print_state(self):
        print("Translation: ", self.tcp.transform.translation)

        print("Rotation: ", self.tcp.transform.rotation)
        print("Joint State: ", self.joint_state.position)
        print("\n \n \n")

    def save_state(self, path="/home/sascha/catkin_ws/data/robot_data.json"):
        translation = (self.tcp.transform.translation.x, self.tcp.transform.translation.y,
                       self.tcp.transform.translation.z)
        quat = (self.tcp.transform.rotation.x, self.tcp.transform.rotation.y,
                self.tcp.transform.rotation.z, self.tcp.transform.rotation.w)

        json_dict = {'translation': translation,
                     'rotation': quat,
                     'joint_states': self.joint_state.position,
                     'joint_names': self.joint_state.name,
                     'info': 'Translation and Rotation of the TCP and the joint state of the robot. Rotation as x,y,z,w'}
        json_object = json.dumps(json_dict, indent=4, sort_keys=True)
        with open(path, "w") as outfile:
            outfile.write(json_object)


def load_state(path, print_data=False):
    with open(path) as json_file:
        data = json.load(json_file)

        joints = JointState()
        joints.position = data['joint_states']
        joints.name = data['joint_names']

        tcp = tf2_ros.TransformStamped()
        translation = data['translation']
        quat = data['rotation']
        tcp.transform.translation = translation
        tcp.transform.rotation = quat

        if print_data:
            print("joint Positions: ", joints.position)
            print("joint Names: ", joints.name)
            print("tcp: ", tcp)
        return joints, tcp

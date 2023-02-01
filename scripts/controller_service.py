#!/usr/bin/env python

import threading
from enum import Enum

import rospy
import roslaunch

from move_robot.srv import *


class Type(Enum):
    NONE = "none"
    CARTESIAN = "cartesian"
    JOINT = "joint"
    FORCE = "force"
    VELOCITY = "velocity"
    STOP = "stop"

    def __str__(self):
        return '%s' % self.value


global type
type = Type.NONE


class ControllerNode:
    def __init__(self):
        self.controller = None
        self.uuid = None

    def start(self, type):
        print("launching controller")

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        if type == Type.CARTESIAN:
            cli_args = ['/home/sascha/catkin_ws/src/franka_ros/franka_example_controllers/launch/cartesian_impedance_controller.launch',
                        'robot_ip:=172.16.0.2', "load_gripper:=true", "robot:=fr3"]
        elif type == Type.JOINT:
            cli_args = ['/home/sascha/catkin_ws/src/franka_ros/franka_example_controllers/launch/joint_position_controller.launch',
                        'robot_ip:=172.16.0.2', "load_gripper:=true", "robot:=fr3"]
        elif type == Type.FORCE:
            cli_args = ['/home/sascha/catkin_ws/src/franka_ros/franka_example_controllers/launch/force_example_controller.launch',
                        'robot_ip:=172.16.0.2', "load_gripper:=true", "robot:=fr3"]
        elif type == Type.VELOCITY:
            cli_args = ['/home/sascha/catkin_ws/src/franka_ros/franka_example_controllers/launch/joint_velocity_controller.launch',
                        'robot_ip:=172.16.0.2', "load_gripper:=true", "robot:=fr3"]

        roslaunch_args = cli_args[1:]
        roslaunch_file = [
            (roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        self.controller = roslaunch.parent.ROSLaunchParent(
            self.uuid, roslaunch_file)

        self.controller.start()
        rospy.sleep(2)
        if type == Type.CARTESIAN:
            rospy.loginfo("started cartesian controller")
        elif type == Type.JOINT:
            rospy.loginfo("started joint controller")
        elif type == Type.FORCE:
            rospy.loginfo("started force controller")
        elif type == Type.VELOCITY:
            rospy.loginfo("started velocity controller")

    def stop(self):
        try:
            self.controller.shutdown()
        except:
            pass
        rospy.logerr("shutdown")
        rospy.sleep(2)


def start_cart(req):
    global type
    rospy.loginfo("starting cartesian controller")
    type = Type.CARTESIAN
    return StartControllerResponse(success=True)


def start_joint(req):
    global type
    rospy.loginfo("starting joint controller")
    type = Type.JOINT
    return StartControllerResponse(success=True)


def start_force(req):
    global type
    rospy.loginfo("starting force controller")
    type = Type.FORCE
    return StartControllerResponse(success=True)


def start_velocity(req):
    global type
    rospy.loginfo("starting velocity controller")
    type = Type.VELOCITY
    return StartControllerResponse(success=True)


def stop_controller(req):
    global type
    rospy.loginfo("stopping controller")
    type = Type.STOP
    return StopControllerResponse(success=True)


if __name__ == '__main__':
    rospy.init_node('controller_service', anonymous=True)
    rospy.loginfo("started controller service")

    start_cart_service = rospy.Service(
        'start_cart', StartController, start_cart)
    start_joint_service = rospy.Service(
        'start_joint', StartController, start_joint)
    start_force_service = rospy.Service(
        'start_force', StartController, start_force)
    start_velocity_service = rospy.Service(
        'start_velocity', StartController, start_velocity)
    stop_service = rospy.Service(
        'stop_controller', StopController, stop_controller)

    node = ControllerNode()
    rate = rospy.Rate(20)  # 2hz
    while not rospy.is_shutdown():
        # print(type)
        if type == Type.CARTESIAN or type == Type.JOINT or type == Type.FORCE or type == Type.VELOCITY:
            print(type)
            node.start(type)
            type = Type.NONE
        elif type == Type.NONE:
            rate.sleep()
            continue
        else:
            print(type)
            node.stop()
            type = Type.NONE

    rospy.spin()

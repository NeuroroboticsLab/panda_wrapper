#!/usr/bin/env python

import threading
from enum import Enum

import rospy
import roslaunch

from move_robot.srv import *


class Type(Enum):
    NONE = 0
    CARTESIAN = 1
    JOINT = 2
    FORCE = 3
    STOP = 4


global controller
global type
type = Type.NONE


def start_controller(type):
    global controller
    print("launching controller")

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    if type == Type.CARTESIAN:
        cli_args = ['/home/sascha/catkin_ws/src/franka_ros/franka_example_controllers/launch/cartesian_impedance_controller.launch',
                    'robot_ip:=172.16.0.2', "load_gripper:=true", "robot:=fr3"]
    elif type == Type.JOINT:
        cli_args = ['/home/sascha/catkin_ws/src/franka_ros/franka_example_controllers/launch/joint_position_controller.launch',
                    'robot_ip:=172.16.0.2', "load_gripper:=true", "robot:=fr3"]
    elif type == Type.FORCE:
        cli_args = ['/home/sascha/catkin_ws/src/franka_ros/franka_example_controllers/launch/force_example_controller.launch',
                    'robot_ip:=172.16.0.2', "load_gripper:=true", "robot:=fr3"]

    roslaunch_args = cli_args[1:]
    roslaunch_file = [
        (roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    cart_controller = roslaunch.parent.ROSLaunchParent(
        uuid, roslaunch_file)

    cart_controller.start()
    rospy.sleep(2)
    rospy.loginfo("started controller")


def stop():
    global controller
    try:
        controller.shutdown()
    except:
        pass
    rospy.loginfo("shutdown")


def start_cart(req):
    global type
    rospy.loginfo("starting controller")
    type = Type.CARTESIAN
    return StartControllerResponse(success=True)


def start_joint(req):
    global type
    rospy.loginfo("starting controller")
    type = Type.JOINT
    return StartControllerResponse(success=True)


def start_force(req):
    global type
    rospy.loginfo("starting controller")
    type = Type.FORCE
    return StartControllerResponse(success=True)


def stop_controller(req):
    global type
    rospy.loginfo("stoping controller")
    type = Type.STOP
    res = StopControllerResponse(success=True)
    return res


if __name__ == '__main__':
    rospy.init_node('controller_service', anonymous=True)
    rospy.loginfo("started controller service")

    start_cart_service = rospy.Service(
        'start_cart', StartController, start_cart)
    start_joint_service = rospy.Service(
        'start_joint', StartController, start_joint)
    start_force_service = rospy.Service(
        'start_force', StartController, start_force)
    stop_service = rospy.Service(
        'stop_controller', StopController, stop_controller)

    rate = rospy.Rate(2)  # 2hz
    while not rospy.is_shutdown():
        # print(type)
        if type == Type.CARTESIAN or type == Type.JOINT or type == Type.FORCE:
            start_controller(type)
        elif type == Type.STOP:
            stop()

        type = Type.NONE
        rate.sleep()

    rospy.spin()

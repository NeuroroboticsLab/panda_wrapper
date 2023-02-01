#!/usr/bin/env python

import rospy

import franka_gripper.msg
import franka_gripper
import actionlib


def homing():
    homingClient = actionlib.SimpleActionClient(
        '/franka_gripper/homing', franka_gripper.msg.HomingAction)

    homingClient.wait_for_server()
    homingClient.send_goal(franka_gripper.msg.HomingGoal())
    homingClient.wait_for_result()
    print("Homing result: ", homingClient.get_result())


def grasp():
    graspClient = actionlib.SimpleActionClient(
        '/franka_gripper/grasp', franka_gripper.msg.GraspAction)
    graspClient.wait_for_server()

    goal = franka_gripper.msg.GraspGoal()
    goal.width = 0.1
    goal.speed = 0.1
    goal.force = 40.0
    graspClient.send_goal(goal)
    graspClient.wait_for_result()
    print("Grasp result: ", graspClient.get_result())


def moving():
    movingClient = actionlib.SimpleActionClient(
        '/franka_gripper/move', franka_gripper.msg.MoveAction)
    movingClient.wait_for_server()

    goal = franka_gripper.msg.MoveGoal()
    goal.width = 0.01
    goal.speed = 0.1
    movingClient.send_goal(goal)
    movingClient.wait_for_result()
    print("Moving result: ", movingClient.get_result())


def stop():
    stopClient = actionlib.SimpleActionClient(
        '/franka_gripper/stop', franka_gripper.msg.StopAction)
    stopClient.wait_for_server()

    stopClient.send_goal(franka_gripper.msg.StopAction())
    stopClient.wait_for_result()
    print("Stoping result: ", stopClient.get_result())


if __name__ == '__main__':
    rospy.init_node('gripper', anonymous=True)

    try:
        grasp()
        homing()
        moving()
        stop()

    except rospy.ROSInterruptException:
        pass

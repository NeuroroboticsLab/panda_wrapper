#!/usr/bin/env python

import threading
import curses

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import tf2_ros

from move_robot.srv import *
from panda import RobotCart, Gripper


def keyboard_input(robot, gripper):
    print("Keyboard input started")
    screen = curses.initscr()
    screen.keypad(True)

    while not rospy.is_shutdown():
        key = curses.initscr().getch()
        # print(key)
        translation_offset = 0.005
        orientation_offset = 0.01

        # Translation
        if key == curses.KEY_UP:
            robot.pose.pose.position.z += translation_offset
            print("Up")
        elif key == curses.KEY_DOWN:
            robot.pose.pose.position.z -= translation_offset
            print("Down")
        elif key == 52:  # Num 4
            robot.pose.pose.position.y += translation_offset
            print("Left")
        elif key == 54:  # Num 6
            robot.pose.pose.position.y -= translation_offset
            print("Right")
        elif key == 56:  # Num 8
            robot.pose.pose.position.x += translation_offset
            print("Forward")
        elif key == 50:  # Num 2
            robot.pose.pose.position.x -= translation_offset
            print("Backward")
        # Orientation
        elif key == 113:  # Q
            robot.pose.pose.orientation.x += orientation_offset
            print("Roll left")
        elif key == 101:  # E
            robot.pose.pose.orientation.x -= orientation_offset
            print("Roll right")
        elif key == 97:  # A
            robot.pose.pose.orientation.y += orientation_offset
            print("Pitch down")
        elif key == 100:  # D
            robot.pose.pose.orientation.y -= orientation_offset
            print("Pitch up")
        elif key == 119:  # W
            robot.pose.pose.orientation.z += orientation_offset
            print("Yaw left")
        elif key == 115:  # S
            robot.pose.pose.orientation.z -= orientation_offset
            print("Yaw right")
        # Gripper
        elif key == 99:  # C
            gripper.grasp(0.006, 0.1)
            print("Grasp")
        elif key == 111:  # O
            gripper.homing()
            print("Homing")
        elif key == 27:  # ESC
            print("Quit")
            return
        else:
            print("Not an saved key!")


if __name__ == '__main__':
    rospy.init_node('robot_cart')

    rospy.wait_for_service('/start_force')
    start_cart_service = rospy.ServiceProxy('/start_cart', StartController)
    response = start_cart_service(StartControllerRequest())
    rospy.sleep(5)

    robot = RobotCart()
    gripper = Gripper()
    threading.Thread(target=keyboard_input, args=(robot, gripper, )).start()

    rospy.spin()

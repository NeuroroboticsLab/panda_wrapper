#!/usr/bin/env python

import threading
import curses

import rospy
import roslaunch
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from panda import *

from move_robot.srv import *


def keyboard_input(robot, gripper):
    print("Keyboard input started")
    screen = curses.initscr()
    screen.keypad(True)

    while not rospy.is_shutdown():
        key = curses.initscr().getch()
        print(key)
        offset = 0.005

        if key == curses.KEY_UP:
            robot.pose.pose.position.z += offset
            print("Up")
        elif key == curses.KEY_DOWN:
            robot.pose.pose.position.z -= offset
            print("Down")
        elif key == 52:  # Num 4
            robot.pose.pose.position.y += offset
            print("Left")
        elif key == 54:  # Num 6
            robot.pose.pose.position.y -= offset
            print("Right")
        elif key == 56:  # Num 8
            robot.pose.pose.position.x += offset
            print("Forward")
        elif key == 50:  # Num 2
            robot.pose.pose.position.x -= offset
            print("Backward")
        elif key == 99:  # cc
            gripper.grasp(0.01, 0.1)
            print("Grasp")
        elif key == 111:  # o
            gripper.homing()
            print("Homing")
        elif key == 113:  # q
            print("Quit")
            return
        else:
            print("Not an saved key!")


if __name__ == '__main__':
    rospy.init_node('caller')

    rospy.wait_for_service('/start_cart')
    start_cart_service = rospy.ServiceProxy('/start_cart', StartController)
    response = start_cart_service(StartControllerRequest())
    rospy.sleep(5)

    robot = RobotCart()
    gripper = Gripper()
    print("Robot initialized")
    keyThread = threading.Thread(
        target=keyboard_input, args=(robot, gripper,)).start()

    rospy.spin()

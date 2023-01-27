#!/usr/bin/env python

import threading
import curses

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import tf2_ros

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
        rospy.sleep(0.1)

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
            rospy.loginfo(self.pose)
            self.rate.sleep()


def keyboard_input(robot):
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
        elif key == 52:
            robot.pose.pose.position.y += offset
            print("Left")
        elif key == 54:
            robot.pose.pose.position.y -= offset
            print("Right")
        elif key == 56:
            robot.pose.pose.position.x += offset
            print("Forward")
        elif key == 50:
            robot.pose.pose.position.x -= offset
            print("Backward")
        elif key == 113:
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
    threading.Thread(target=keyboard_input, args=(robot,)).start

    rospy.spin()

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

from move_robot.srv import *
from panda import RobotJoint


def talker():
    pub = rospy.Publisher('/desired_joints', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(2)  # 2hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        state = JointState()
        state.name = ['1', '2', '3', '4', '5', '6', '7']
        state.position = [0, -0.785398163397, 0, -
                          2.35619449019, 0, 1.57079632679, 0.785398163397]
        pub.publish(state)
        rospy.loginfo(state)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('example_velocity', anonymous=True)

    rospy.wait_for_service('/start_velocity')
    start_velocity_service = rospy.ServiceProxy(
        '/start_position', StartController)
    stop_controller_service = rospy.ServiceProxy(
        '/stop_controller', StopController)
    print(start_velocity_service(StartControllerRequest()))
    rospy.sleep(5)

    robot = RobotJoint()
    state = JointState()
    state.name = ['1', '2', '3', '4', '5', '6', '7']
    state.position = [0, -0.785398163397, 0, -
                      2.35619449019, 0, 1.57079632679, 0.785398163397]
    robot.set_target(state)

    rospy.sleep(5)
    print(stop_controller_service(StopControllerRequest()))

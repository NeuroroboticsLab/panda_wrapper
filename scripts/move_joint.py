#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from move_robot.srv import *


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
    rospy.wait_for_service('/start_joint')
    start_joint_service = rospy.ServiceProxy('/start_cart', StartController)
    response = start_joint_service(StartControllerRequest())
    rospy.sleep(5)

    try:
        talker()
    except rospy.ROSInterruptException:
        pass

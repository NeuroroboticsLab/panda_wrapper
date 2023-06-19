#!/usr/bin/env python

import rospy

from panda_wrapper.srv import *
from panda import StateViewer


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.wait_for_service('/start_force')
    start_force_service = rospy.ServiceProxy('/start_force', StartController)
    response = start_force_service(StartControllerRequest())
    rospy.sleep(5)

    viewer = StateViewer()
    rate = rospy.Rate(10, False)  # 2hz
    while not rospy.is_shutdown():
        # input("Press Enter to save state")
        viewer.print_state()
        viewer.save_state()
        rate.sleep()

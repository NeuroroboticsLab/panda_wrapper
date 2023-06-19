#!/usr/bin/env python

import rospy

from panda_wrapper.srv import *
from panda import Gripper


if __name__ == '__main__':
    rospy.init_node('example_gripper', anonymous=True)

    rospy.wait_for_service('/start_force')
    start_joint_service = rospy.ServiceProxy(
        '/start_force', StartController)
    stop_controller_service = rospy.ServiceProxy(
        '/stop_controller', StopController)
    print(start_joint_service(StartControllerRequest()))
    rospy.sleep(5)

    gripper = Gripper()

    gripper.grasp(0, 1)
    gripper.homing()
    gripper.move(0.2)
    gripper.stop()

    print(stop_controller_service(StopControllerRequest()))

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from panda import *
from move_robot.srv import *


if __name__ == '__main__':
    rospy.init_node('caller')
    rospy.wait_for_service('/start_velocity')
    rospy.wait_for_service('/stop_controller')
    print("Services found")

    pose_folder = "/home/sascha/catkin_ws/data/poses/"
    [home_j, t] = load_state(path=pose_folder+"home.json")
    [push_inter_j, t] = load_state(path=pose_folder+"intermediate_push.json")
    [push_j, t] = load_state(path=pose_folder+"push.json")
    [cup_inter_1_j, t] = load_state(path=pose_folder+"intermediate_cup_1.json")
    [cup_inter_2_j, t] = load_state(path=pose_folder+"intermediate_cup_2.json")
    [cup_j, t] = load_state(path=pose_folder+"grasp_cup.json")
    [drink_j, t] = load_state(path=pose_folder+"drink.json")
    [drink_2_j, t] = load_state(path=pose_folder+"drink_2.json")

    # start joint controller
    start_vel_service = rospy.ServiceProxy('/start_velocity', StartController)
    response = start_vel_service(StartControllerRequest())
    rospy.sleep(1)
    gripper = Gripper()
    robotJoint = RobotJoint()

    print("move to home position")
    robotJoint.set_target(home_j)
    rospy.sleep(1)  # 3
    gripper.homing()

    print("move to intermediate push position")
    robotJoint.set_target(push_inter_j)
    rospy.sleep(2)
    gripper.grasp(0.001, 0.1, 50)

    print("move to push position")
    robotJoint.set_target(push_j)
    rospy.sleep(5)

    print("move to intermediate push position")
    robotJoint.set_target(push_inter_j)
    rospy.sleep(3)
    gripper.grasp(0.001, 0.1, 50)

    print("move to intermediate cup position")
    robotJoint.set_target(cup_inter_1_j)
    rospy.sleep(2)  # 3

    print("move to intermediate cup position")
    robotJoint.set_target(cup_inter_2_j)
    rospy.sleep(2)  # 3
    gripper.move(0.02, 0.1)

    print("move to cup position")
    robotJoint.set_target(cup_j)
    rospy.sleep(5)  # 3
    gripper.grasp(0.0075, 0.1, 50)
    rospy.sleep(0.1)

    # print("move to grasp position")
    # [joints, tcp] = load_state(path=pose_folder+"grasp.json")
    # robotJoint.set_target(joints)
    # rospy.sleep(6)
    # robotJoint.stop()
    # rospy.sleep(0.2)
    # # gripper.grasp(0.02, 0.1, 50)
    # gripper.grasp(0.0075, 0.1, 50)
    # rospy.sleep(0.1)

    print("move to drink position")

    robotJoint.set_target(drink_j)
    rospy.sleep(4)
    robotJoint.set_target(drink_2_j)
    rospy.sleep(2.5)
    robotJoint.set_target(drink_j)
    rospy.sleep(2.5)
    gripper.homing()
    robotJoint.stop()
    rospy.sleep(1)

    print("stop joint controller")
    stop_controller_service = rospy.ServiceProxy(
        '/stop_controller', StopController)
    rospy.sleep(3)

    # # let the robot drink
    # # start cartesian controller
    # start_cart_service = rospy.ServiceProxy('/start_cart', StartController)
    # response = start_cart_service(StartControllerRequest())
    # rospy.sleep(5)
    # robot = RobotCart()
    # print("Robot initialized")

    # # let the user control the robot with the keyboard
    # keyThread = threading.Thread(
    #     target=keyboard_input, args=(robot, gripper,)).start()

    # rospy.spin()

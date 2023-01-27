#!/usr/bin/env python


import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros
from std_msgs.msg import Float64
import json
from move_robot.srv import *

joint_state = JointState()


def joint_state_callback(data):
    global joint_state
    joint_state = data


def talker():
    global joint_state

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(0.1)

    rate = rospy.Rate(10)  # 2hz
    while not rospy.is_shutdown():
        tcp = tf_buffer.lookup_transform('fr3_link0', 'fr3_EE', rospy.Time())
        print("Translation: ", tcp.transform.translation)

        print("Rotation: ", tcp.transform.rotation)
        print("Joint State: ", joint_state.position)
        print("\n \n \n")
        translation = (tcp.transform.translation.x, tcp.transform.translation.y,
                       tcp.transform.translation.z)
        quat = (tcp.transform.rotation.x, tcp.transform.rotation.y,
                tcp.transform.rotation.z, tcp.transform.rotation.w)

        json_dict = {'translation': translation,
                     'rotation': quat,
                     'joint_states': joint_state.position,
                     'joint_names': joint_state.name,
                     'info': 'Translation and Rotation of the TCP and the joint state of the robot. Rotation as x,y,z,w'}
        json_object = json.dumps(json_dict, indent=4, sort_keys=True)
        with open("/home/sascha/robot_data.json", "w") as outfile:
            outfile.write(json_object)

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.wait_for_service('/start_force')
    start_force_service = rospy.ServiceProxy('/start_force', StartController)
    response = start_force_service(StartControllerRequest())
    rospy.sleep(5)

    sub = rospy.Subscriber("/joint_states", JointState, joint_state_callback)

    try:
        talker()
    except rospy.ROSInterruptException:
        pass

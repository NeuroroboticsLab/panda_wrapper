#!/usr/bin/env python

import rospy
import numpy as np
import cv2

from move_robot.srv import *
from panda import StateViewer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Viewer:
    def __init__(self):
        self.viewer = StateViewer()
        self.path = "/home/sascha/catkin_ws/data/"
        self.count = 0
        self.load_count()
        self.image = None
        self.bridge = CvBridge()

        sub_image = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_callback, queue_size=1)

    def save_count(self):
        f = open(self.path + "count.txt", "w")
        f.write(str(self.count))

    def load_count(self):
        f = open(self.path + "count.txt", "r")
        count = int(f.read())
        print("count:", count)
        if (count < 0):
            self.count = 0
        else:
            self.count = int(count)

    def save(self):
        file = self.path + str(self.count)
        self.viewer.save_state_cv(path=file + '.xml')
        cv2.imwrite(file + '.png', self.image)
        self.count += 1
        self.save_count()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg=msg)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.image = cv_image.copy()
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.wait_for_service('/start_force')
    start_force_service = rospy.ServiceProxy('/start_force', StartController)
    response = start_force_service(StartControllerRequest())
    rospy.sleep(5)

    viewer = Viewer()
    rate = rospy.Rate(10, False)  # 2hz

    while not rospy.is_shutdown():
        # input("Press Enter to save state")
        # viewer.print_state()
        # viewer.save_state()
        # viewer.save_state_cv()
        print('Enter:')
        x = input()
        if (x == 's'):
            viewer.save()
        elif (x == 'q'):
            break

        rate.sleep()

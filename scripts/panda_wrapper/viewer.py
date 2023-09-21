#!/usr/bin/env python

import os
import rospy
import numpy as np
import cv2 as cv
import threading

from panda_wrapper.srv import *
from panda import StateViewer
import pyrealsense2 as rs
import franka_msgs.srv as frankaMsg


SAVE_POINTCLOUD = False


class Viewer:
    def __init__(self):
        self.lock = threading.Lock()
        self.viewer = StateViewer()
        self.path = "/home/sascha/catkin_ws/data/"
        self.count = 0
        self.load_count()
        self.depth_frame = None
        self.color_frame = None
        self.color_image = None
        self.exit_thread = False

        self.init_rs()
        thread = threading.Thread(target=self.rs_thread)
        thread.start()

    def init_rs(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The app requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.pipeline.start(config)

        self.pc = rs.pointcloud()
        self.decimate = rs.decimation_filter()
        self.decimate.set_option(rs.option.filter_magnitude, 2)
        self.decimation_filter = rs.decimation_filter()
        self.spatial_filter = rs.spatial_filter()
        self.temporal_filter = rs.temporal_filter()

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

    def display(self, img, window_name="default", destroyable=True):
        img = cv.resize(img, (1280, 720))
        cv.imshow(window_name, img)
        cv.waitKey(1)

    def save(self):
        file = self.path + str(self.count)
        self.viewer.save_state(path=file + '.xml')

        self.lock.acquire()
        cv.imwrite(file + '.png', self.color_image)
        if SAVE_POINTCLOUD:
            self.pc.map_to(self.color_frame)
            points = self.pc.calculate(self.depth_frame)
            points.export_to_ply(file + '.ply', self.color_frame)
        self.lock.release()

        self.count += 1
        self.save_count()

    def rs_thread(self):
        while self.exit_thread is False:
            frames = self.pipeline.wait_for_frames()
            if SAVE_POINTCLOUD:
                depth_frame = frames.get_depth_frame()
                depth_frame = self.decimate.process(depth_frame)
                depth_frame = self.spatial_filter.process(depth_frame)
                depth_frame = self.temporal_filter.process(depth_frame)
            color_frame = frames.get_color_frame()

            color_image = np.asanyarray(color_frame.get_data())
            color_image = cv.cvtColor(color_image, cv.COLOR_RGB2BGR)
            self.display(color_image, "live stream", True)

            self.lock.acquire()
            if SAVE_POINTCLOUD:
                self.depth_frame = depth_frame
            self.color_frame = color_frame
            self.color_image = color_image.copy()
            self.lock.release()


def set_load():
    service = "/franka_control/set_load"
    rospy.wait_for_service(service)

    objectMass = 0.04
    loadSet = frankaMsg.SetLoadRequest()
    loadSet.mass = objectMass
    loadSet.F_x_center_load = [0.06, 0.0001, 0.01]
    loadSet.load_inertia = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    try:
        set_load_service = rospy.ServiceProxy(service, frankaMsg.SetLoad)
        response = set_load_service.call(loadSet)
        print(response.success)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        print(response.error)


if __name__ == '__main__':
    np.set_printoptions(precision=3)
    rospy.init_node('viewer', anonymous=True)

    # set_load()

    viewer = Viewer()
    rate = rospy.Rate(20, False)

    while not rospy.is_shutdown():
        # input("Press Enter to save state")
        # viewer.print_state()
        # viewer.save_state()
        print('Enter (s) to save | (q) to quit | (r) to reset counter:')
        x = input()
        if (x == 's'):
            viewer.save()
        elif (x == 'r'):
            viewer.count = 0
            viewer.save_count()
            viewer.load_count()
        elif (x == 'q'):
            viewer.exit_thread = True
            break

        rate.sleep()

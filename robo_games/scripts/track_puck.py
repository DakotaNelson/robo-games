#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
from math import exp, sqrt
from sensor_msgs.msg import Image
from robo_games.msg import PuckCameraLocation as PuckPosition
from cv_bridge import CvBridge, CvBridgeError

class ColorTracker:
    def __init__(self, cameraTopic):
        rospy.init_node('puck_tracker')

        self.img_sub = rospy.Subscriber(cameraTopic, Image, self.process_frame, queue_size=1)
        self.puck_pub = rospy.Publisher("puck_camera_position", PuckPosition, queue_size=1)
        #self.capture = cv.VideoCapture('red_puck.mp4')
        self.bridge = CvBridge()

        self.count = 0
        self.max_count = 4
        lowpass_buffer_size = 5
        self.offsetQueue = [0 for _ in range(lowpass_buffer_size)]
        self.distanceQueue = [0 for _ in range(lowpass_buffer_size)]

    def process_frame(self, frame):
        # only send over every 4th frame
        self.count += 1
        if not self.count % self.max_count == 0:
            return
        # first, convert image from ROS to OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        except CvBridgeError, e:
            print(e)
            return

        # consider blurring image a bit

        #hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        thresh = cv.inRange(frame, np.array((0, 0, 130)), np.array((130, 130, 255)))

        # erosion and dilation to remove small particles/noise
        kernel = np.ones((5,5), np.uint8)
        thresh = cv.erode(thresh, kernel, iterations=8)
        thresh = cv.dilate(thresh, kernel, iterations=8)

        thresh2 = thresh.copy()

        contours, hierarchy = cv.findContours(thresh, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

        max_area = 0
        best_contour = False
        for contour in contours:
            area = cv.contourArea(contour)
            if area > max_area:
                max_area = area
                best_contour = contour

        try:
            moments = cv.moments(best_contour)
        except UnboundLocalError:
            print("no contour found")
            return

        returnNow = False
        try:
            cx, cy = int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])
        except ZeroDivisionError:
            print("can't divide by 0")
            cx = 0
            cy = 0
            msg = PuckPosition(puck_offset=-1, puck_distance=-1)
            self.puck_pub.publish(msg)
            returnNow = True

        if not returnNow:
            cv.circle(frame, (cx,cy), 5, 255, -1)
        cv.imshow('frame', frame)

        if returnNow:
            cv.waitKey(25)
            return

        cv.circle(thresh2, (cx,cy), 5, 255, -1)
        cv.imshow('thresh', thresh2)
        if cv.waitKey(25) == 27:
            return

        height, width, channels = frame.shape
        #print(max_area)
        #print(width)
        #print(cx)

        self.offsetQueue = self.offsetQueue[1:]
        offset = float(cx) / width
        self.offsetQueue.append(offset)

        offset = sum(self.offsetQueue) / 5.0

        def distance_fit(y):
            params = [4.18084598e+03, -1.20346945e+00]
            c = params[0] * (1/sqrt(y))
            d = params[1]
            return c+d

        self.distanceQueue = self.distanceQueue[1:]
        dist = distance_fit(max_area)
        self.distanceQueue.append(dist)

        dist = sum(self.distanceQueue) / 5.0

        msg = PuckPosition(puck_offset=offset, puck_distance=dist)
        self.puck_pub.publish(msg)

    def run(self):
        cv.namedWindow('frame')
        cv.namedWindow('thresh')

        # Uncomment the following to capture from a file
        # while(True):
        #     _, frame = self.capture.read()
        #     self.process_frame(frame)
        #     if cv.waitKey(33) == 27:
        #         break

        rospy.spin()

if __name__ == "__main__":
    ct = ColorTracker('camera/image_raw')
    ct.run()

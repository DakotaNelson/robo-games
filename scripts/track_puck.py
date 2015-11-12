#!/usr/bin/env python

import cv2 as cv
import numpy as np
from time import sleep

class ColorTracker:
    def __init__(self):
        self.capture = cv.VideoCapture('video-1447095222.mp4.mp4')

    def process_frame(self, frame):
        # consider blurring image a bit

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        thresh = cv.inRange(hsv, np.array((50, 50, 110)), np.array((100, 200, 255)))
        thresh2 = thresh.copy()

        # consider erosion and dilation to remove small particles/noise

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

        try:
            cx, cy = int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])
        except ZeroDivisionError:
            print("can't divide by 0")
            return

        cv.circle(frame, (cx,cy), 5, 255, -1)

        cv.imshow('frame', frame)
        cv.imshow('thresh', thresh2)

    def run(self):
        cv.namedWindow('frame')
        cv.namedWindow('thresh')
        while(True):
            _, frame = self.capture.read()
            self.process_frame(frame)
            if cv.waitKey(33) == 27:
                break

if __name__ == "__main__":
    ct = ColorTracker()
    ct.run()

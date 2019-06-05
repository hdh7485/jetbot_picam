#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
import imutils
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cam_pi", Image, self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

            (rows,cols,channels) = cv_image.shape
            if cols > 60 and rows > 60 :
                cv2.circle(cv_image, (50,50), 10, 255)

            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)

        try:
            small_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5)
            hsv = cv2.cvtColor(small_image, cv2.COLOR_BGR2HSV)

            lower_blue = np.array([100, 130, 130])
            upper_blue = np.array([150, 255, 255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

            # only proceed if at least one contour was found
            if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                    cv2.circle(mask, (int(x), int(y)), int(radius), 100, 2)
                    cv2.circle(mask, center, 5, 250, -1)
                    x,y,w,h = cv2.boundingRect(c)
                    cv2.rectangle(mask, (x,y), (x+w,y+h), 200, 10)

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
            rospy.loginfo("publish")
        except CvBridgeError as e:
            print(e)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

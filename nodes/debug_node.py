#!/usr/bin/env python

import rospy
import cv2
import sys
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import String

# change this if we're live (on the pi)
_live = False

def _ros2cv(msg):
   try:
      cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
      return cv_image
   except CvBridgeError as e:
      print(e)


def _show_raw(cv_image):
    # create a CLAHE object (Arguments are optional).
   clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
   b, g, r = cv2.split(cv_image)
   clb = clahe.apply(b)
   clg = clahe.apply(g)
   clr = clahe.apply(r)
   img=cv2.merge((clb, clg, clr))
   if not _live:
      cv2.imshow("Control", np.hstack([cv_image, img]))
   # cv2.imshow("Image window", cv_image)


def _nothing(x):
    pass


isFirst = True

def convert_coordinates(x,y,theta, field):
   # This assumes the field position zero is top left
   # field param is (x,y,w,h)
   # may need to rotate angles...
   realW = 3.40
   scaleW = realW / field[2]
   realH = 2.38
   scaleH = realH / field[3]
   rval = Pose2D()
   # subtract half the width/height to make 0,0 at center of field
   rval.x = (x - field[0])*scaleW - realW/2
   rval.y = (y - field[1])*scaleH - realH/2
   rval.theta = theta
   # flip y because image says y is down
   rval.y = -rval.y
   rval.theta = (-rval.theta) % 360
   return rval


def convert_coordinates_back(rval):
   realW = 3.40
   scaleW = 1.0/200 # m/pixel
   realH = 2.38
   scaleH = 1.0/200 # m/pixel
   x = int((realW/2 + rval.x)/scaleW)  # field[0] is 0,0 for our image
   y = int((realH/2 + -rval.y)/scaleH) # field[0] is 0,0 for our image
   theta = (-rval.theta) % 360
   return x,y,theta


def _process_measured_us1(msg):
   pass

def _process_measured_us2(msg):
   pass

def _process_measured_them1(msg):
   pass

def _process_measured_them2(msg):
   pass

def _process_measured_ball(msg):
   pass

def _process_estimated_us1(msg):
   pass

def _process_estimated_us2(msg):
   pass

def _process_estimated_them1(msg):
   pass

def _process_estimated_them2(msg):
   pass

def _process_estimated_ball(msg):
   pass

def _process_commanded_us1(msg):
   pass

def _process_commanded_us2(msg):
   pass

def main():

   # subscribe to locations
   rospy.Subscriber('vision/us1',   Pose2D, _process_measured_us1)
   rospy.Subscriber('vision/us2',   Pose2D, _process_measured_us2)
   rospy.Subscriber('vision/them1', Pose2D, _process_measured_them1)
   rospy.Subscriber('vision/them2', Pose2D, _process_measured_them2)
   rospy.Subscriber('vision/ball',  Pose2D, _process_measured_ball)

   rospy.Subscriber('controller/us1',  Pose2D, _process_estimated_us1)
   rospy.Subscriber('controller/us2',  Pose2D, _process_estimated_us2)
   rospy.Subscriber('estimator/them1', Pose2D, _process_estimated_them1)
   rospy.Subscriber('estimator/them2', Pose2D, _process_estimated_them2)
   rospy.Subscriber('estimator/ball',  Pose2D, _process_estimated_ball)

   rospy.Subscriber('desired_position/us1', Pose2D, _process_commanded_us1)
   rospy.Subscriber('desired_position/us2', Pose2D, _process_commanded_us2)

   cv2.namedWindow("debug")

   
   rospy.spin()
   rate = rospy.Rate(100) # 100 Hz
   while not rospy.is_shutdown():


      # Wait however long it takes to make this tick at 100Hz
      rate.sleep()

if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()

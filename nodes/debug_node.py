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
global image
isFirst = True
us1Vision = Pose2D()
us1Estimated = Pose2D()
us1Commanded = Pose2D()



def _ros2cv(msg):
   try:
      cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
      return cv_image
   except CvBridgeError as e:
      print(e)


def _process_camera(msg):
   global image, isFirst
   image = _ros2cv(msg)
   _draw(image, isFirst)
   isFirst = False


def _draw(image, isFirst):
   if not _live:
      if isFirst:
         cv2.namedWindow("debug")
      us1VisionX,us1VisionY,us1VisionTheta = convert_coordinates_back(us1Vision)
      cv2.putText(image, 'us1V', (us1VisionX,us1VisionY), cv2.FONT_ITALIC, 0.3, (0,0,255))

      us1EstimatedX,us1EstimatedY,us1EstimatedTheta = convert_coordinates_back(us1Estimated)
      cv2.putText(image, 'us1E', (us1EstimatedX,us1EstimatedY), cv2.FONT_ITALIC, 0.3, (255,0,0))

      us1CommandedX,us1CommandedY,us1CommandedTheta = convert_coordinates_back(us1Commanded)
      cv2.putText(image, 'us1C', (us1CommandedX,us1CommandedY), cv2.FONT_ITALIC, 0.3, (0,255,255))

      cv2.imshow("debug",image)
      cv2.waitKey(3)


def _nothing(x):
    pass



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
   global us1Vision 
   us1Vision = msg
   

def _process_measured_us2(msg):
   global image
   pass

def _process_measured_them1(msg):
   global image
   pass

def _process_measured_them2(msg):
   global image
   pass

def _process_measured_ball(msg):
   global image
   pass

def _process_estimated_us1(msg):
   global us1Estimated
   us1Estimated = msg

def _process_estimated_us2(msg):
   global image
   pass

def _process_estimated_them1(msg):
   global image
   pass

def _process_estimated_them2(msg):
   global image
   pass

def _process_estimated_ball(msg):
   global image
   pass

def _process_commanded_us1(msg):
   global us1Commanded
   us1Commanded = msg

def _process_commanded_us2(msg):
   global image
   pass

def main():
   rospy.init_node('debug', anonymous=False)

   #get camera to overlay images on
   rospy.Subscriber('camera', Image, _process_camera)

   # subscribe to locations
   rospy.Subscriber('vision/us1',   Pose2D, _process_measured_us1)
   # rospy.Subscriber('vision/us2',   Pose2D, _process_measured_us2)
   # rospy.Subscriber('vision/them1', Pose2D, _process_measured_them1)
   # rospy.Subscriber('vision/them2', Pose2D, _process_measured_them2)
   # rospy.Subscriber('vision/ball',  Pose2D, _process_measured_ball)

   rospy.Subscriber('controller/us1',  Pose2D, _process_estimated_us1)
   # rospy.Subscriber('controller/us2',  Pose2D, _process_estimated_us2)
   # rospy.Subscriber('estimator/them1', Pose2D, _process_estimated_them1)
   # rospy.Subscriber('estimator/them2', Pose2D, _process_estimated_them2)
   # rospy.Subscriber('estimator/ball',  Pose2D, _process_estimated_ball)

   rospy.Subscriber('desired_position/us1', Pose2D, _process_commanded_us1)
   # rospy.Subscriber('desired_position/us2', Pose2D, _process_commanded_us2)

   
   rospy.spin()
   rate = rospy.Rate(100) # 100 Hz
   while not rospy.is_shutdown():


      # Wait however long it takes to make this tick at 100Hz
      rate.sleep()

if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()

#!/usr/bin/env python

import rospy
import cv2
import sys
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from soccerref.msg import GameState

# change this if we're live (on the pi)
_live = False
global image
isFirst = True
us1Vision = Pose2D()
us1Estimated = Pose2D()
us1Commanded = Pose2D()
us2Vision = Pose2D()
us2Estimated = Pose2D()
us2Commanded = Pose2D()
ballVision = Pose2D()
field_dim = Pose2D()
field_dim.x = 1 # avoid dividing by zero the first couple of times
field_dim.y = 1
field_pos = Pose2D()


_game_state = GameState()
def _process_game_state(msg):
   global _game_state
   _game_state = msg
   

def _ros2cv(msg):
   try:
      cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
      return cv_image
   except CvBridgeError as e:
      print(e)


def _process_camera(msg):
   global image, isFirst
   array = np.fromstring(msg.data, np.uint8)
   uncompressed_img = cv2.imdecode(array, 1)
   image = uncompressed_img
   # image = _ros2cv(msg)
   image = cv2.resize(image, (0,0), fx=0.7, fy=0.7)
   if _game_state.second_half:
      image = cv2.flip(image, 1)
      image = cv2.flip(image, 0)

   _draw(image, isFirst)
   isFirst = False


def _draw(image, isFirst):
   if not _live:
      if isFirst:
         cv2.namedWindow("debug")

      us1VisionX,us1VisionY,us1VisionTheta = convert_coordinates_back(us1Vision)
      cv2.putText(image, 'us1V', (us1VisionX,us1VisionY), cv2.FONT_ITALIC, 0.3,
                  (0,0,255))

      us1EstimatedX,us1EstimatedY,us1EstimatedTheta = convert_coordinates_back(
         us1Estimated)
      cv2.putText(image, 'us1E', (us1EstimatedX,us1EstimatedY), cv2.FONT_ITALIC,
                  0.3, (255,0,0))

      us1CommandedX,us1CommandedY,us1CommandedTheta = convert_coordinates_back(
         us1Commanded)
      cv2.putText(image, 'us1C', (us1CommandedX,us1CommandedY), cv2.FONT_ITALIC,
                  0.3, (0,255,255))

      us2VisionX,us2VisionY,us2VisionTheta = convert_coordinates_back(us2Vision)
      cv2.putText(image, 'us2V', (us2VisionX,us2VisionY), cv2.FONT_ITALIC, 0.3,
                  (0,0,255))

      us2EstimatedX,us2EstimatedY,us2EstimatedTheta = convert_coordinates_back(
         us2Estimated)
      cv2.putText(image, 'us2E', (us2EstimatedX,us2EstimatedY), cv2.FONT_ITALIC,
                  0.3, (255,0,0))

      us2CommandedX,us2CommandedY,us2CommandedTheta = convert_coordinates_back(
         us2Commanded)
      cv2.putText(image, 'us2C', (us2CommandedX,us2CommandedY), cv2.FONT_ITALIC,
                  0.3, (0,255,255))

      ballVisionX,ballVisionY,ballVisionTheta = convert_coordinates_back(ballVision)
      cv2.putText(image, 'ballV', (ballVisionX,ballVisionY), cv2.FONT_ITALIC,
                  0.3, (203, 192, 255))

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
   scaleW = realW / field_dim.x # m/pixel
   # scaleW = 1.0/200
   realH = 2.38
   scaleH = realH / field_dim.y # m/pixel
   # scaleH = 1.0/200
   x = int((realW/2 + rval.x)/scaleW + field_pos.x)  # field[0] is 0,0 for our image
   # negate y to flip to image coordinates again (y is down)
   y = int((realH/2 + -rval.y)/scaleH + field_pos.y) # field[0] is 0,0 for our image
   x = max(x,0) # make sure we don't get negative values
   y = max(y,0)
   # flip coordinates
   theta = (-rval.theta) % 360
   # print 'x_new = {}, y_new = {}'.format(x, y)
   return x,y,theta


def _process_measured_us1(msg):
   global us1Vision
   us1Vision = msg


def _process_measured_us2(msg):
   global us2Vision
   us2Vision = msg

def _process_measured_them1(msg):
   global image
   pass

def _process_measured_them2(msg):
   global image
   pass

def _process_measured_ball(msg):
   global ballVision
   ballVision = msg

def _process_estimated_us1(msg):
   global us1Estimated
   us1Estimated = msg

def _process_estimated_us2(msg):
   global us2Estimated
   us2Estimated = msg

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
   global us2Commanded
   us2Commanded = msg

def _process_field_dim(msg):
   global field_dim
   field_dim = msg

def _process_field_pos(msg):
   global field_pos
   field_pos = msg

def main():
   rospy.init_node('debug', anonymous=False)

   #get camera to overlay images on
   # rospy.Subscriber('camera', Image, _process_camera)
   rospy.Subscriber('camera', CompressedImage, _process_camera)

   rospy.Subscriber('game_state', GameState, _process_game_state)

   # subscribe to locations
   rospy.Subscriber('vision/us1',   Pose2D, _process_measured_us1)
   rospy.Subscriber('vision/us2',   Pose2D, _process_measured_us2)
   # rospy.Subscriber('vision/them1', Pose2D, _process_measured_them1)
   # rospy.Subscriber('vision/them2', Pose2D, _process_measured_them2)
   rospy.Subscriber('vision/ball',  Pose2D, _process_measured_ball)

   rospy.Subscriber('controller/us1',  Pose2D, _process_estimated_us1)
   rospy.Subscriber('controller/us2',  Pose2D, _process_estimated_us2)
   # rospy.Subscriber('estimator/them1', Pose2D, _process_estimated_them1)
   # rospy.Subscriber('estimator/them2', Pose2D, _process_estimated_them2)
   # rospy.Subscriber('estimator/ball',  Pose2D, _process_estimated_ball)

   rospy.Subscriber('desired_position/us1', Pose2D, _process_commanded_us1)
   rospy.Subscriber('desired_position/us2', Pose2D, _process_commanded_us2)

   rospy.Subscriber('vision/field_dim', Pose2D, _process_field_dim)
   rospy.Subscriber('vision/field_pos', Pose2D, _process_field_pos)


   rospy.spin()
   rate = rospy.Rate(100) # 100 Hz
   while not rospy.is_shutdown():


      # Wait however long it takes to make this tick at 100Hz
      rate.sleep()

if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()

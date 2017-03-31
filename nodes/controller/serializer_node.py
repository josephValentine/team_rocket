#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D

from Serializer import Serializer
import FrameConverter
import numpy as np

_twist = Twist()
def _handle_twist(msg):
    # print 'handle_twist: {}'.format(msg)
    # print msg
    global _twist
    _twist = msg


_me = Pose2D()
def _handle_me(msg):
    global _me
    _me = msg


def main():

   # initialize the node
   rospy.init_node('serializer', anonymous=False)

   # subscribe to the velocities message
   rospy.Subscriber('vel_cmds', Twist, _handle_twist)
   rospy.Subscriber('me', Pose2D, _handle_me)

   # create a serializer object
   ser = Serializer()
   #ser.setPID(0, 1.5, 3, 100000)
   ser.setPID(1, 1.6, 3, 100000)
   ser.setPID(2, 1.5, 3, 100000)
   ser.setPID(3, 1.4, 3, 100000)

   # loop until shutdown
   rate = rospy.Rate(100) # 100 Hz

   cntr = 0
   while not rospy.is_shutdown():

      # get body frame speeds
      vx_w, vy_w = _twist.linear.x, _twist.linear.y
      curAngle   = _me.theta
      # vx_b, vy_b = FrameConverter._convert_world_to_body_velocities(
      #    vx_w, vy_w, curAngle)

      # get wheel speeds
      wz = _twist.angular.z
      # w1, w2, w3 = FrameConverter._convert_world_to_motor_velocities(
      #     vx_b, vy_b, wz)
      # w1, w2, w3 = FrameConverter._convert_world_to_motor_velocities(
      #     vx_w, vy_w, curAngle)

      # Try going straight forward at 0.5 m/s
      # vx_w, vy_w, wz, curAngle = 0.5, 0, 0, 0
      w1, w2, w3 = FrameConverter._convert_world_to_motor_velocities(
          vx_w, vy_w, wz, curAngle)

      ser_factor = 1
      w1, w2, w3 = w1*ser_factor, w2*ser_factor, w3*ser_factor

     #Parker Lusks suggestion to reduce effects from stiction
      smallestCommandForMotion=0.99
      negligableCommandThresh=0.1
      if np.abs(w1) < smallestCommandForMotion and np.abs(w1) > negligableCommandThresh:
        w1 = np.sign(w1)*smallestCommandForMotion
      elif np.abs(w1) < negligableCommandThresh:
        w1 = 0

      if np.abs(w2) < smallestCommandForMotion and np.abs(w2) > negligableCommandThresh:
        w2 = np.sign(w2)*smallestCommandForMotion
      elif np.abs(w2) < negligableCommandThresh:
        w2 = 0

      if np.abs(w3) < smallestCommandForMotion and np.abs(w3) > negligableCommandThresh:
        w3 = np.sign(w3)*smallestCommandForMotion
      elif np.abs(w3) < negligableCommandThresh:
        w3 = 0

      if cntr == 20:
          print 'Serializer: w1: {:.3f}\tw2: {:.3f}\tw3: {:.3f}'.format(
              w1, w2, w3)
          cntr = 0
      cntr += 1
      # update the speeds
      ser.set_speed(w1, w2, w3)

      # Wait however long it takes to make this tick at 100Hz
      rate.sleep()

      # time.sleep(1.0/sampleRate)

      # speed = getSpeed()
      # speedsM1.append(speed[0]/pulsePerRotation)
      # speedsM2.append(speed[1]/pulsePerRotation)
      # speedsM3.append(speed[2]/pulsePerRotation)

   # shutdown
   ser.disengage()


if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()

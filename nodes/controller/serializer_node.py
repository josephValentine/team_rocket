#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D

from Serializer import Serializer
import FrameConverter


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
   # rospy.Subscriber('me', Pose2D, _handle_me)

   # create a serializer object
   ser = Serializer()

   # loop until shutdown
   rate = rospy.Rate(100) # 100 Hz
   while not rospy.is_shutdown():

      # get body frame speeds
      vx_w, vy_w = _twist.linear.x, _twist.linear.y
      curAngle   = _me.theta
      vx_b, vy_b = FrameConverter._convert_world_to_body_velocities(
         vx_w, vy_w, curAngle)

      # get wheel speeds
      wz = _twist.angular.z
      # w1, w2, w3 = FrameConverter._convert_world_to_motor_velocities(
      #     vx_b, vy_b, wz)
      # w1, w2, w3 = FrameConverter._convert_world_to_motor_velocities(
      #     vx_w, vy_w, curAngle)
      w1, w2, w3 = FrameConverter._convert_world_to_motor_velocities(
          vx_w, vy_w, wz, curAngle)

      ser_factor = 2
      w1, w2, w3 = w1*ser_factor, w2*ser_factor, w3*ser_factor

      print 'Serializer: w1: {:.3f}\tw2: {:.3f}\tw3: {:.3f}'.format(w1, w2, w3)
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

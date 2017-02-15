#!/usr/bin/env python

import rospy
from x import Velocities

from Serializer import Serializer

_vels = Velocities()
def _handle_vels(msg):
    print "handle_vels"
    global _vels
    _vels = msg


def main():

   # initialize the node
   rospy.init_node('serializer', anonymous=False)

   # subscribe to the velocities message
   rospy.Subscriber('wheels', Velocities, _handle_vels)

   # create a serializer object
   ser = Serializer()

   # loop until shutdown
   rate = rospy.Rate(100) # 100 Hz
   while not rospy.is_shutdown():

      # update the speeds
      ser.set_speed(*_vels)

      # Wait however long it takes to make this tick at 100Hz
      rate.sleep()
      # time.sleep(1.0/sampleRate)

      # speed = getSpeed()
      # speedsM1.append(speed[0]/pulsePerRotation)
      # speedsM2.append(speed[1]/pulsePerRotation)
      # speedsM3.append(speed[2]/pulsePerRotation)

   # shutdown
   disengage()


if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()

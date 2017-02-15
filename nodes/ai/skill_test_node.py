#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from Geometry.Models import Position, Point, Angle
# from soccerref.msg import GameState

import numpy as np

from SkillTest import SkillTest

#### Global vars ####

# initialize vision positions
_me = Pose2D()
_ally = Pose2D()
_opp1 = Pose2D()
_opp2 = Pose2D()
_ball = Pose2D()

# -------------------

_tmp_hm_cnt = 0
def _handle_me(msg):
   global _tmp_hm_cnt
   _tmp_hm_cnt = (_tmp_hm_cnt + 1) % 100
   # if _tmp_hm_cnt == 0:
   #    print 'handle_me: {}'.format(msg)
   global _me
   _me = msg


_tmp_hb_cnt = 0
def _handle_ball(msg):
   global _tmp_hb_cnt
   _tmp_hb_cnt = (_tmp_hb_cnt + 1) % 100
   # if _tmp_hb_cnt == 0:
   #    print 'handle_ball: {}'.format(msg)
   global _ball
   _ball = msg


def main():
   rospy.init_node('ai', anonymous = False)

   # Subscribe to Robot and Ball positions
   rospy.Subscriber('me',   Pose2D, _handle_me  )
   rospy.Subscriber('ball', Pose2D, _handle_ball)

   # This is our publisher that tells the controller where we want to be
   pub = rospy.Publisher('desired_position', Pose2D, queue_size=10)

   # Create the SKILL_TEST object
   skill_test = SkillTest()

   rate = rospy.Rate(100) # 100 Hz
   while not rospy.is_shutdown():

      # Based on the state of the game and the positions of the players,
      # run the SKILL_TEST and return commanded positions for this robot
      skill_test.update(_pose2d_to_pos(_me), _pose2d_to_pos(_ball))
      # pos = skill_test.get_commanded_position('spin')
      pos = skill_test.get_commanded_position('move_in_box')
      print 'comamanded position: {}'.format(pos)
      cmds = _pos2cmd(pos)
      print 'commands: {}'.format(cmds)

      # Get a message ready to send
      msg = Pose2D()

      msg.x = cmds[0]
      msg.y = cmds[1]
      msg.theta = cmds[2]

      pub.publish(msg)

      # Wait however long it takes to make this tick at 100Hz
      rate.sleep()


def _pos2cmd(pos):
   # return (pos.point.x, pos.point.y, pos.angle.radian)
   return (pos.point.x, pos.point.y, pos.angle.degree)


def _pose2d_to_pos(pose2d):
   return Position(Point(pose2d.x, pose2d.y), Angle(pose2d.theta, True))


def _pos_to_pose2d(pos):
   return Pose2D(pos.point.x, pos.point.y, pos.angle.degree)


if __name__ == '__main__':
   # If this file was run from the command line, then do the following:
    main()

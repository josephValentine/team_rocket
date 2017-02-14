#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from soccerref.msg import GameState

import numpy as np

from AI import AI

#### Global vars ####

# create a blank GameState message
# to keep track of current state
_game_state = GameState()

# initialize vision positions
_me = Pose2D()
_ally = Pose2D()
_opp1 = Pose2D()
_opp2 = Pose2D()
_ball = Pose2D()

# -------------------

def _handle_me(msg):
   print "handle_me"
   global _me
   _me = msg


def _handle_ball(msg):
   print "handle_ball"
   global _ball
   _ball = msg


def main():
   rospy.init_node('ai', anonymous = False)

    # Subscribe to Robot and Ball positions
    rospy.Subscriber('me',   Pose2D, _handle_me  )
    rospy.Subscriber('ball', Pose2D, _handle_ball)

    # This message comes from the soccerref and
    # tells us if we should be playing or not
    rospy.Subscriber('/game_state', GameState, _handle_game_state)

    # This is our publisher that tells the controller where we want to be
    pub = rospy.Publisher('desired_position', Pose2D, queue_size=10)

    # Create the SKILL_TEST object
    skill_test = SKILL_TEST(_team_side, _ally_number)

    rate = rospy.Rate(100) # 100 Hz
    while not rospy.is_shutdown():

       # Based on the state of the game and the positions of the players,
       # run the SKILL_TEST and return commanded positions for this robot
       skill_test.update(_me, _ball, _game_state)
       pos = skill_test.get_commanded_position('spin')
       cmd = _pos2cmd(pos)

       # Get a message ready to send
       msg = Pose2D()

       msg.x = cmds[0]
       msg.y = cmds[1]
       msg.theta = cmds[2]


       # If we shouldn't play and the field doesn't need to be
       # reset, then the SKILL_TEST node is out of a job.
       if _game_state.play or _game_state.reset_field:
          pub.publish(msg)

       # Wait however long it takes to make this tick at 100Hz
       rate.sleep()


def _pos2cmd(pos):
   return (pos.position.x, pos.position.y, pos.angle.radian)


if __name__ == '__main__':
   # If this file was run from the command line, then do the following:
    main()

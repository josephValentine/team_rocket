#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from soccerref.msg import GameState

import numpy as np

from AI import AI

#### Global vars ####

# which team/robot am I running on?
_team_side = 'home'
_ally_number = 1

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


def _handle_ally(msg):
    print "handle_ally"
    global _ally
    _ally = msg


def _handle_opp1(msg):
    print "handle_opp1"
    global _opp1
    _opp1 = msg


def _handle_opp2(msg):
    print "handle_opp2"
    global _opp2
    _opp2 = msg


def _handle_ball(msg):
    print "handle_ball"
    global _ball
    _ball = msg


def _handle_game_state(msg):
    # print "handle_game_state"
    global _game_state
    _game_state = msg


def main():
    rospy.init_node('ai', anonymous=False)

    # are we home or away?
    global _team_side
    param_name   = rospy.search_param('team_side')
    _team_side   = rospy.get_param(param_name, 'home')
    print 'team_side: {} ({})'.format(param_name, type(param_name))

    # which ally are we?
    global _my_number
    global _ally_number

    param_name   = rospy.search_param('my_number')
    _my_number   = rospy.get_param(param_name, '1')
    print '_my_number = {}'.format(_my_number)
    print 'my_number: {} ({})'.format(param_name, type(param_name))

    param_name   = rospy.search_param('ally_number')
    _ally_number = rospy.get_param(param_name, '2')
    print '_ally_number = {}'.format(_ally_number)
    print 'ally_number: {} ({})'.format(param_name, type(param_name))

    # An exteremely brittle way of getting the robot number
    # Try setting as a rosparam instead?
    # _ally_number = int(rospy.get_namespace().split('/')[-2][-1])

    # # orienter node name
    param_name     = rospy.search_param('orienter_path')
    _orienter_path = rospy.get_param(param_name, 'teamrocket_home/orienter')
    print '_orienter_path = {}'.format(_orienter_path)
    print 'orienter_path: {} ({})'.format(param_name, type(param_name))

    # Subscribe to Robot and Ball positions
    # rospy.Subscriber(_orienter_path + '/me',   Pose2D, _handle_me  )
    # rospy.Subscriber(_orienter_path + '/ally', Pose2D, _handle_ally)
    # rospy.Subscriber(_orienter_path + '/opp1', Pose2D, _handle_opp1)
    # rospy.Subscriber(_orienter_path + '/opp2', Pose2D, _handle_opp2)
    # rospy.Subscriber(_orienter_path + '/ball', Pose2D, _handle_ball)
    rospy.Subscriber('me',   Pose2D, _handle_me  )
    rospy.Subscriber('ally', Pose2D, _handle_ally)
    rospy.Subscriber('opp1', Pose2D, _handle_opp1)
    rospy.Subscriber('opp2', Pose2D, _handle_opp2)
    rospy.Subscriber('ball', Pose2D, _handle_ball)

    # This message comes from the soccerref and
    # tells us if we should be playing or not
    rospy.Subscriber('/game_state', GameState, _handle_game_state)

    # This is our publisher that tells the controller where we want to be
    # desired_position_str = '/ally{}/desired_position'.format(_my_number)
    pub = rospy.Publisher('desired_position', Pose2D, queue_size=10)
    # pub = rospy.Publisher(desired_position_str, Pose2D, queue_size=10)

    # Create the AI object
    # ai = AI(_team_side, _ally_number)
    print 'my_number: {} ({})'.format(_my_number, type(_my_number))
    ai = AI(_team_side, _my_number)

    rate = rospy.Rate(100) # 100 Hz
    while not rospy.is_shutdown():

        # Based on the state of the game and the positions of the players,
        # run the AI and return commanded positions for this robot
        ai.update(_me, _ally, _opp1, _opp2, _ball, _game_state)
        cmds = ai.strategize()

        # Get a message ready to send
        msg = Pose2D()

        if _game_state.reset_field:
            # Send robot to home
            # if _ally_number == 1:
            if _my_number == 1:
                msg.x = -0.5
                msg.y = 0
                msg.theta = 0

            # elif _ally_number == 2:
            elif _my_number == 2:
                msg.x = -1
                msg.y = 0
                msg.theta = 0

        else:
            # Run AI as normal
            msg.x = cmds[0]
            msg.y = cmds[1]
            msg.theta = cmds[2]


        # If we shouldn't play and the field doesn't need to be
        # reset, then the AI node is out of a job.
        if _game_state.play or _game_state.reset_field:
            pub.publish(msg)

        # Wait however long it takes to make this tick at 100Hz
        rate.sleep()



if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()

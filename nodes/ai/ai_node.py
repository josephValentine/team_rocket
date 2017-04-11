#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from soccerref.msg import GameState

import numpy as np

from AI import AI

#### Global vars ####

# which team/robot am I running on?
_team_side = 'home'
_my_number = 1

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
    # print "handle_me"
    global _me
    _me = msg


def _handle_ally(msg):
    # print "handle_ally"
    global _ally
    _ally = msg


def _handle_opp1(msg):
    # print "handle_opp1"
    global _opp1
    _opp1 = msg


def _handle_opp2(msg):
    # print "handle_opp2"
    global _opp2
    _opp2 = msg


def _handle_ball(msg):
    # print "handle_ball"
    global _ball
    _ball = msg


def _handle_game_state(msg):
    # print "handle_game_state"
    global _game_state
    _game_state = msg



def main():
    rospy.init_node('ai', anonymous=False)

    # global
    global _game_state

    # are we home or away?
    global _team_side
    team_param_name = rospy.search_param('team_side')
    _team_side = rospy.get_param(team_param_name, 'home')

    global _ignore_game_state
    ignore_game_state_param_name = rospy.search_param('ignore_game_state')
    _ignore_game_state = rospy.get_param(ignore_game_state_param_name, 'False')
    print 'ignoring game state: {} ({})'.format(_ignore_game_state,
                                                type(_ignore_game_state))

    # which ally are we?
    global _my_number
    global _my_number
    # An exteremely brittle way of getting the robot number
    # Try setting as a rosparam instead?
    my_number_param_name = rospy.search_param('my_number')
    _my_number = rospy.get_param(my_number_param_name, '1')
    my_number_param_name = rospy.search_param('my_number')
    _my_number = rospy.get_param(my_number_param_name, '2')
    print 'team_param_name', team_param_name
    print '_team_side', _team_side
    print 'my_number_param_name', my_number_param_name
    print '_my_number', _my_number
    print 'my_number_param_name', my_number_param_name
    print '_my_number', _my_number
    # _my_number = int(rospy.get_namespace().split('/')[-2][-1])
    # _my_number = '1'

    # Subscribe to Robot and Ball positions
    rospy.Subscriber('me',   Pose2D, _handle_me  )
    # rospy.Subscriber('ally', Pose2D, _handle_ally)
    # rospy.Subscriber('opp1', Pose2D, _handle_opp1)
    # rospy.Subscriber('opp2', Pose2D, _handle_opp2)
    rospy.Subscriber('ball', Pose2D, _handle_ball)

    # This message comes from the soccerref and
    # tells us if we should be playing or not
    # rospy.Subscriber('/game_state', GameState, _handle_game_state)
    # We used to always listen to the game_state in the root namespace but now
    # our launch file remaps us the correct one. In competition this will be
    # /game_state, and in hardware testing it will be
    # /teamrocket_teamside/game_state.
    rospy.Subscriber('game_state', GameState, _handle_game_state)

    # This is our publisher that tells the controller where we want to be
    pub = rospy.Publisher('desired_position', Pose2D, queue_size=10)

    # Create the AI object
    ai = AI(_team_side, _my_number)

    rate = rospy.Rate(100) # 100 Hz
    count = 0
    print_when_commanding = False
    while not rospy.is_shutdown():

        # Get a message ready to send
        msg = Pose2D()
        # print 'ai_node: msg = %s' % msg

        count = (count + 1) % 100
        if count == 0 and print_when_commanding:
            # print _game_state
            print 'tick'
            print 'reset: {}\nplay: {}'.format(_game_state.reset_field,
                                               _game_state.play)
            pass


        if _game_state.reset_field and not _ignore_game_state:
            # Send robot to home
            if count == 0 and print_when_commanding:
                print 'Send robot to home'
                pass

            if _team_side == 'away':
                if _my_number == 1:
                    msg.x = +0.5
                    msg.y = 0
                    msg.theta = 180

                elif _my_number == 2:
                    msg.x = +1
                    msg.y = 0
                    msg.theta = 180

            else:
                if _my_number == 1:
                    msg.x = -0.5
                    msg.y = 0
                    msg.theta = 0

                elif _my_number == 2:
                    msg.x = -1
                    msg.y = 0
                    msg.theta = 0
            #if _my_number == 1:
            #    msg.x = 0.5
            #    msg.y = 0
            #    msg.theta = 0

            #elif _my_number == 2:
            #    msg.x = 1
            #    msg.y = 0
            #    msg.theta = 0

        elif _game_state.play or _ignore_game_state:
        # elif True:
            # Run AI as normal
            if count == 0 and print_when_commanding:
                print 'Run AI as normal'
                pass
            # Based on the state of the game and the positions of the players,
            # run the AI and return commanded positions for this robot
            ai.update(_me, _ally, _opp1, _opp2, _ball, _game_state)
            # ai.update(_me, _ally, _opp1, _opp2, _ball, None)
            cmds = ai.strategize()

            msg.x = cmds[0]
            msg.y = cmds[1]
            msg.theta = cmds[2]
            # msg.x = 0
            # msg.y = 0
            # msg.theta = _me.theta

        elif not _game_state.play and not _ignore_game_state:
            if count == 0 and print_when_commanding:
                print 'Paused'
                pass
            msg.x = _me.x
            msg.y = _me.y
            msg.theta = _me.theta

        # If we shouldn't play and the field doesn't need to be
        # reset, then the AI node is out of a job.
        # if _game_state.play or _game_state.reset_field or _ignore_game_state:
        if True:
            if count == 0 and print_when_commanding:
                print 'publishing command'
            pub.publish(msg)

        # Wait however long it takes to make this tick at 100Hz
        rate.sleep()



if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()

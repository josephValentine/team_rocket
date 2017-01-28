import numpy as np
from std_srvs.srv import Trigger
import rospy

import Skills
from Models import GameState, Field, GameInfo
from Geometry.Models import Position, Point, Angle

field_width = 3.53
no_attack = True

class AI(object):
    def __init__(self, team_side, ally_number):
        super(AI, self).__init__()

        # Create GameState object
        self.game_state = GameState(Field(), GameInfo(team_side))

        # Am I ally1?
        self.ally1 = (ally_number == 1)


    def update(self, me, ally, opp1, opp2, ball, game_state):
        # print me, ally, opp1, opp2, ball, game_state, type(game_state)
        f = self.game_state.field
        if self.ally1:
            f.ally1.position = _pose2d_to_position(me)
            f.ally2.position = _pose2d_to_position(ally)
        else:
            f.ally1.position = _pose2d_to_position(ally)
            f.ally2.position = _pose2d_to_position(me)
        f.opp1.position = _pose2d_to_position(opp1)
        f.opp2.position = _pose2d_to_position(opp2)
        f.ball.point = _pose2d_to_point(ball)
        # update game state


    def strategize(self):

        if self.ally1:
            # rush ball
            cmds = self.rush_goal(
                _position_to_pose2d(self.game_state.field.ally1.position),
                _point_to_pose2d(self.game_state.field.ball.point))

        else:
            # be a goalie (i.e., follow line on ball)
            # cmds = self.follow_ball_on_line(ball, -1.25)
            cmds = Skills.stay_between_goalnball(
                self.game_state, self.game_state.field.ally2)


        return cmds


    def follow_ball_on_line(self, ball, x_c):
        y_c = ball.y
        theta_c = 0
        return (x_c, y_c, theta_c)


    def rush_goal(self, me, ball):
        if no_attack:
            return me
        # print 'me ({}): {}'.format(type(me), me)
        # print 'ball ({}): {}'.format(type(ball), ball)
        # Use numpy to create vectors
        ballvec = np.array([[ball.x], [ball.y]])
        mevec = np.array([[me.x], [me.y]])
        goalvec = np.array([[field_width/2], [0]])

        # unit vector from ball to goal
        uv = goalvec - ballvec
        uv = uv/np.linalg.norm(uv)

        # compute a position 20cm behind ball, but aligned with goal
        p = ballvec - 0.20*uv

        # If I am sufficiently close to the point behind the ball,
        # or in other words, once I am 21cm behind the ball, just
        # drive to the goal.
        dist_to_ball = np.linalg.norm(p - mevec)
        # print 'p:', p
        # print 'mevec:', mevec
        # print 'distance to ball: ', dist_to_ball, type(dist_to_ball)
        if dist_to_ball < 0.21:
            # print 'Close enough to drive to goal'
            cmdvec = goalvec
            # Addition
            if dist_to_ball < 0.11:
                # print 'Close enough to kick!'
                # kick!
                try:
                    self.kick()
                except Exception as e:
                    print e
        else:
            # print 'Get behind ball'
            cmdvec = p

        return (cmdvec.flatten()[0], cmdvec.flatten()[1], 0)


    def kick(self):
        """Kick

        Send a service call to kick the ball.
        """
        # global _kick_num
        try:
            kick_srv = rospy.ServiceProxy('kick', Trigger)
            kick_srv()
            # print 'successfully kicked ball'
            # _kick_num = _kick_num + 1
            # print ("Kicking. Kick number: {}" .format(_kick_num))
        except rospy.ServiceException as e:
            # print "Kick service call failed: %s"%e
            pass


def p2d_2_pos(p):
    return Position(Point(p.x, p.y), Angle(p.theta, False))


def test():
    import Constants
    ai = AI('home', 2)
    cmds = ai.strategize()
    print cmds
    ai.update(_position_to_pose2d(ai.game_state.field.ally2.position),
              _position_to_pose2d(ai.game_state.field.ally1.position),
              _position_to_pose2d(ai.game_state.field.opp1.position),
              _position_to_pose2d(ai.game_state.field.opp2.position),
              Pose2D(1.5, -1.0, 0.0),
              # (1.5, -1.0, 0.0),
              ())
    ai.game_state.game_info.side = Constants.right_side
    cmds = ai.strategize()
    print cmds


def _position_to_pose2d(position):
    from geometry_msgs.msg import Pose2D
    return Pose2D(position.point.x, position.point.y, position.angle.radian)

def _point_to_pose2d(point):
    from geometry_msgs.msg import Pose2D
    return Pose2D(point.x, point.y, 0)

def _pose2d_to_position(pose2d):
    return Position(Point(pose2d.x, pose2d.y), Angle(pose2d.theta, False))

def _pose2d_to_point(pose2d):
    return Point(pose2d.x, pose2d.y)

if __name__ == '__main__':
   test()

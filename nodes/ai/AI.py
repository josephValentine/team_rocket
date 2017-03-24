import numpy as np
from std_srvs.srv import Trigger
import rospy

import Skills
from Models import GameState, Field, GameInfo
from Geometry.Models import Position, Point, Angle
from geometry_msgs.msg import Pose2D

#field_width = 3.53
field_width = 3.40
field_height = 2.38
no_attack = False

class AI(object):
    def __init__(self, team_side, ally_number):
        super(AI, self).__init__()

        # Create GameState object
        self.game_state = GameState(Field(), GameInfo(team_side))
        self.game_state.game_info.side = team_side
        self.team_side = team_side
        # Am I ally1?
        self.ally1 = (ally_number == "1")
        print 'I am ally %s' % ally_number
        self.timer = 0


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

        role_str = 'ally1' if self.ally1 else 'ally2'
        # print self.team_side, role_str, (me.x, me.y, me.theta)
        # update game state
        # print game_state
        # home_score, away_score, home_bot_count, away_bot_count,
        # remaining_seconds, play, reset_field, second_half

    def strategize(self):

        if self.ally1:
            # return _position_to_tuple(self.game_state.field.ally1.position)
            # rush ball
            cmds = self.rush_goal(
                _position_to_pose2d(self.game_state.field.ally1.position),
                _point_to_pose2d(self.game_state.field.ball.point))
            # print self.team_side, "forward cmds:", cmds
            # print self.team_side, "forward pos: ", \
            #     _position_to_pose2d(self.game_state.field.ally1.position)

        else:
            # be a goalie (i.e., follow line on ball)
            # cmds = self.follow_ball_on_line(ball, -1.25)
            cmds = _position_to_tuple(Skills.stay_between_goalnball(
                self.game_state, self.game_state.field.ally2))
            # print self.team_side,  "goalie cmds:", cmds
            # print self.team_side, "goalie pos: ", \
            #     _position_to_pose2d(self.game_state.field.ally2.position)

        # print self.team_side
        # if self.team_side != 'home':
        #     print 'flip (before =', cmds, ',',
        #     cmds = _flip_coordinate_system(cmds)
        #     print 'after =', cmds, ')'

        pos_str = "forward" if self.ally1 else "goalie"
        # print self.team_side, pos_str, "cmds:", cmds
        my_pos = self.game_state.field.ally1.position if self.ally1 else \
                 self.game_state.field.ally2.position
        # print self.team_side, pos_str, "pos: ", my_pos

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
        if self.game_state.game_info.side == 'home':
            goalvec = -goalvec

        # Check if ball is between us and goal. If so, go to goal.
        me2ballvec = (ballvec - mevec) * 1000
        goalTopvec = np.array([[field_width/2], [field_height/2]])
        goalBottomvec = np.array([[field_width/2], [-field_height/2]])
        command = _intersection(mevec, mevec + me2ballvec, goalBottomvec, goalTopvec)
        if command != None:
            cmdvec = command
        else:
            # Get vectors away from ball along the principle axes.
            # These will be used to get around the ball instead of
            # driving the ball into our own goal.
            # 20 cm away from the ball
            b_posXvec = ballvec + np.array([[ 0.20], [  0.0]])
            b_posYvec = ballvec + np.array([[  0.0], [ 0.20]])
            b_negXvec = ballvec + np.array([[-0.20], [  0.0]])
            b_negYvec = ballvec + np.array([[  0.0], [-0.20]])

            # unit vector from ball to goal
            uv = goalvec - ballvec
            uv = uv/np.linalg.norm(uv)

            # compute a position 20cm behind ball, but aligned with goal
            p = ballvec - 0.30*uv

            # Compute vector from me to commanded position
            me2comvec = p - mevec

            # Find the closest ball vector that me2comvec intersects
            # Initialize closest distances to 50 meters
            close_dist = 50
            dist = 50
            closest = _intersection(mevec, p, ballvec, b_posXvec)
            if closest != None:
                close_dist = np.linalg.norm(mevec - closest)
                p = ballvec + b_posXvec
            intersect = _intersection(mevec, p, ballvec, b_posYvec)
            if intersect != None:
                dist = np.linalg.norm(mevec - intersect)
            if dist < close_dist:
                closest = intersect
                close_dist = dist
                p = ballvec + b_posYvec
            intersect = _intersection(mevec, p, ballvec, b_negXvec)
            if intersect != None:
                dist = np.linalg.norm(mevec - intersect)
            if dist < close_dist:
                closest = intersect
                close_dist = dist
                p = ballvec + b_negXvec
            intersect = _intersection(mevec, p, ballvec, b_negYvec)
            if intersect != None:
                dist = np.linalg.norm(mevec - intersect)
            if dist < close_dist:
                closest = intersect
                close_dist = dist
                p = ballvec + b_negYvec


            cmdvec = p
            # # If I am sufficiently close to the point behind the ball,
            # # or in other words, once I am 21cm behind the ball, just
            # # drive to the goal.
            # dist_to_ball = np.linalg.norm(p - mevec)
            # # print 'p:', p
            # # print 'mevec:', mevec
            # # print 'distance to ball: ', dist_to_ball, type(dist_to_ball)
            # if dist_to_ball < 0.31:
            #     if self.timer > 25:
            #         # print 'Close enough to drive to goal'
            #         cmdvec = goalvec
            #     else:
            #         self.timer += 1
            #         # try to get stable first before driving
            #         cmdvec = p
            #     # Addition
            #     # if dist_to_ball < 0.11:
            #     #     # print 'Close enough to kick!'
            #     #     # kick!
            #     #     try:
            #     #         self.kick()
            #     #     except Exception as e:
            #     #         print e
            # else:
            #     self.timer = 0
            #     # print 'Get behind ball'
            #     cmdvec = p

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
    # print cmds
    ai.update(_position_to_pose2d(ai.game_state.field.ally2.position),
              _position_to_pose2d(ai.game_state.field.ally1.position),
              _position_to_pose2d(ai.game_state.field.opp1.position),
              _position_to_pose2d(ai.game_state.field.opp2.position),
              Pose2D(1.5, -1.0, 0.0),
              # (1.5, -1.0, 0.0),
              ())
    ai.game_state.game_info.side = Constants.right_side
    cmds = ai.strategize()
    # print cmds


def _position_to_pose2d(position):
    from geometry_msgs.msg import Pose2D
    return Pose2D(position.point.x, position.point.y, position.angle.degree)

def _point_to_pose2d(point):
    from geometry_msgs.msg import Pose2D
    return Pose2D(point.x, point.y, 0)

def _pose2d_to_position(pose2d):
    return Position(Point(pose2d.x, pose2d.y), Angle(pose2d.theta, True))

def _pose2d_to_point(pose2d):
    return Point(pose2d.x, pose2d.y)

def _pose2d_to_tuple(pose2d):
    return (pose2d.x, pose2d.y, pose2d.theta)

def _position_to_tuple(position):
    return (position.point.x, position.point.y, position.angle.degree)

def _flip_coordinate_system(cmds):
    return cmds
    # return (-cmds[0], -cmds[1], (cmds[2]+180) % 360)

def _intersection(a1, a2, b1, b2):#s1, s2):
    left = max(min(a1[0], a2[0]), min(b1[0], b2[0]))
    right = min(max(a1[0], a2[0]), max(b1[0], b2[0]))
    top = max(min(a1[1], a2[1]), min(b1[1], b2[1]))
    bottom = min(max(a1[1], a2[1]), max(b1[1], b2[1]))

    if top > bottom or left > right:
        return None # No intersection
    if (top,left) == (bottom,right):
        return np.array([[left],[top]])
    return None # Lines overlap; shouldn't ever happen

def _perp(a):
    b = empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

def _seg_intersect(a1, a2, b1, b2):
    da = a2 - a1
    db = b2 - b1
    dp = a1 - b1
    dap = _perp(da)
    denom = dot(dap, db)
    num = dot(dap, dp)
    # This could potentially cause a divide by 0 error if the
    # lines are parallel
    intersect = (num / denom.astype(float)) * db + b1
    return intersect

if __name__ == '__main__':
   test()

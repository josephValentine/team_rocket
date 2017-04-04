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
goal_height = 0.619
no_attack = False

class AI(object):
    def __init__(self, team_side, my_number):
        super(AI, self).__init__()

        # Create GameState object
        self.game_state = GameState(Field(), GameInfo(team_side))
        self.game_state.game_info.side = team_side
        self.team_side = team_side
        # Am I ally1?
        # self.ally1 = (ally_number == "1")
        self.ally1 = (my_number == 1) or (my_number == '1')
        print 'I am ally %s' % my_number
        print 'my_number: {} ({})'.format(my_number, type(my_number))
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
            # print 'rush goal'
            cmds = self.rush_goal(
                _position_to_pose2d(self.game_state.field.ally1.position),
                _point_to_pose2d(self.game_state.field.ball.point))
            # print self.team_side, "forward cmds:", cmds
            # print self.team_side, "forward pos: ", \
            #     _position_to_pose2d(self.game_state.field.ally1.position)

        else:
            # print 'goalie'
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
        # go from center of goal to +/- quarter goal width to aim for the center
        # half of the goal
        # this is a factor proportional to our distance from the goal to make it
        # ok to be 'less accurate' if we're far away from the goal
        min_target_factor = 2 * field_height/(field_width*2)
        min_target_height = max(min_target_factor*(me.x + field_width/2), goal_height/4)
	
	#if me.x > 0:
	#    min_target_height = goal_height/4
	#else:
	#    min_target_height = 2

        goalTopvec = np.array([[field_width/2], [min_target_height]])
        goalBottomvec = np.array([[field_width/2], [-min_target_height]])
        # If we are home the goal we score on is on the other side.
        if self.game_state.game_info.side == 'home':
            goalvec = -goalvec
            goalTopvec = -goalTopvec
            goalBottomvec = -goalBottomvec

        # Check if ball is between us and goal. If so, go to goal.
        me2ballvec = (ballvec - mevec) * 1000
        did_intersect = _do_intersect(mevec, mevec + me2ballvec, goalBottomvec,
                                      goalTopvec)
        if did_intersect:
            # print 'Going to goal.'
            cmdvec = _seg_intersect(mevec, mevec + me2ballvec, goalBottomvec,
                                    goalTopvec)
        else:
            # Get vectors away from ball along the principle axes.
            # These will be used to get around the ball instead of
            # driving the ball into our own goal.
            # 20 cm away from the ball
            b_posXvec = ballvec + np.array([[ 0.20], [ 0.00]])
            b_posYvec = ballvec + np.array([[ 0.00], [ 0.20]])
            b_negXvec = ballvec + np.array([[-0.20], [ 0.00]])
            b_negYvec = ballvec + np.array([[ 0.00], [-0.20]])

            # unit vector from ball to goal
            uv = goalvec - ballvec
            uv = uv/np.linalg.norm(uv)

            # compute a position 20cm behind ball, but aligned with goal
            p = ballvec - 0.30*uv
            # print('p: {} ({})'.format(p, type(p)))

            # Compute vector from me to commanded position
            me2comvec = p - mevec

            # Find the closest ball vector that me2comvec intersects
            # Initialize closest distances to 50 meters
            close_dist = 50
            dist = 50

            # print 'mevec: {}, ballvec: {}'.format(mevec, ballvec)
            # print 'mevec->p: {}'.format(p - mevec)
            closest_intersection = None
            closest_distance = float('inf')
            closest_intersection_vec = None
            for vec in [b_posXvec, b_posYvec, b_negXvec, b_negYvec]:
                # print 'vec:', vec
                did_intersect = _do_intersect(mevec, p, ballvec, vec)
                if did_intersect:
                    # print '\tIntersect with: {}'.format(vec)
                    intersection = _seg_intersect(mevec, p, ballvec, vec)
                    distance = np.linalg.norm(mevec - intersection)
                    # print '\tintersection: {}\n\tdistance: {}'.format(
                    #     intersection, distance)
                    if distance < closest_distance:
                        closest_intersection = intersection
                        closest_distance = distance
                        closest_intersection_vec = vec
                # print

            if closest_intersection is not None:
                p = ballvec + closest_intersection_vec
                # print('p: {} ({})'.format(p, type(p)))

            # print('p: {} ({})'.format(p, type(p)))
            cmdvec = p
            # print('p: {} ({})'.format(p, type(p)))
	    
        buffHor = 0.5
	buffVert = 0.5
	maxX = (field_width - buffHor)/2
	minX = -maxX
	maxY = (field_height - buffVert)/2
	minY = -maxY
	#print(maxX,minX,maxY,minY)
	if cmdvec[0] > maxX:
	    cmdvec[0] = maxX
	if cmdvec[1] > maxY:
	    cmdvec[1] = maxY
	if cmdvec[0] < minX:
	    cmdvec[0] = minX
	if cmdvec[1] < minY:
	    cmdvec[1] = minY
	    # print(cmdvec)
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

def _perp(a):
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

def _seg_intersect(a1, a2, b1, b2):
    da = a2 - a1
    db = b2 - b1
    dp = a1 - b1
    dap = _perp(da)
    # print '\tdb: {} ({})'.format(db, type(db))
    # print '\tdap: {} ({})'.format(dap, type(dap))
    denom = np.asscalar(np.dot(dap.T, db))
    num = np.asscalar(np.dot(dap.T, dp))
    # print '\tdenom: {} ({})'.format(denom, type(denom))
    # This could potentially cause a divide by 0 error if the
    # lines are parallel
    # if denom[0][0] == 0:
    if denom == 0:
        # print '\t_seg_intersect(...): denom == 0'
        return ((a1[0] + a2[0]) / 2, (a1[1] + a2[1]) / 2)
    intersect = (num / denom) * db + b1
    return intersect


################################################################################
# see: http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
# 0: co-linear
# 1: clockwise
# 2: counter-clockwise
def _orientation(p, q, r):
    val = (q[1] - p[1]) * (r[0] - q[0]) - \
          (q[0] - p[0]) * (r[1] - q[1])

    if val == 0: return 0
    return 1 if val > 0 else 2

def _on_segment(p, q, r):
    if q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and \
       q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]):
        return True
    else:
        return False

def _do_intersect(p1, q1, p2, q2):
    o1 = _orientation(p1, q1, p2)
    o2 = _orientation(p1, q1, q2)
    o3 = _orientation(p2, q2, p1)
    o4 = _orientation(p2, q2, q1)

    # general case
    if o1 != o2 and o3 != o4: return True

    # special cases
    # p1, q1, and p2 are colinear and p2 lies on segment p1q1
    if o1 == 0 and _on_segment(p1, p2, q1): return True
    # p1, q1, and q2 are colinear and q2 lies on segment p1q1
    if o2 == 0 and _on_segment(p1, p2, q1): return True
    # p2, q2, and p1 are colinear and p1 lies on segment p2q2
    if o3 == 0 and _on_segment(p2, p1, q2): return True
    # p2, q2, and q1 are colinear and q1 lies on segment p2q2
    if o4 == 0 and _on_segment(p2, p1, q2): return True

    return False
################################################################################

def test():
    import Constants
    ai = AI('home', "1")
    cmds = ai.strategize()
    print cmds
    ai.update(_position_to_pose2d(ai.game_state.field.ally2.position),
              _position_to_pose2d(ai.game_state.field.ally1.position),
              _position_to_pose2d(ai.game_state.field.opp1.position),
              _position_to_pose2d(ai.game_state.field.opp2.position),
              Pose2D(1.5, -1.0, 0.0),
              # (1.5, -1.0, 0.0),
              None)
    ai.game_state.game_info.side = Constants.right_side
    cmds = ai.strategize()
    print cmds
    print '-'*80
    print 'First'
    # put our robot in between ball and goal, it should go to the side
    ai.update(Pose2D(1.0, 0.01, 0.0), # me
              Pose2D(-1.0, 1.0, 0.0), # ally
              Pose2D(1.0, 1.0, 0.0),  # opp1
              Pose2D(1.0, -1.0, 0.0), # opp2
              Pose2D(0.0, 0.0, 0.0),  # ball
              # (1.5, -1.0, 0.0),
              None)
    cmds = ai.strategize()
    print cmds
    print '-'*80
    print 'Second'
    ai.update(Pose2D(1.0, -1, 0.0), # me
              Pose2D(-1.0, 1.0, 0.0), # ally
              Pose2D(1.0, 1.0, 0.0),  # opp1
              Pose2D(1.0, -1.0, 0.0), # opp2
              Pose2D(0.0, 0.0, 0.0),  # ball
              # (1.5, -1.0, 0.0),
              None)
    cmds = ai.strategize()
    print cmds



if __name__ == '__main__':
   test()

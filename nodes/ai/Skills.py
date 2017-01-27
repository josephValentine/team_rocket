"""Combination of Utilities to perform simple tasks"""

import Constants
from Models import Position
import Functions

def get_point_behind_ball(field, angle, distance):
   """Get the point behind the ball facing a given direction.

   field (Field)  : Current state of game field
   angle (Angle)  : Direction to face
   return (Point) : A point behind the ball

   """
   ball_point = field.ball.point
   dir_vec = Functions.get_normalized_vector(angle)
   # dist_behind_ball = 0.2
   offset_vec = Functions.scale_vector(dir_vec, -distance)
   return ball_point + offset_vec


# def kick_ball_toward_pos(robot, ball, pos):
#    ang = find_angle(ball, pos)
#    rotate_to_ang_abs(robot, ang)
#    if ready_to_kick(robot, ball):
#       kick(robot)


# def ready_to_kick(robot, ball):
#    return ball_close_to_kicker and not other_robot_close_to_kicker(robot)


# def ball_close_to_kicker(robot, ball):
#    distance = find_distance(robot, ball)
#    return distance <= min_kick_distance



# def push_ball_toward_pos(robot, ball, pos):


def test():
   from Models import Field, Angle
   f = Field()
   print(f)
   a = Angle(33, True)
   d1 = 0.2
   p1 = get_point_behind_ball(f, a, d1)
   print('get_point_behind_ball(f, {}, {}) = {}'.format(a, d1, p1))
   print('distance = {}'.format(Functions.dist(f.ball.point, p1)))
   d2 = 3.0
   p2 = get_point_behind_ball(f, a, d2)
   print('get_point_behind_ball(f, {}, {}) = {}'.format(a, d2, p2))
   print('distance = {}'.format(Functions.dist(f.ball.point, p2)))



if __name__ == '__main__':
   test()

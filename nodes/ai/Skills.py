""" Combination of Utilities to perform simple tasks """

import Constants

def get_pos_behind_ball(field, angle):
   """ Get the position behind the ball facing a given direction

   field (Field)   : Current state of game field
   angle (Angle)   : Direction to face
   return (Pose2D) : A position and orientation to be behind the ball
   """
   p = field.ball.pos
   v = get_normalized_vector(angle)
   distance_behind_ball = 0.2
   v = scale_vector(-distance_behind_ball)
   
   pass


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

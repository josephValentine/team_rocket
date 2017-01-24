""" Combination of Utilities to perform simple tasks """


def kick_ball_toward_pos(robot, ball, pos):
   ang = find_angle(ball, pos)
   rotate_to_ang_abs(robot, ang)
   if ready_to_kick(robot, ball):
      kick(robot)
   

def ready_to_kick(robot, ball):
   return ball_close_to_kicker and not other_robot_close_to_kicker(robot)


def ball_close_to_kicker(robot, ball):
   distance = find_distance(robot, ball)
   return distance <= min_kick_distance

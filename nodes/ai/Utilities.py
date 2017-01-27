"""Elementary and Secondary actions"""

# Elementary 
def move_to_pos_abs(robot, pos):
   set_commanded_pos(pos)
   pass

def move_to_pos_rel(robot, pos):
   set_commanded_pos((robot.x + pos.x, robot.y + pos.y))
   pass

def rotate_to_ang_abs(robot, ang):
   pass

def rotate_by_ang_rel(robot, ang):
   pass

def move_to_pose2d(robot, pose2d):
   set_commanded_pos(robot, (pose2d.x, pose2d.y))
   set_commanded_ang(robot, pose2d.ang)

def kick(robot):
   # call kicker
   pass


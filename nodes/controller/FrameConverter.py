import math
from geometry_msgs.msg import Pose2D

def _destination_world_to_body_coordinates(dest, curPos):
    """ Takes the destination of the robot in world coordinates and converts it
    a position in the robot frame of reference.
    dest is a Pose2D representing the world coordinates of where the robot needs
    to go.
    curPos is a Pose2D representing where the robot currently is in world
    coordinates.
    Returns a Pose2D representing where the robot should go in its own frame of
    reference.
    
    Assumptions:
    All angles are in degrees.
    """
    
    deltaX = dest.x - curPos.x
    deltaY = dest.y - curPos.y
    angle_rad = math.radians(curPos.theta)
    newX = math.cos(angle_rad) * deltaX + math.sin(angle_rad) * deltaY
    newY = math.cos(angle_rad) * deltaY - math.sin(angle_rad) * deltaX
    newAngle = dest.theta - curPos.theta
    return Pose2D(newX, newY, newAngle)

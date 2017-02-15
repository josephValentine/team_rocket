import math
from geometry_msgs.msg import Pose2D

def _convert_world_to_body_velocities(vx_w, vy_w, curAngle):
    """ Converts world velocities to body velocities.
    vx_w is the world x velocity.
    vy_w is the world y velocity.
    curAngle is the current bearing of the robot in the world coordinate system.
    
    Assumptions:
    curAngle is in radians.
    """
    
    vx_b = math.cos(curAngle) * vx_w + math.sin(curAngle) * vy_w
    vy_b = math.cos(curAngle) * vy_w - math.sin(curAngle) * vx_w
    return vx_b, vy_b

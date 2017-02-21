<<<<<<< HEAD
=======
import math
import numpy as np
# from geometry_msgs.msg import Pose2D

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

def _convert_world_to_motor_velocities(vx_w, vy_w, wz, curAngle):
    """ Converts world velocities to motor velocities.
    vx_w is the world x velocity.
    vy_w is the world y velocity.
    wz is the angular speed of the robot.
    curAngle is the current bearing of the robot in the world coordinate system.

    returns numpy matrix of wheel speeds(omega)
    wz is in radians/second.
    curAngle is in radians.
    converts inch measurements to meters
    """
    in2m=0.0254
    #radius of wheels
    R = 1

    #velocity vectors of wheels
    sbx1 = in2m * np.sqrt(3)/2
    sbx2 = in2m * -np.sqrt(3)/2
    sbx3 = in2m * 0

    sby1 = in2m * -0.5
    sby2 = in2m * -0.5
    sby3 = in2m * 1

    #distance from center of robot to wheel
    rbx1 = in2m * 3.5 * np.cos(np.pi/3)
    rbx2 = in2m * 3.5 * np.cos(-np.pi/3)
    rbx3 = in2m * -3.5

    rby1 = in2m * 3.5 * np.sin(np.pi/3)
    rby2 = in2m * 3.5 * np.sin(-np.pi/3)
    rby3 = in2m * 0

    r_theta = np.matrix([
        [np.cos(curAngle), np.sin(curAngle), 0],
        [-np.sin(curAngle), np.cos(curAngle), 0],
        [0, 0, 1]
    ])
    v_world = np.matrix([
        [vx_w],
        [vy_w],
        [wz]
    ])
    M = (1/R) * np.matrix([
        [sbx1, sby1, (sby1*rbx1 - sbx1*rby1)],
        [sbx2, sby2, (sby2*rbx2 - sbx2*rby2)],
        [sbx3, sby3, (sby3*rbx3 - sbx3*rby3)]
    ])

    omega = np.dot(np.dot(M,r_theta), v_world)
    # print 'omega: {}\nrepr: {}\ntype: {}'.format(
    #     omega, repr(omega), type(omega))
    # print '[0]: {}\n[0][0]: {}'.format(omega[0], omega[0][0])
    # print 'array: {}\ntype: {}'.format(np.array(omega), type(np.array(omega)))
    return omega.item(0,0), omega.item(1,0), omega.item(2,0)

def test():
    omega = _convert_world_to_motor_velocities(10,0,3.14)
    print(omega)

if __name__ == '__main__':
   test()
>>>>>>> 09732407875aaab8b78d768311467f1e74afda44

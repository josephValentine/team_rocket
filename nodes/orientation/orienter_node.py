#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose2D
# from std_srvs.srv import Trigger, TriggerResponse

_team_side = 'home'

def main():
    global _team_side
    param_name = rospy.search_param('team_side')
    _team_side = rospy.get_param(param_name, 'home')
    
    rospy.init_node('orienter', anonymous=False)

    # Sub/Pub
    rospy.Subscriber('vision/away1', RobotState, _handle_robot_state)
    rospy.Subscriber('desired_position', Pose2D, _handle_desired_position)
    pub = rospy.Publisher('vel_cmds', Twist, queue_size=10)
    pub_PIDInfo = rospy.Publisher('pidinfo', PIDInfo, queue_size=10)

    # Services
    # rospy.Service('/controller/toggle', Trigger, _toggle)

    # Get the correct PID stuff
    gains = rospy.get_param('gains') # returns as a dict
    # {'x': {'P': 0, 'I': 0, 'D': 0}, ... }

    # initialize the controller
    Controller.init(gains)

    rate = rospy.Rate(int(1/_ctrl_period))
    while not rospy.is_shutdown():

        global _ctrl_on

        if _ctrl_on:
            (vx, vy, w) = Controller.update(_ctrl_period, _xhat, _yhat, _thetahat)

            if _team_side == 'away':
               # flip coordinate system
               vx = -vx
               vy = -vy
               w  = w + 180 % 360
            
            # Publish Velocity Commands
            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.angular.z = w
            pub.publish(msg)

            # # Publish PID Info
            # msg = PIDInfo()
            # msg.error.x = Controller.PID_x.error_d1
            # msg.error.y = Controller.PID_y.error_d1
            # msg.error.theta = Controller.PID_theta.error_d1
            # set_point = Controller.get_commanded_position()
            # msg.desired.x = set_point[0]
            # msg.desired.y = set_point[1]
            # msg.desired.theta = set_point[2]
            # msg.actual.x = _xhat
            # msg.actual.y = _yhat
            # msg.actual.theta = _thetahat
            # pub_PIDInfo.publish(msg)


        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()

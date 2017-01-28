import numpy as np
from std_srvs.srv import Trigger
import rospy

field_width = 3.53

class AI(object):
    def __init__(self, team_side, ally_number):
        super(AI, self).__init__()

        # Which team side (home/away) am I on?
        self.team_side = team_side

        # Am I ally1?
        self.ally1 = (ally_number == 1)
        

    def strategize(self, me, ally, opp1, opp2, ball, game_state):
        
        if self.ally1:
            # rush ball
            cmds = self.rush_goal(me, ball)

        else:
            # be a goalie (i.e., follow line on ball)
            cmds = self.follow_ball_on_line(ball, -1.25)


        return cmds


    def follow_ball_on_line(self, ball, x_c):
        y_c = ball.y
        theta_c = 0
        return (x_c, y_c, theta_c)


    def rush_goal(self, me, ball):
        # Use numpy to create vectors
        ballvec = np.array([[ball.x], [ball.y]])
        mevec = np.array([[me.x], [me.y]])
        goalvec = np.array([[field_width/2], [0]])

        # unit vector from ball to goal
        uv = goalvec - ballvec
        uv = uv/np.linalg.norm(uv)

        # compute a position 20cm behind ball, but aligned with goal
        p = ballvec - 0.20*uv

        # If I am sufficiently close to the point behind the ball,
        # or in other words, once I am 21cm behind the ball, just
        # drive to the goal.
        dist_to_ball = np.linalg.norm(p - mevec)
        print( 'p:', p)
        print( 'mevec:', mevec)
        print( 'distance to ball: ', dist_to_ball, type(dist_to_ball))
        if dist_to_ball < 0.21:
            print( 'Close enough to drive to goal')
            cmdvec = goalvec
            # Addition
            if dist_to_ball < 0.11:
                print( 'Close enough to drive to kick!')
                # kick!
                try:
                    self.kick()
                except Exception as e:
                    print( e)
        else:
            print( 'Get behind ball')
            cmdvec = p

        return (cmdvec.flatten()[0], cmdvec.flatten()[1], 0)


    def kick(self):
        """Kick

        Send a service call to kick the ball.
        """
        # global _kick_num
        try:
            kick_srv = rospy.ServiceProxy('kick', Trigger)
            kick_srv()
            print( 'successfully kicked ball')
            # _kick_num = _kick_num + 1
            # print( ("Kicking. Kick number: {}" .format(_kick_num)))
        except rospy.ServiceException as e:
            print( "Kick service call failed: %s"%e)

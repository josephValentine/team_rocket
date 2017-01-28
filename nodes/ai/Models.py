"""Models for robots, ball, etc."""

from Geometry.Models import Position, Point, Angle
import Constants

class GameState:
    """Information about the past and current state of the game.

    This includes the current field and the past history.

    """
    def __init__(self, field, game_info, game_history=None):
        self.field                         = field
        self.game_info                     = game_info
        if game_history: self.game_history = game_history
        else:            self.game_history = GameHistory()
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'GameState({}, {})'.format(self.field, self.game_history)


class Field:
    """Holds information about everything on the field.

    This includes all robots and the ball.

    """
    def __init__(self):
        self.ball  = Ball(Point(Constants.ball_start_x,
                                Constants.ball_start_y))
        self.ally1 = Robot(Constants.team_us,
                           Constants.ally_1_id,
                           Point(Constants.ally_1_start_x,
                                 Constants.ally_1_start_y))
        self.ally2 = Robot(Constants.team_us,
                           Constants.ally_2_id,
                           Point(Constants.ally_2_start_x,
                                 Constants.ally_2_start_y))
        self.opp1  = Robot(Constants.team_them,
                           Constants.opp_1_id,
                           Point(Constants.opp_1_start_x,
                                 Constants.opp_1_start_y))
        self.opp2  = Robot(Constants.team_them,
                           Constants.opp_2_id,
                           Point(Constants.opp_2_start_x,
                                 Constants.opp_2_start_y))
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'Field({}, {}, {}, {}, {})'.format(
            self.ball, self.ally1, self.ally2, self.opp1, self.opp2)


class Robot:
    """Holds information about a robot."""
    def __init__(self, team, id, position):
       self.team     = team
       self.id       = id
       self.position = position
    def __eq__(self, other):
        return type(self) == type(other) and self.id == other.id
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'Robot({}, {}, {})'.format(self.team, self.id, self.position)
    def update_position_from_tuple(self, position_tuple):
        self.position = Position.from_tuple(position_tuple)


class Ball:
    """Holds information about the ball."""
    def __init__(self, point):
        self.point = point
    def __eq__(self, other):
        return type(self) == type(other)
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'Ball({})'.format(self.point)
    def update_point_from_tuple(self, position_tuple):
        self.point = Point(position_tuple)


class GameInfo:
    """Information about the current status of the game.

    This includes the current side, half, time elapsed/left, and points.

    """
    def __init__(self, side, half=None, time_elapsed=None, score=None):
        self.side                          = side
        if half:         self.half         = half
        else:            self.half         = Constants.first_half
        if time_elapsed: self.time_elapsed = time_elapsed
        else:            self.time_elapsed = 0
        if score:        self.score        = score
        else:            self.score        = Score(0,0)
    def get_time_elapsed(self):
        return self.time_elapsed
    def get_time_left(self):
        return Constants.time_in_half - self.time_elapsed
    def __str__(self):
        return repr(self)
    def __repr__(self):
        return 'GameInfo({}, {}, {}, {})'.format(self.side, self.half,
                                                 self.time_elapsed, self.score)
    def get_home_goal_point(self):
        if self.side == Constants.left_side:
            return Point(Constants.goal_point_left_x,
                         Constants.goal_point_left_y)
        elif self.side == Constants.right_side:
            return Point(Constants.goal_point_right_x,
                         Constants.goal_point_right_y)
        raise Exception('invalid side in GameInfo')
    def get_opp_goal_point(self):
        if self.side == Constants.left_side:
            return Point(Constants.goal_point_right_x,
                         Constants.goal_point_right_y)
        elif self.side == Constants.right_side:
            return Point(Constants.goal_point_left_x,
                         Constants.goal_point_left_y)
        raise Exception('invalid side in GameInfo')



class Score:
    """The scores for us and them."""
    def __init__(self, us=None, them=None):
        if (us is None) != (them is None):
            raise Exception('cannot initialize Score with only one side\'s '+
                            'score')
        if us:   self.us   = us
        else:    self.us   = 0
        if them: self.them = them
        else:    self.them = 0


class GameHistory:
    """Information about what has happened in the game so far.

    This could contain strategies for our team or our opponenets, how aggressive
    each team was, etc.

    """
    def __init__(self):
        pass
    def __str__(self):
        return 'Unimplemented GameHistory'
    def __repr__(self):
        return 'GameHistory()'

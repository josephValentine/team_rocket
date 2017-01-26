""" Models for robots, ball, etc. """

import Functions as f

class Robot:
    def __init__(self, team, id, pos):
       self.team = team
       self.id   = id
       self.pos  = pos


class Ball:
    def __init__(self, pos):
        self.pos = pos


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Line:
    def __init__(self, beg, end):
        self.beg = beg
        self.end = end

class Angle:
    def __init__(self, value, is_degree):
        if is_degree:
            self.degree = value
            self.radian = f.deg_to_rad(value)
        else:
            self.degree = f.rad_to_deg(value)
            self.radian = value

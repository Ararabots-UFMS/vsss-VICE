
from strategy.blackboard import Blackboard
from strategy.robots.skill.route import NormalMovement

"""Contains all KickOffActions the robot must do (in order or not) during the match"""

class OurActionDefender():
    def __init__(self):
        self.name = "OurAttackerAction"
        self.blackboard = Blackboard()

    def __call__(self, **kwds):
        self.movement = NormalMovement()
        return self.movement.outsideCenterCircle()

    def run(self):
        self.movement = NormalMovement()
        return self.movement.outsideCenterCircle()


class TheirActionDefender():
    def __init__(self):
        self.name = "TheirAttackerAction"
        self.blackboard = Blackboard()

    def __call__(self, **kwds):
        self.movement = NormalMovement()
        return self.movement.outsideCenterCircle()
        
    def run(self):
        self.movement = NormalMovement()
        return self.movement.outsideCenterCircle()
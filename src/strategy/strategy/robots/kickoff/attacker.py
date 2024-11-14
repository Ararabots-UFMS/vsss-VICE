from strategy.blackboard import Blackboard
from strategy.skill.route import NormalMovement

"""Contains all KickOffActions the robot must do (in order or not) during the match"""

class OurActionAttacker():
    def __init__(self):
        self.name = "OurActionAttacker"
        self.blackboard = Blackboard()

    def __call__(self, **kwds):
        self.movement = NormalMovement()
        return self.movement.moveToCenter()


class TheirActionAttacker():
    def __init__(self):
        self.name = "TheirActionAttacker"
        self.blackboard = Blackboard()

    def __call__(self, **kwds):
        self.movement = NormalMovement()
        return self.movement.outsideCenterCircle()

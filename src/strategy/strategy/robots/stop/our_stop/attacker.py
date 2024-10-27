from strategy.blackboard import Blackboard
from strategy.robots.skill.route import NormalMovement

"""Contains all KickOffActions the robot must do (in order or not) during the match"""

class OurAttackerAction():
    def __init__(self):
        self.name = "OurAttackerAction"
        self.blackboard = Blackboard()

    def __call__(self, id, **kwds):
        self.movement = NormalMovement(id)
        return self.movement.run()

    def run(self, id):
        self.movement = NormalMovement(id)
        return self.movement.run()


class TheirAttackerAction():
    def __init__(self):
        self.name = "TheirAttackerAction"
        self.blackboard = Blackboard()

    def __call__(self, id, **kwds):
        self.movement = NormalMovement(id)
        return self.movement.run()
        
    def run(self, id):
        self.movement = NormalMovement(id)
        return self.movement.run()
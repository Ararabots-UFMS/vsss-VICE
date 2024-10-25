
from strategy.blackboard import Blackboard
from strategy.robots.skill.route import NormalMovement

"""Contains all KickOffActions the robot must do (in order or not) during the match"""

class OurAttackerAction():
    def __init__(self):
        self.name = "OurAttackerAction"
        self.blackboard = Blackboard()
        self.movement = NormalMovement(self.name)

    def run(self):
        return self.movement.run()


class TheirAttackerAction():
    def __init__(self):
        self.name = "TheirAttackerAction"
        self.blackboard = Blackboard()
        self.movement = NormalMovement(self.name)

    def run(self):
        return self.movement.run()
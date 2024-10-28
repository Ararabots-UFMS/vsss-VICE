from strategy.blackboard import Blackboard
from strategy.robots.skill.route import NormalMovement

"""Contains all PenaltyActions the robot must do (in order or not) during the match"""

class OurActionDefender():
    def __init__(self):
        self.name = "OurAttackerAction"
        self.blackboard = Blackboard()
        #self.movement = MoveToPoint(self.name)

    def __call__(self):
        self.movemet = NormalMovement() 
        return self.movemet.moveToPenaltyDefender() # TODO remove this method
        #return self.movement.moveToPenalty()

    def run(self):
        self.movemet = NormalMovement() 
        return self.movemet.moveToPenaltyDefender() # TODO remove this method
        

class TheirActionDefender():
    def __init__(self):
        self.name = "TheirAttackerAction"
        self.blackboard = Blackboard()

    def __call__(self):
        self.movement = NormalMovement() # TODO remove this class
        return self.movement.moveToPenaltyDefender()

    def run(self):
        self.movement = NormalMovement() # TODO remove this class
        return self.movement.moveToPenaltyDefender()
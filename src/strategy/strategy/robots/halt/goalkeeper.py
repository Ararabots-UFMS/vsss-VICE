from strategy.blackboard import Blackboard
from strategy.skill.route import BreakStrategy

"""Contains all PenaltyActions the robot must do (in order or not) during the match"""

class ActionGoalKeeper():
    def __init__(self):
        self.name = "OurAttackerAction"
        self.blackboard = Blackboard()
        #self.movement = MoveToPoint(self.name)

    def __call__(self):
        self.movemet = BreakStrategy() 
        return self.movemet._break() # TODO remove this method
        #return self.movement.moveToPenalty()

    def run(self):
        self.movemet = BreakStrategy() 
        return self.movemet._break() # TODO remove this method
        

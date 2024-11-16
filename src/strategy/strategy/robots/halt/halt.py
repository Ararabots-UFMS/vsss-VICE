from strategy.blackboard import Blackboard
from strategy.skill.route import BreakStrategy

"""Contains all HaltActions the robot must do (in order or not) during the match"""

class GameHalted():
    def __init__(self):
        self.name = "GameHalted"
        self.blackboard = Blackboard()
        self.movemet = BreakStrategy() 

    def __call__(self):
        return self.movemet._break() 
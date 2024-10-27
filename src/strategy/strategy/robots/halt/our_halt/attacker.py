from strategy.blackboard import Blackboard
from strategy.robots.skill.route import NormalMovement

"""Contains all PenaltyActions the robot must do (in order or not) during the match"""

class OurAttackerAction():
    def __init__(self):
        self.name = "OurAttackerAction"
        self.blackboard = Blackboard()
        #self.movement = MoveToPoint(self.name)

    def __call__(self, id):
        self.movemet = NormalMovement(id) 
        return self.movemet.run() # TODO remove this method
        #return self.movement.moveToPenalty()

    def run(self, id):
        self.movemet = NormalMovement(id) 
        return self.movemet.run() # TODO remove this method
        

class TheirAttackerAction():
    def __init__(self):
        self.name = "TheirAttackerAction"
        self.blackboard = Blackboard()

    def __call__(self, id):
        self.movement = NormalMovement(id) # TODO remove this class
        return self.movement.run()

    def run(self, id):
        self.movement = NormalMovement(id) # TODO remove this class
        return self.movement.run()
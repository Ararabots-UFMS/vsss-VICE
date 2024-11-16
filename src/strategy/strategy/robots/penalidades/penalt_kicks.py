from strategy.behaviour import LeafNode, Selector
from strategy.blackboard import Blackboard
from strategy.skill.route import NormalMovement, StraightMovement


        
class PenaltyKick(LeafNode):
    def __init__(self, name):
        super().__init__(name, [])
        self.blackboard = Blackboard()
        self.movement = StraightMovement()

    def __call__(self):
        if self.blackboard.gui.is_field_side_left:
            theta = 0
            return self.movement.moveToEnemyGoal(theta)
        return self.movement.moveToEnemyGoal()
    


class TheirActionAttacker():
    def __init__(self):
        self.name = "TheirAttackerAction"
        self.blackboard = Blackboard()

    def __call__(self):
        self.movement = NormalMovement() # TODO remove this class
        return self.movement.moveToPenaltyDefender()

    def run(self):
        self.movement = NormalMovement() # TODO remove this class
        return self.movement.moveToPenaltyDefender()

        


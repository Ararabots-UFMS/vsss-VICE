from strategy.behaviour import LeafNode, Selector
from strategy.blackboard import Blackboard
from strategy.skill.route import NormalMovement

"""Contains all FreeKickActions the robot must do (in order or not) during the match"""

# class MoveVertical(LeafNode):
#     def __init__(self, name):
#         super().__init__(name)
#         self.blackboard = Blackboard()

#     def run(self):
#         pass
    



# class OurAttackerAction(Selector):
#     def __init__(name, self):
#         super().__init__(name, [])
#         self.name = name
#         self.blackboard = Blackboard()

#     def run(self):
#         moveVertical = 
#         moveHorizontal =

#         self.add_children([moveVertical, moveHorizontal])



class OurAttackerAction():
    def __init__(self):
        self.name = "OurActionAttacker"
        self.blackboard = Blackboard()

    def __call__(self, **kwds):
        self.movement = NormalMovement()
        return self.movement.moveToCenter()

    def run(self):
        self.movement = NormalMovement()
        return self.movement.moveToCenter()


class TheirAttackerAction():
    def __init__(self):
        self.name = "TheirActionAttacker"
        self.blackboard = Blackboard()

    def __call__(self, **kwds):
        self.movement = NormalMovement()
        return self.movement.moveToCenter()
        
    def run(self):
        self.movement = NormalMovement()
        return self.movement.moveToCenter()
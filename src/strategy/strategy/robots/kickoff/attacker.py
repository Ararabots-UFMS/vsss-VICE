
from strategy.behaviour import LeafNode, Selector, TaskStatus
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

    def run(self):
        self.movement = NormalMovement()
        return self.movement.moveToCenter()


class CheckPosition(LeafNode):
    def __init__(self, name, robot_id):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.robot_x = self.blackboard.ally_robots[self.robot].position_x
        self.robot_y = self.blackboard.ally_robots[self.robot].position_y
        self.robot = robot_id
        self.movement = NormalMovement()

    def run(self):
        if self.blackboard.gui.is_field_side_left:
            if self.robot_x != -600 or self.robot_y != 0:
                return TaskStatus.SUCCESS, self.movement.outsideCenterCircle()
        elif self.robot_x != -600 or self.robot_y != 0:
            return TaskStatus.SUCCESS, self.movement.outsideCenterCircle()
        else:
            return TaskStatus.FAILURE, None

class TheirActionAttacker(Selector):
    def __init__(self,name, robot):
        super().__init__(name, [])

        self.name = "TheirActionAttacker"
        self.blackboard = Blackboard()

    def __call__(self, **kwds):
        self.movement = NormalMovement()
        return self.movement.outsideCenterCircle()
        
    def run(self):
        self.movement = NormalMovement()
        return self.movement.outsideCenterCircle()
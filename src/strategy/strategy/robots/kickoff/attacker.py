
from strategy.behaviour import LeafNode, Selector, Sequence, TaskStatus
from strategy.blackboard import Blackboard
from strategy.skill.route import BreakStrategy, NormalMovement

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


############################################################################

class CheckPosition(LeafNode):
    def __init__(self, name, robot_id):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.robot = robot_id
        self.robot_x = self.blackboard.ally_robots[self.robot].position_x
        self.robot_y = self.blackboard.ally_robots[self.robot].position_y
        self.movement = NormalMovement()

    def run(self):
        if self.blackboard.gui.is_field_side_left:
            if self.robot_x != -600 or self.robot_y != 0:
                return TaskStatus.SUCCESS, self.movement.outsideCenterCircle()
        elif self.robot_x != -600 or self.robot_y != 0:
            return TaskStatus.SUCCESS, self.movement.outsideCenterCircle()
        else:
            return TaskStatus.FAILURE, None

class MoveToCircle(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.movement = NormalMovement()

    def run(self):
        return TaskStatus.SUCCESS, self.movement.outsideCenterCircle()
    
class BallMovement(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.movement = BreakStrategy()
        self.ball_vx = self.blackboard.balls[0].velocity_x
        self.ball_vy = self.blackboard.balls[0].velocity_y
    
    def run(self):
        if round(abs(self.ball_vx)) != 1 or round(abs(self.ball_vy)) != 1:
            Blackboard().update_referee_start()
            return TaskStatus.SUCCESS, self.movement._break()
        
        return TaskStatus.FAILURE, None
    

class TheirActionAttacker(Selector):
    def __init__(self,name, robot):
        super().__init__(name, [])
        self.name = "TheirActionAttacker"
        self.robot_id = robot
        self.blackboard = Blackboard()

        move2circle = MoveToCircle("MoveToCircle")
        check_position = CheckPosition("CheckPosition", self.robot_id)

        check_ball_movement = BallMovement("BallMovement")

        move2position = Sequence("MoveToPosition", [check_position,move2circle])

        self.add_children([check_ball_movement, move2position])

    def __call__(self, **kwds):
        self.movement = NormalMovement()
        return self.movement.outsideCenterCircle()
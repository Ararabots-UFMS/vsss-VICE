
import math
from strategy.behaviour import LeafNode, Selector, Sequence, TaskStatus
from strategy.blackboard import Blackboard
from strategy.skill.route import BreakStrategy, GetInAngleStrategy, NormalMovement

"""Contains all RunningActions the robot must do (in order or not) during the match"""

class MoveToBall(LeafNode):
    def __init__(self,name):
        super().__init__(name)
        self.name = "OurActionAttacker"
        self.blackboard = Blackboard()
        self.movement = GetInAngleStrategy()
        self.ball_x = self.blackboard.balls[0].position_x
        self.ball_y = self.blackboard.balls[0].position_y
        self.robot_x = self.blackboard.ally_robots[0].position_x
        self.robot_y = self.blackboard.ally_robots[0].position_y

    def run(self):
        theta, b = self.draw_line()

        if self.blackboard.gui.is_field_side_left:
            return TaskStatus.SUCCESS, self.movement.run(self.ball_x, self.ball_y, theta)
        else:
            #TODO angle is with issues!!!
            theta = theta + math.pi
            return TaskStatus.SUCCESS, self.movement.run(self.ball_x, self.ball_y, theta)
    
    def draw_line(self):
        
        if self.ball_x == self.robot_x:
            b = self.ball_x
            return 0, b
        
        m = (self.robot_y - self.ball_y)/(self.robot_x - self.ball_x)
        b = self.ball_y - m * self.ball_x
        theta = math.atan(m)

        return theta, b    

class CheckBallDistance(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.ball_position_x = self.blackboard.balls[0].position_x
        self.ball_position_y = self.blackboard.balls[0].position_y
        self.position_x = self.blackboard.ally_robots[0].position_x
        self.position_y = self.blackboard.ally_robots[0].position_y
        self.radius = 122

    def run(self):

        distance = math.sqrt((self.position_x - self.ball_position_x) ** 2 + (self.position_y - self.ball_position_y) ** 2)

        if distance > self.radius:
            print(f"Estou longe da bola {distance}")
            return TaskStatus.SUCCESS, None
        else:
            print(f"Estou perto da bola {distance}")
            return TaskStatus.FAILURE, None
        

class CheckGoalDistance(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.goal_position_x = 2250
        self.goal_position_y = 0
        self.position_x = self.blackboard.ally_robots[0].position_x
        self.position_y = self.blackboard.ally_robots[0].position_y
        self.radius = 675

    def run(self):

        distance = math.sqrt((self.position_x - self.goal_position_x) ** 2 + (self.position_y - self.goal_position_y) ** 2)

        if distance > self.radius:
            return TaskStatus.SUCCESS, None
        else:
            return TaskStatus.FAILURE, None
        
class MoveToGoal(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.goal_position_y = 0
        self.movement = GetInAngleStrategy()
        self.theta = 0
        if self.blackboard.gui.is_field_side_left:
            self.goal_position_x = 2250
        else:
            self.goal_position_x = -2250

    def run(self):
        if self.blackboard.gui.is_field_side_left:
            return TaskStatus.SUCCESS, self.movement.run(self.goal_position_x, self.goal_position_y, self.theta)
        else:
            self.theta = math.pi
            return TaskStatus.SUCCESS, self.movement.run(-self.goal_position_x, self.goal_position_y, self.theta)


class ShootBall(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.movement = BreakStrategy()

    def run(self):
        return TaskStatus.SUCCESS, self.movement._break()

class OurActionAttacker(Selector):
    def __init__(self, name):
        super().__init__(name, [])

        move2ball = MoveToBall("MoveToBall")
        is_near_ball = CheckBallDistance("CheckBallDistance")
        is_near_goal = CheckGoalDistance("CheckGoalDistance")
        move2goal = MoveToGoal("MoveToGoal")
        shoot_ball = ShootBall("ShootBall")

        going_to_ball = Sequence("GoingToBall", [is_near_ball, move2ball])
        going_to_goal = Sequence("GoingToGoal", [is_near_goal, move2goal])

        prepare2shoot = Selector("Prepare2Shoot", [going_to_ball, going_to_goal])

        self.add_children([prepare2shoot, shoot_ball])

    def __call__(self):
        return super().run()[1]


# class TheirActionAttacker():
#     def __init__(self):
#         self.name = "TheirActionAttacker"
#         self.blackboard = Blackboard()

#     def __call__(self):
#         self.movement = NormalMovement()
#         return self.movement.outsideCenterCircle()
        
#     def run(self):
#         self.movement = NormalMovement()
#         return self.movement.outsideCenterCircle()
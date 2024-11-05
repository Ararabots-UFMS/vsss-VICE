import math
from strategy.behaviour import LeafNode, Selector, Sequence, TaskStatus
from strategy.blackboard import Blackboard
from strategy.skill.route import BreakStrategy, GetInAngleStrategy, NormalMovement

"""Contains all FreeKickActions the robot must do (in order or not) during the match"""

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
        m, b = self.draw_line()
        theta = math.atan(m)

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

        return m, b
    
class CheckBallDistance(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.ball_position_x = self.blackboard.balls[0].position_x
        self.ball_position_y = self.blackboard.balls[0].position_y
        self.position_x = self.blackboard.ally_robots[0].position_x
        self.position_y = self.blackboard.ally_robots[0].position_y
        self.radius = 172
        self.movement = BreakStrategy() 

    def run(self):

        distance = math.sqrt((self.position_x - self.ball_position_x) ** 2 + (self.position_y - self.ball_position_y) ** 2)

        if distance > self.radius:
            print(f"Estou longe da bola: {distance}")
            return TaskStatus.SUCCESS, None
        else:
            print(f"Estou perto da bola : {distance}")
            return TaskStatus.FAILURE, self.movement._break()
            
class OurAttackerAction(Sequence):
    def __init__(self, name):
        super().__init__(name, [])

        move2ball = MoveToBall("MoveToBall")
        check_distance = CheckBallDistance("CheckBallDistance")

        self.add_children([check_distance, move2ball])

    def __call__(self):
        return super().run()[1]



class TheirAttackerAction():
    def __init__(self):
        self.name = "TheirActionAttacker"
        self.blackboard = Blackboard()

    def __call__(self):
        self.movement = NormalMovement()
        return self.movement.moveToCenter()
        
    def run(self):
        self.movement = NormalMovement()
        return self.movement.moveToCenter()
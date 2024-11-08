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
        self.radius = 200

        if self.blackboard.gui.is_field_side_left: 

            for line in self.blackboard.geometry.field_lines:
                if line.name == 'RightGoalLine':
                    self.goal_field = line
        else:

            for line in self.blackboard.geometry.field_lines:
                if line.name == 'LeftGoalLine':
                    self.goal_field = line


        self.ball_x = self.blackboard.balls[0].position_x
        self.ball_y = self.blackboard.balls[0].position_y

    def run(self):
        m, b = self.draw_line()

        if self.blackboard.gui.is_field_side_left:
            theta = math.atan(m)
        else:
            theta = math.atan(m) + math.pi 

        x_d, y_d = self.search_point(theta) 

        print(f"position x_d : {-x_d}")
        print(f"position y_d : {-y_d}")
        print(f"theta : {theta}")


        return TaskStatus.SUCCESS, self.movement.run(-x_d, -y_d, theta)
    
    def draw_line(self):

        if self.ball_y == 0: # Considerando y = 0 parar ir ao meio do gol. 
            b = self.ball_y
            return 0, b

        m = (-self.ball_y)/(self.goal_field.x1 - self.ball_x)
        b = self.ball_y - m * self.ball_x

        return m, b
    
    def search_point(self, theta):
        ball_x = self.blackboard.balls[0].position_x
        ball_y = self.blackboard.balls[0].position_y

        sin_theta = self.radius * math.sin(theta)
        cos_theta = self.radius * math.cos(theta)

        y_d = sin_theta + -1 * ball_y
        x_d = cos_theta + -1 * ball_x

        return x_d, y_d
    
class CheckBallDistance(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.ball_position_x = self.blackboard.balls[0].position_x
        self.ball_position_y = self.blackboard.balls[0].position_y
        self.position_x = self.blackboard.ally_robots[0].position_x
        self.position_y = self.blackboard.ally_robots[0].position_y
        self.radius = 300 # 90 de raio do robo + 22 da bola + 50 de folga   
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
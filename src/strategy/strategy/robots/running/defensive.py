
import math
from strategy.behaviour import LeafNode, Selector, TaskStatus
from strategy.blackboard import Blackboard
from strategy.coach.running.Defense_play import DefensivePlay
from strategy.skill.route import BreakStrategy, GetInAngleStrategy

class DefensePosition(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.movement = GetInAngleStrategy()
        

    def run(self):
        pass
    

class CheckBallDistance(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.movement = BreakStrategy()
        self.ball_position_x = self.blackboard.balls[0].position_x
        self.ball_position_y = self.blackboard.balls[0].position_y
        self.position_x = self.blackboard.ally_robots[0].position_x
        self.position_y = self.blackboard.ally_robots[0].position_y
        self.radius = 142

    def run(self):

        distance = math.sqrt((self.position_x - self.ball_position_x) ** 2 + (self.position_y - self.ball_position_y) ** 2)

        if distance > self.radius:
            print(f"Estou longe da bola : {distance}")
            return TaskStatus.SUCCESS, None
        else:
            print(f"Estou perto da bola {distance}")
            return TaskStatus.FAILURE, self.movement._break()
        


class OurActionDefender(Selector):
    def __init__(self, name, points):
        super().__init__(name, [])
        self.blackboard = Blackboard()
        is_near_ball = CheckBallDistance("CheckBallDistance")
        defensive_mode = DefensePosition("DefensivePosition")
        self.add_children([is_near_ball, defensive_mode])
    
    def __call__(self):
        return super().run()[1]
        


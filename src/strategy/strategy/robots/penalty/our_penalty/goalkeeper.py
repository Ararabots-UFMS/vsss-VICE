import math
from strategy.behaviour import LeafNode, Selector, TaskStatus
from strategy.blackboard import Blackboard
from strategy.skill.route import BreakStrategy, GetInAngleStrategy

class DefensiveTriangle():
    def __init__(self):
        self.blackboard = Blackboard()
        self.point_1 = [-1750, 40]
        self.point_2 = [-1750, -40]
        self.point_3 = [self.blackboard.balls[0].position_x,self.blackboard.balls[0].position_y]


class DefensePosition(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.movement = GetInAngleStrategy()
        self.ball_x = self.blackboard.balls[0].position_x
        self.ball_y = self.blackboard.balls[0].position_y
        self.minimal_distance = 300
        self.goal_y = None
        if self.blackboard.gui.is_field_side_left:
            self.goal_x = -2250
        else:
            self.goal_x = 2250
        

    def run(self):
        distance, id = self.closest_enemy_with_ball()
        m, n = self.draw_line(id)
        self.find_point_in_goal(m, n)
        theta = math.atan(m)

        # return TaskStatus.SUCCESS, self.movement.run(self.goal_x, self.goal_y, theta)
        #TODO: Maybe we need to change this movement to something more simple

        if distance > self.minimal_distance:
            return TaskStatus.SUCCESS, self.movement.run(self.goal_x, self.goal_y, theta)
        else:
            return TaskStatus.SUCCESS, self.movement.run(self.ball_x, self.ball_y, theta)

    def find_point_in_goal(self, m, n):
        self.goal_y = m*self.goal_x + n
        

    def draw_line(self, id):
        self.robot_x = self.blackboard.enemy_robots[id].position_x
        self.robot_y = self.blackboard.enemy_robots[id].position_y
        
        if self.ball_x == self.robot_x:
            n = self.ball_x
            return 0, n
        
        m = (self.robot_y - self.ball_y)/(self.robot_x - self.ball_x)
        n = self.ball_y - m * self.ball_x

        return m, n    
    
    def closest_enemy_with_ball(self):
        distance = +math.inf
        enemy_id = None
        enemy_robots = self.blackboard.enemy_robots
        for enemy in list(self.blackboard.enemy_robots):
            enemy_distance = math.sqrt((enemy_robots[enemy].position_x - self.ball_x) ** 2 + (enemy_robots[enemy].position_x - self.ball_y) ** 2)
            if enemy_distance <= distance:
                distance = enemy_distance
                enemy_id = enemy
        
        return distance, enemy_id
    

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
        


class OurGoalkeeperAction(Selector):
    def __init__(self, name):
        super().__init__(name, [])
        self.blackboard = Blackboard()
        # is_near_ball = CheckBallDistance("CheckBallDistance")
        defensive_mode = DefensePosition("DefensivePosition")
        self.add_children([defensive_mode])
    
    def __call__(self):
        return super().run()[1]
        
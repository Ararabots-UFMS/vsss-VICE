
import math
from strategy.behaviour import LeafNode, Selector, Sequence, TaskStatus
from strategy.blackboard import Blackboard
from strategy.coach.running.Defense_play import DefensivePlay
from strategy.skill.route import BreakStrategy, GetInAngleStrategy, NormalMovement

class DefensePosition(LeafNode):
    def __init__(self, name, point):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.movement = NormalMovement()
        self.point = point
        

    def run(self):
        # print(f"indo para o ponto : {self.point}")
        return TaskStatus.SUCCESS, self.movement.move_to_position_with_orientation(self.point[0], self.point[1], self.point[2])
    

class CheckBallDistance(LeafNode):
    def __init__(self, name, point, robot_id):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.movement = NormalMovement()
        self.ball = self.blackboard.balls[0] # refatorar dps 
        self.ball_position_x = self.blackboard.balls[0].position_x
        self.ball_position_y = self.blackboard.balls[0].position_y
        self.position_x = self.blackboard.ally_robots[robot_id].position_x
        self.position_y = self.blackboard.ally_robots[robot_id].position_y
        self.point = point
        self.radius = 172 # raio do robo 
         

    def run(self):

        distance = math.sqrt((self.position_x - self.ball_position_x) ** 2 + (self.position_y - self.ball_position_y) ** 2)

        if distance > self.radius:
            print(f"Estou longe da bola : {distance}")
            return TaskStatus.FAILURE, None
        else:
            print(f"Estou perto da bola {distance}")
            return TaskStatus.SUCCESS, None

class CheckForEnemies(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.movement = NormalMovement()
        self.ball_position_x = self.blackboard.balls[0].position_x
        self.ball_position_y = self.blackboard.balls[0].position_y
        self.goal_position_x = 2250
        self.goal_position_y = 0
        self.theta = 0


    def run(self):

        for enemy in self.blackboard.enemy_robots:
            distance = math.sqrt((self.blackboard.enemy_robots[enemy].position_x - self.ball_position_x) ** 2 + (self.blackboard.enemy_robots[enemy].position_y - self.ball_position_y) ** 2)
            if distance <= 100:
                return TaskStatus.SUCCESS, self.movement.move2point(0, self.ball_position_y)
        
        return TaskStatus.SUCCESS, self.movement.moveToEnemyGoal(self.goal_position_x, self.goal_position_y, self.theta)

class OurActionDefender(Selector):
    def __init__(self, name, points, robot_id):
        super().__init__(name, [])
        self.blackboard = Blackboard()
        self.point = points
        is_near_ball = CheckBallDistance("CheckBallDistance", self.point, robot_id)
        is_there_enemies = CheckForEnemies("CheckForEnemies")
        react_to_ball = Sequence("ReactBall", [is_near_ball, is_there_enemies]) 
        defensive_mode = DefensePosition("DefensivePosition", self.point)
        self.add_children([react_to_ball, defensive_mode])
    
    def __call__(self):
        return super().run()[1]
        
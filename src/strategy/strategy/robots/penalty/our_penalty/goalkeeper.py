import math
from strategy.behaviour import LeafNode, Selector, TaskStatus
from strategy.blackboard import Blackboard
from strategy.skill.route import BreakStrategy, GetInAngleStrategy, NormalMovement
from movement.obstacles.static_obstacles import PenaltyAreaObstacles


class DefensePosition(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.movement = NormalMovement()
        self.ball = self.blackboard.balls[0]
        self.minimal_distance = 300
        self.padding = 150
        self.goal_y = None
        self.penalty_area = PenaltyAreaObstacles(self.blackboard.geometry)

        # Goal bounds. Do not have on blackboard yet 
        self.goal_bound_y1 = 350 #  Top limit of the goal
        self.goal_bound_y2 = -350 # Bottom limit of the goal


        if self.blackboard.gui.is_field_side_left:
            for line in self.blackboard.geometry.field_lines:
                if line.name == 'LeftGoalLine':
                    self.goal_x = line.x1 + self.padding
                elif line.name == 'LeftPenaltyStretch':
                    self.penalty_stretch_x = line.x1                     

        else:
            for line in self.blackboard.geometry.field_lines:
                if line.name == 'RightGoalLine':
                    self.goal_x = line.x1 - self.padding
                elif line.name == 'RightPenaltyStretch':
                    self.penalty_stretch_x = line.x1

    def run(self):
        
        enemy_distance, enemy_id = self.closest_enemy_with_ball()

        m, b, theta = self.draw_line(enemy_id)
        self.find_point_in_goal(m, b)

        distance_ball_goal = self.calculate_distance_to_ball_goal()

        if self.check_for_ball_in_defense_area(): 
            return TaskStatus.SUCCESS, self.movement.move_to_position_with_orientation_no_obstacle(self.ball.position_x, self.ball.position_y, theta)
        else:
            return TaskStatus.SUCCESS, self.movement.move_to_position_with_orientation_no_obstacle(self.goal_x, self.goal_y, theta)


    def check_for_ball_in_defense_area(self):

        if self.penalty_area.is_colission([self.ball.position_x, self.ball.position_y]):
            # We need to know wich side of field the ball had collission
            if self.blackboard.gui.is_field_side_left == True and self.blackboard.balls[0].position_x < 0:
                return True
            elif self.blackboard.gui.is_field_side_left == False and self.blackboard.balls[0].position_x > 0:
                return True
        
        return False

    def calculate_distance_to_ball_goal(self):
        return math.sqrt((self.ball.position_x - self.penalty_stretch_x) ** 2 + (self.ball.position_y) ** 2)


    def find_point_in_goal(self, m, n):
        self.goal_y = m*self.goal_x + n

        if self.goal_y > self.goal_bound_y1:
            self.goal_y = self.goal_bound_y1
        elif self.goal_y < self.goal_bound_y2:
            self.goal_y = self.goal_bound_y2
        

    def draw_line(self, id):
        self.robot_x = self.blackboard.enemy_robots[id].position_x
        self.robot_y = self.blackboard.enemy_robots[id].position_y
        
        if self.ball.position_x == self.robot_x:
            n = self.ball.position_x
            return 0, n 

        m = (self.robot_y - self.ball.position_y) / (self.robot_x - self.ball.position_x)
        b = self.ball.position_y - m * self.ball.position_x

        theta = math.atan2(self.robot_y - self.ball.position_y, self.robot_x - self.ball.position_x)

        return m, b, theta 
    
    def closest_enemy_with_ball(self):
        distance = +math.inf
        enemy_id = None
        enemy_robots = self.blackboard.enemy_robots
        for enemy in list(self.blackboard.enemy_robots):
            enemy_distance = math.sqrt((enemy_robots[enemy].position_x - self.ball.position_x) ** 2 + (enemy_robots[enemy].position_y - self.ball.position_y) ** 2)
            if enemy_distance <= distance:
                distance = enemy_distance
                enemy_id = enemy
        
        return distance, enemy_id
    
# TODO : Check if the robot is near the ball
class CheckBallDistance(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.movement = BreakStrategy()
        self.ball_position_x = self.blackboard.balls[0].position_x
        self.ball_position_y = self.blackboard.balls[0].position_y

        if self.blackboard.gui._is_team_color_yellow:
            id_goalkeeper = self.blackboard.referee.teams[1].goalkeeper
            self.goalkeeper = self.blackboard.ally_robots[id_goalkeeper]
        else:
            id_goalkeeper = self.blackboard.referee.teams[0].goalkeeper
            self.goalkeeper = self.blackboard.ally_robots[id_goalkeeper]

        self.radius = 142

    def run(self):

        distance = math.sqrt((self.goalkeeper.position_x - self.ball_position_x) ** 2 + (self.goalkeeper.position_y - self.ball_position_y) ** 2)

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
        defensive_mode = DefensePosition("DefensivePosition")
        self.add_children([defensive_mode])
    
    def __call__(self):
        return super().run()[1]
        
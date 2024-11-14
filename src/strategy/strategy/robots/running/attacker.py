import math
from strategy.behaviour import LeafNode, Selector, Sequence, TaskStatus
from strategy.blackboard import Blackboard
from strategy.skill.route import BreakStrategy, GetInAngleStrategy, NormalMovement, SpinStrategy

"""Contains all RunningActions the robot must do (in order or not) during the match"""

class MoveToBall(LeafNode):
    def __init__(self,name, robot):
        super().__init__(name)
        self.name = "OurActionAttacker"
        self.blackboard = Blackboard()
        self.movement = GetInAngleStrategy()
        # self.movement = NormalMovement()
        self.radius = 112 # raio do robo + raio da bola

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
        self.position_x = self.blackboard.ally_robots[robot].position_x

    def run(self):
        theta = self.draw_line()

        x_d, y_d = self.search_point(theta)

        # if the robot is in front of the ball, so the wanted point must be more distance.
        if self.blackboard.gui.is_field_side_left:
            if self.ball_x < self.position_x:
                x_d += 100
        elif self.ball_x > self.position_x:
            x_d -= 100

        # print(f"position x_d : {-x_d}")
        # print(f"position y_d : {-y_d}")
        # print(f"theta : {theta}")
        print("Indo até a bola")
        # Yeh, I don't know why, but if the points were positive, this doesn't work
        return TaskStatus.SUCCESS, self.movement.moveToBall(-x_d, -y_d, theta)
    
    def draw_line(self):

        if self.ball_y == 0: # Considerando y = 0 parar ir ao meio do gol. 
            b = self.ball_y
            return 0, b

        m = (-self.ball_y)/(self.goal_field.x1 - self.ball_x)
        b = self.ball_y - m * self.ball_x

        #Huge mamaco, refactor this
        if self.blackboard.gui.is_field_side_left:
            theta = math.atan2((-self.ball_y), (self.goal_field.x1 - self.ball_x))
        else:
            theta = math.atan2((-self.ball_y), (self.goal_field.x1 - self.ball_x)) + math.pi 

        return theta
    
    def search_point(self, theta):
        ball_x = self.blackboard.balls[0].position_x
        ball_y = self.blackboard.balls[0].position_y

        sin_theta = self.radius * math.sin(theta)
        cos_theta = self.radius * math.cos(theta)

        y_d = sin_theta + -1 * ball_y
        x_d = cos_theta + -1 * ball_x

        return x_d, y_d

class CheckBallDistance(LeafNode):
    def __init__(self, name, robot):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.robot = robot
        self.ball_position_x = self.blackboard.balls[0].position_x
        self.ball_position_y = self.blackboard.balls[0].position_y
        self.position_x = self.blackboard.ally_robots[robot].position_x
        self.position_y = self.blackboard.ally_robots[robot].position_y
        self.radius = 120

    def run(self):

        distance = math.sqrt((self.position_x - self.ball_position_x) ** 2 + (self.position_y - self.ball_position_y) ** 2)

        if distance > self.radius:
            print(f"Estou longe da bola {distance}")
            return TaskStatus.SUCCESS, None
        else:
            print(f"Estou perto da bola {distance}")
            # self.blackboard.ally_robots[robot].
            return TaskStatus.FAILURE, None
        

class CheckGoalDistance(LeafNode):
    def __init__(self, name, robot):
        super().__init__(name)
        self.blackboard = Blackboard()
        if self.blackboard.gui.is_field_side_left:
            self.goal_position_x = 2250
        else:
            self.goal_position_x = -2250

        self.goal_position_y = 0
        self.position_x = self.blackboard.ally_robots[robot].position_x
        self.position_y = self.blackboard.ally_robots[robot].position_y
        self.radius = 675 # eh o raio da area que o nosso robo deve chutar a bola

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
        # self.movement = GetInAngleStrategy()
        self.movement = NormalMovement()
        self.theta = 0

        #trocar para field lines dps
        if self.blackboard.gui.is_field_side_left:
            self.goal_position_x = 2250 - 450 # linha do gol em x - padding de 450 
        else:
            self.goal_position_x = -2250 + 450 # linha do gol em x + padding de 450 

    def run(self):
        print("Indo até o gol")
        if self.blackboard.gui.is_field_side_left:
            # self.blackboard.activate_kick()
            return TaskStatus.SUCCESS, self.movement.moveToEnemyGoal(self.goal_position_x, self.goal_position_y, self.theta)
        else:
            # self.blackboard.activate_kick()
            self.theta = math.pi
            return TaskStatus.SUCCESS, self.movement.moveToEnemyGoal(-self.goal_position_x, self.goal_position_y, self.theta)

#It must be used with we dont have a kick:

class SpinBall(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.movement = SpinStrategy()
        self.enemy_goalkeeper_id = 0
        if self.blackboard.gui.is_team_color_yellow:
            self.enemy_color = 0 # blue color in list
        else:
            self.enemy_color = 1 # Yellow color in list
        self.is_left = False
    
    def where_is_enemy_goalkeeper(self):
        try:
            for enemy in self.blackboard.enemy_robots:
                if enemy == self.blackboard.referee.teams[self.enemy_color].goalkeeper:
                    self.enemy_goalkeeper_id = enemy
            
            if round(self.blackboard.enemy_robots[self.enemy_goalkeeper_id].position_y) < 0:
                self.is_left = True 

            return self.is_left

        except:
            return self.is_left
        
    def run(self):
        self.where_is_enemy_goalkeeper()
        return TaskStatus.SUCCESS, self.movement.spin(self.is_left)


#It must be used with we have a kick:

class ShootBall(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.movement = BreakStrategy()
        self.blackboard = Blackboard()

    def run(self):
        self.blackboard.activate_kick()
        return TaskStatus.SUCCESS, self.movement._break()

class OurActionAttacker(Selector):
    def __init__(self, name, robot_id):
        super().__init__(name, [])

        move2ball = MoveToBall("MoveToBall", robot_id)
        is_near_ball = CheckBallDistance("CheckBallDistance", robot_id)
        is_near_goal = CheckGoalDistance("CheckGoalDistance", robot_id)
        move2goal = MoveToGoal("MoveToGoal")

        # Types of kicks:

        shoot_ball = ShootBall("ShootBall")
        spin_ball = SpinBall("SpinBall")

        going_to_ball = Sequence("GoingToBall", [is_near_ball, move2ball])
        going_to_goal = Sequence("GoingToGoal", [is_near_goal, move2goal])

        prepare2shoot = Selector("Prepare2Shoot", [going_to_ball, going_to_goal])

        # self.add_children([prepare2shoot, shoot_ball])
        self.add_children([prepare2shoot, spin_ball])

    def __call__(self):
        return super().run()[1]

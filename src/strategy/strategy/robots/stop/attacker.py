from strategy.behaviour import LeafNode, Selector, TaskStatus, Sequence
from strategy.blackboard import Blackboard
from strategy.skill.route import BreakStrategy, GetInAngleStrategy, NormalMovement, StraightMovement
import math
"""Contains all StopActions the robot must do (in order or not) during the match"""
    
class StopAction(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.name = name
        self.blackboard = Blackboard()
        self.movement = BreakStrategy()

    def run(self):
        return TaskStatus.SUCCESS, self.movement._break()

class CheckDistance(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.name = "CheckDistance"
        self.blackboard = Blackboard()

    def run(self):
        position_x = self.blackboard.ally_robots[0].position_x
        position_y = self.blackboard.ally_robots[0].position_y
        ball_x = self.blackboard.balls[0].position_x
        ball_y = self.blackboard.balls[0].position_y
        radius = 612

        distance = math.sqrt((position_x - ball_x) ** 2 + (position_y - ball_y) ** 2)

        if distance <= radius:
            # print(f"distance <- radius : {distance}")
            return TaskStatus.SUCCESS, None
        else:
            # print(f"distance > radius : {distance}")
            return TaskStatus.FAILURE, None
        
class GoAway(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.name = "GoAway"
        self.blackboard = Blackboard()
        self.movement = GetInAngleStrategy()

    def run(self):
        m, b = self.draw_line()
        if self.blackboard.gui.is_field_side_left:
            theta = math.atan(m)
        else:
            theta = math.atan(m) + math.pi
        x_d,y_d = self.search_point(theta)

        # print(f"position x_d : {-x_d}")
        # print(f"position y_d : {-y_d}")
        # print(f"theta : {theta}")

        return TaskStatus.SUCCESS, self.movement.run(-x_d, -y_d, theta)

    def draw_line(self):
        ball_x = self.blackboard.balls[0].position_x
        ball_y = self.blackboard.balls[0].position_y
        
        if self.blackboard.gui.is_field_side_left:
            goal_center_x = 2160
        else:
            goal_center_x = -2160

        if ball_x == goal_center_x:
            b = ball_x
            return 0, b
        
        m = -ball_y/(goal_center_x-ball_x)
        b = ball_y - m * ball_x

        return m, b
    
    def search_point(self, theta):
        distance_needed = 612
        ball_x = self.blackboard.balls[0].position_x
        ball_y = self.blackboard.balls[0].position_y
        y_d = distance_needed * math.sin(theta) + ball_y
        x_d = distance_needed * math.cos(theta) + ball_x

        return x_d, y_d


class AttackerAction(Selector):
    def __init__(self, name):
        super().__init__(name, [])

        check_distance = CheckDistance("CheckDistance")

        go_away = GoAway("GoAway")

        is_close_to_ball = Sequence("IsCloseToBall", [check_distance, go_away])

        stop = StopAction("StopAction")

        self.add_children([is_close_to_ball, stop])
    
    def __call__(self):
        return super().run()[1]
    

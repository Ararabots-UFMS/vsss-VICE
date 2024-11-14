import math
from strategy.blackboard import Blackboard


class DefenseTriangle():
    def __init__(self):
        self.blackboard = Blackboard()
        ball_x = self.blackboard.balls[0].position_x
        ball_y = self.blackboard.balls[0].position_y
        self.point_ball = [ball_x,ball_y]
        if self.blackboard.gui.is_field_side_left:
            self.point_left = [-2250,400]
            self.point_rigth = [-2250,-400]
        else:
            self.point_left = [2250,400]
            self.point_rigth = [2250,-400]
    
    def draw_line_left(self):
        if self.point_left[1] == self.point_ball[1]:
            n = self.point_ball[1]
            return 0, n
        
        m = (self.point_left[1] - self.point_ball[1])/(self.point_left[0] - self.point_ball[0])
        n = self.point_ball[1] - m * self.point_ball[0]
        return m, n    
    
    def draw_line_right(self):
        if self.point_rigth[1] == self.point_ball[1]:
            n = self.point_ball[1]
            return 0, n 
        
        m = (self.point_rigth[1] - self.point_ball[1])/(self.point_rigth[0] - self.point_ball[0])
        n = self.point_ball[1] - m * self.point_ball[0]
        return m, n   
    
class PenaltyArea():
    def __init__(self):
        self.blackboard = Blackboard()
        if self.blackboard.gui.is_field_side_left:
            self.point_A = [-2250,675]
            self.point_B = [-1750,675]
            self.point_C = [-2250,-675]
            self.point_D = [-1750,-675]
        else:
            self.point_A = [2250,675]
            self.point_B = [1750,675]
            self.point_C = [2250,-675]
            self.point_D = [1750,-675]

    def draw_line_left(self):
        
        m = (self.point_B[1] - self.point_A[1])/(self.point_B[0] - self.point_A[0])
        n = self.point_A[1] - m * self.point_A[0]
        return m, n   
    
    def draw_line_rigth(self):
        
        m = (self.point_D[1] - self.point_C[1])/(self.point_D[0] - self.point_C[0])
        n = self.point_C[1] - m * self.point_C[0]
        return m, n  
    
    def draw_line_front(self):
        x = self.point_B[0]
        return x  
    

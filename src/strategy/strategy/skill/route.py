import math
from movement.obstacles.dynamic_obstacles import BallObstacle, RobotObstacle
from movement.obstacles.static_obstacles import PenaltyAreaObstacles
from movement.path.path_profiles import MovementProfiles, DirectionProfiles, SpinProfile
from strategy.blackboard import Blackboard
from math import pi


"""This file should have a properly class to use the Movement classes, look the example below"""

class NormalMovement():
    """This class should be a strategy skill to move a robot to a especific point"""
    def __init__(self):
        super().__init__()
        self.blackboard = Blackboard()
        self.obstacle_list = []

        # Populate obstacle_list with RobotObstacle instances for each enemy
        for enemy_id, enemy in self.blackboard.enemy_robots.items():
            self.obstacle_list.append(RobotObstacle(enemy))

        # Check if obstacle_list is empty
        if len(self.obstacle_list) == 0:
            pass  # Optional: add handling for the case when there are no enemy robots



    # theta is radians and position(x,y) is millimeter

    def moveToEnemyGoal(self, p_x, p_y, theta):
        """Moviment to the enemy goal"""
        return {"obstacles" : [PenaltyAreaObstacles(self.blackboard.geometry)],
                "path_profile" : MovementProfiles.Normal,
                "orientation_profile": DirectionProfiles.Aim,
                "sync" : False,
                "path_kwargs" : {"goal_state" : (p_x,p_y)},
                "orientation_kwargs" : {"theta" : theta}}


    def move_to_position_with_orientation(self, p_x, p_y, theta):
        return {"obstacles" : [PenaltyAreaObstacles(self.blackboard.geometry)],
                "path_profile" : MovementProfiles.Normal,
                "orientation_profile": DirectionProfiles.Aim,
                "sync" : False,
                "path_kwargs" : {"goal_state" : (p_x,p_y)},
                "orientation_kwargs" : {"theta" : theta}}
    
    def move_to_position_goalkeeper(self, p_x, p_y, theta):
        return {"obstacles" : [],
                "path_profile" : MovementProfiles.Normal,
                "orientation_profile": DirectionProfiles.Aim,
                "sync" : False,
                "path_kwargs" : {"goal_state" : (p_x,p_y)},
                "orientation_kwargs" : {"theta" : theta}} 
    
    def move_to_position_with_orientation_no_obstacle(self, p_x, p_y, theta):
        return {"obstacles" : [],
                "path_profile" : MovementProfiles.Normal,
                "orientation_profile": DirectionProfiles.Aim,
                "sync" : False,
                "path_kwargs" : {"goal_state" : (p_x,p_y)},
                "orientation_kwargs" : {"theta" : theta}}


    def move2point(self, p_x, p_y):

        return {"obstacles" : [],
                "path_profile" : MovementProfiles.Normal,
                "orientation_profile": DirectionProfiles.Aim,
                "sync" : False,
                "path_kwargs" : {"goal_state" : (p_x,p_y)},
                "orientation_kwargs" : {"theta" : 0}}

    def moveToCenter(self):   

        """Moviment to point when the robot is not goalkeeper"""
        # This need the PenaltyArea()
        if self.blackboard.gui._is_field_side_left:
            return {"obstacles" : [PenaltyAreaObstacles(self.blackboard.geometry)],
                    "path_profile" : MovementProfiles.Normal,
                    "orientation_profile": DirectionProfiles.Aim,
                    "sync" : False,
                    "path_kwargs" : {"goal_state" : (-112.0,0.0)},
                    "orientation_kwargs" : {"theta" : 0}}
        else:
            #TODO theta is a not perfect, so I add 2.1 to adjust the error
            return {"obstacles" : [PenaltyAreaObstacles(self.blackboard.geometry)],
                    "path_profile" : MovementProfiles.Normal,
                    "orientation_profile": DirectionProfiles.Aim,
                    "sync" : False,
                    "path_kwargs" : {"goal_state" : (122.0,0.0)},
                    "orientation_kwargs" : {"theta" : pi}}
    
    def centerLineGoal(self):
        
        """"Moviment to the center of our goal"""

        if self.blackboard.gui._is_field_side_left:
            return {"obstacles" : [],
                    "path_profile" : MovementProfiles.Normal,
                    "orientation_profile": DirectionProfiles.Aim,
                    "sync" : False,
                    "path_kwargs" : {"goal_state" : (-2160,0)},
                    "orientation_kwargs" : {"theta" : 0}}
        else:
            return {"obstacles" : [],
                    "path_profile" : MovementProfiles.Normal,
                    "orientation_profile": DirectionProfiles.Aim,
                    "sync" : False,
                    "path_kwargs" : {"goal_state" : (2160,0)},
                    "orientation_kwargs" : {"theta" : pi}}
    
    def outsideCenterCircle(self):
        """"Moviment to the center of our goal"""

        if self.blackboard.gui._is_field_side_left:
            return {"obstacles" : [],
                    "path_profile" : MovementProfiles.Normal,
                    "orientation_profile": DirectionProfiles.Aim,
                    "sync" : False,
                    "path_kwargs" : {"goal_state" : (-600,0)},
                    "orientation_kwargs" : {"theta" : 0}}
        else:
            return {"obstacles" : [],
                    "path_profile" : MovementProfiles.Normal,
                    "orientation_profile": DirectionProfiles.Aim,
                    "sync" : False,
                    "path_kwargs" : {"goal_state" : (600,0)},
                    "orientation_kwargs" : {"theta" : pi}}
      
        
    def moveToPenaltyKicker(self):

        """Moviment to central area, acording with the Penalty requirements"""

        # Penalty mark: at the goal to goal mark (y = 0) 3m away from the goal (x = -750 or x = +750)
        if self.blackboard.gui._is_field_side_left: # point (-750,0) turned to right
            return {"obstacles" : [],
                "path_profile" : MovementProfiles.Normal,
                "orientation_profile": DirectionProfiles.Aim,
                "sync" : False,
                "path_kwargs" : {"goal_state" : (-660,0)},
                "orientation_kwargs" : {"theta" : 0}}
        else: # point(750,0), turned to left
            return {"obstacles" : [],
                "path_profile" : MovementProfiles.Normal,
                "orientation_profile": DirectionProfiles.Aim,
                "sync" : False,
                "path_kwargs" : {"goal_state" : (660,0)},
                "orientation_kwargs" : {"theta" : pi}}
    
    def moveToPenaltyDefender(self):
        if self.blackboard.gui._is_field_side_left:
            return {"obstacles" : [],
                "path_profile" : MovementProfiles.Normal,
                "orientation_profile": DirectionProfiles.Aim,
                "sync" : False,
                "path_kwargs" : {"goal_state" : (1250,0)},
                "orientation_kwargs" : {"theta" : pi}}
        else: # point(750,0), turned to left
            return {"obstacles" : [],
                "path_profile" : MovementProfiles.Normal,
                "orientation_profile": DirectionProfiles.Aim,
                "sync" : False,
                "path_kwargs" : {"goal_state" : (-1250,0)},
                "orientation_kwargs" : {"theta" : 0}}
    

    def moveToBall(self):
        position_x = (self.blackboard.ally_robots[0].position_x) - (self.blackboard.balls[0].position_x)
        position_y = (self.blackboard.ally_robots[0].position_y) - (self.blackboard.balls[0].position_y)
        tang = position_y/position_x

        if self.blackboard.gui._is_field_side_left:
            return {"obstacles" : [],
            "path_profile" : MovementProfiles.Normal,
            "orientation_profile": DirectionProfiles.Aim,
            "sync" : False,
            "path_kwargs" : {"goal_state" : (self.blackboard.balls[0].position_x - 120,self.blackboard.balls[0].position_y)},
            "orientation_kwargs" : {"theta" : math.atan(tang)}}
        else: 
            return {"obstacles" : [],
            "path_profile" : MovementProfiles.Normal,
            "orientation_profile": DirectionProfiles.Aim,
            "sync" : False,
            "path_kwargs" : {"goal_state" : (self.blackboard.balls[0].position_x + 120,self.blackboard.balls[0].position_y)},
            "orientation_kwargs" : {"theta" : math.atan(tang)- pi}}
        
    def moveAwayFromBall(self):
        position_x = (self.blackboard.ally_robots[0].position_x) - (self.blackboard.balls[0].position_x)
        position_y = (self.blackboard.ally_robots[0].position_y) - (self.blackboard.balls[0].position_y)
        tang = position_y/position_x
        return {"obstacles" : [],
            "path_profile" : MovementProfiles.Normal,
            "orientation_profile": DirectionProfiles.Aim,
            "sync" : False,
            "path_kwargs" : {"goal_state" : (self.blackboard.balls[0].position_x - 120,self.blackboard.balls[0].position_y)},
            "orientation_kwargs" : {"theta" : math.atan(tang)- pi}}
        


class StraightMovement():
    """This class have types of movements using StraightProfile"""
    def __init__(self):
        super().__init__()
        self.blackboard = Blackboard()

    def run(self, theta):

        """Moviment to point when the robot is not goalkeeper"""

        return {"obstacles" : [PenaltyAreaObstacles(self.blackboard.geometry)],
                "path_profile" : MovementProfiles.Straight,
                "orientation_profile": [],
                "sync" : True,
                "path_kwargs" : {"theta" : theta},
                "orientation_kwargs" : {}}
    
    def moveToEnemyGoal(self, theta):
        return {"obstacles" : [PenaltyAreaObstacles(self.blackboard.geometry)],
                "path_profile" : MovementProfiles.Straight,
                "orientation_profile": DirectionProfiles.Aim,
                "sync" : False,
                "path_kwargs" : {"theta" : theta},
                "orientation_kwargs" : {"theta" : theta}}
        
        
class GetInAngleStrategy():
    """This class have types of movements using GetInAngleProfile"""
    def __init__(self):
        super().__init__()
        self.blackboard = Blackboard()

    def run(self, p_x, p_y, theta):
    # Theta is use in path_profile and angle is use in orientation_profile
        return {"obstacles" : [],
                "path_profile" : MovementProfiles.GetInAngle,
                "orientation_profile": DirectionProfiles.Aim,
                "sync" : False,
                "path_kwargs" : {"goal_state" : (p_x,p_y),"theta" : theta},
                "orientation_kwargs" : {"theta" : theta}}
    
    def moveToEnemyGoal(self, p_x, p_y, theta):
        """Moviment to the enemy goal"""
        return {"obstacles" : [PenaltyAreaObstacles(self.blackboard.geometry)],
                "path_profile" : MovementProfiles.GetInAngle,
                "orientation_profile": DirectionProfiles.Aim,
                "sync" : False,
                "path_kwargs" : {"goal_state" : (p_x,p_y),"theta" : theta},
                "orientation_kwargs" : {"theta" : theta}}
    
    def moveToBall(self, p_x, p_y, theta):
        return {"obstacles" : [PenaltyAreaObstacles(self.blackboard.geometry)],
                "path_profile" : MovementProfiles.GetInAngle,
                "orientation_profile": DirectionProfiles.Aim,
                "sync" : False,
                "path_kwargs" : {"goal_state" : (p_x,p_y),"theta" : theta},
                "orientation_kwargs" : {"theta" : theta}}

class BreakStrategy():
    """"This class have the BreakProfile movement"""
    def __init__(self):
        super().__init__()
        self.blackboard = Blackboard()

    def _break(self):
        return {"obstacles" : [],
                "path_profile" : MovementProfiles.Break,
                "orientation_profile": DirectionProfiles.Break,
                "sync" : False,
                "path_kwargs" : {},
                "orientation_kwargs" : {}}
    
class SpinStrategy():
    def __init__(self):
        super().__init__()
        self.blackboard = Blackboard()

    def spin(self, clockwise):
        return {"obstacles" : [],
                "path_profile" : MovementProfiles.Break,
                "orientation_profile": DirectionProfiles.Spin,
                "sync" : False,
                "path_kwargs" : {},
                "orientation_kwargs" : {"clockwise" : clockwise}} 
        
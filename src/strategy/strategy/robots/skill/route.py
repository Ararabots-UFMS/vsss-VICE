from movement.path.path_profiles import MovementProfiles
from strategy.blackboard import Blackboard


"""This file should have a properly class to use the Movement classes, look the example below"""

class NormalMovement():
    """This class should be a strategy skill to move a robot to a especific point"""
    def __init__(self, name, id):
        super().__init__()
        self.name = name
        self.id = id
        self.blackboard = Blackboard()

    def run(self):

        """Moviment to point when the robot is not goalkeeper"""

        if self.blackboard.gui.is_team_color_yellow == True and self.id != self.blackboard.referee._teams[1].goalkeeper:
            return {"obstacles" : self.blackboard.enemy_robots,
                    "path_profile" : MovementProfiles.Normal,
                    # "orientation": DirectionProfile.Aim,
                    "sync" : False,
                    "path_kwargs" : {"goal_state" : (0,0)},
                    "orientation_kwargs" : {"theta" : 0},
                    "goalkeeper": self.blackboard.referee._teams[1].goalkeeper,
                    "id": self.id,
                    "position" : "Not Goalkeeper"}
        
        elif self.blackboard.gui.is_team_color_yellow == False and self.id != self.blackboard.referee._teams[0].goalkeeper:
            return {"obstacles" : self.blackboard.enemy_robots,
                    "path_profile" : MovementProfiles.Normal,
                    # "orientation": DirectionProfile.Aim,
                    "sync" : False,
                    "path_kwargs" : {"goal_state" : (0,0)},
                    "orientation_kwargs" : {"theta" : 0},
                    "goalkeeper": self.blackboard.referee._teams[0].goalkeeper,
                    "id": self.id,
                    "position" : "Not Goalkeeper"}
        
        """Moviment to point when the robot is goalkeeper"""

        if self.blackboard.gui.is_team_color_yellow == True and self.id == self.blackboard.referee._teams[1].goalkeeper:
            return {"obstacles" : self.blackboard.enemy_robots,
                    "path_profile" : MovementProfiles.Normal,
                    # "orientation": DirectionProfile.Aim,
                    "sync" : False,
                    "path_kwargs" : {"goal_state" : (0,0)},
                    "orientation_kwargs" : {"theta" : 0},
                    "id": self.id,
                    "goalkeeper": self.blackboard.referee._teams[1].goalkeeper,
                    "position" : "Goalkeeper"}
        
        elif self.blackboard.gui.is_team_color_yellow == False and self.id == self.blackboard.referee._teams[0].goalkeeper:
            return {"obstacles" : self.blackboard.enemy_robots,
                    "path_profile" : MovementProfiles.Normal,
                    # "orientation": DirectionProfile.Aim,
                    "sync" : False,
                    "path_kwargs" : {"goal_state" : (0,0)},
                    "orientation_kwargs" : {"theta" : 0},
                    "id": self.id,
                    "goalkeeper": self.blackboard.referee._teams[0].goalkeeper,
                    "position" : "Goalkeeper"}
        
class StraightMovement():
    """This class have types of movements using StraightProfile"""
    def __init__(self,name):
        super().__init__()
        self.name = name
        self.id = 1 # TODO adjust to general cases
        self.blackboard = Blackboard()

    def run(self):

        """Moviment to point when the robot is not goalkeeper"""

        if self.blackboard.gui.is_team_color_yellow == True and self.id != self.blackboard.referee._teams[1].goalkeeper:
            return {"obstacles" : self.blackboard.enemy_robots,
                    "path_profile" : MovementProfiles.Normal,
                    # "orientation": DirectionProfile.Aim,
                    "sync" : False,
                    "path_kwargs" : {"theta" : 0},
                    "orientation_kwargs" : {"theta" : 0}}
        
        elif self.blackboard.gui.is_team_color_yellow == False and self.id != self.blackboard.referee._teams[0].goalkeeper:
            return {"obstacles" : self.blackboard.enemy_robots,
                    "path_profile" : MovementProfiles.Normal,
                    # "orientation": DirectionProfile.Aim,
                    "sync" : False,
                    "path_kwargs" : {"theta" : 0},
                    "orientation_kwargs" : {"theta" : 0}}
        
        """Moviment to point when the robot is goalkeeper"""

        if self.blackboard.gui.is_team_color_yellow == True and self.id == self.blackboard.referee._teams[1].goalkeeper:
            return {"obstacles" : self.blackboard.enemy_robots,
                    "path_profile" : MovementProfiles.Normal,
                    # "orientation": DirectionProfile.Aim,
                    "sync" : False,
                    "path_kwargs" : {"theta" : 0},
                    "orientation_kwargs" : {"theta" : 0}}
        
        elif self.blackboard.gui.is_team_color_yellow == False and self.id == self.blackboard.referee._teams[0].goalkeeper:
            return {"obstacles" : self.blackboard.enemy_robots,
                    "path_profile" : MovementProfiles.Normal,
                    # "orientation": DirectionProfile.Aim,
                    "sync" : False,
                    "path_kwargs" : {"theta" : 0},
                    "orientation_kwargs" : {"theta" : 0}}
        
        
        
            
from rclpy.node import Node

from time import time
from math import sqrt

from strategy.blackboard import Blackboard
from movement.move import Movement

from movement.path import path_profiles as profiles
from movement.path.path_acceptor import AcceptorStatus
from movement.obstacles.dynamic_obstacles import RobotObstacle
from movement.obstacles.static_obstacles import PenaltyAreaObstacles


from ruckig import Trajectory

class Robot(Node):
    def __init__(self, id, name) -> None:
        super().__init__(f"{name}")
        self.blackboard = Blackboard()
        self.name = name
        self.id = id
        self.behaviour = None

        self.move = Movement(self.id)

        self.path_trajectory = Trajectory(2)  # Degrees of freedom = 2
        self.orientation_trajectory = Trajectory(1)

        self.trajectory_start_time = 0.0
        self.acceptance_radius = 100
        self.min_correction_time = 0.2

        self.behaviour_command = {"obstacles" : None, 
                                "path_profile" : None,
                                "orientation_profile" : None,
                                "sync" : None,
                                "path_kwargs" : None, 
                                "orientation_kwargs" : None}

        self.exit_area_command = {"obstacles" : [],
                                "path_profile" : profiles.MovementProfiles.Normal,
                                "orientation_profile" : profiles.DirectionProfiles.Aim,
                                "sync" : False,
                                "path_kwargs" : None,
                                "orientation_kwargs" : {"theta" : 0}}

        self.bypass_command = {"obstacles" : None, 
                                "path_profile" : profiles.MovementProfiles.Normal,
                                "orientation_profile" : profiles.DirectionProfiles.Aim,
                                "sync" : False,
                                "path_kwargs" : None, 
                                "orientation_kwargs" : {"theta" : 0}}

        self.current_command = self.behaviour_command

        self.current_status: AcceptorStatus = None

        self.running = self.create_timer(1/60, self.run)


        ####
        # self.test_time = time()


    def run(self):
        ''' Runs robot behaviour tree and updates trajectorys based on commands received '''

        # ----- Only for testing -----
        # ----------------------------
        # position = (0, 0)
        # t = time() - self.test_time
        # if t < 10:
        #     position = (3500, 0)
        # elif t < 20:
        #     position = (-3500, 0)
        # elif t >= 20:
        #     self.test_time = time()
        #     return

        command = {"obstacles" : [PenaltyAreaObstacles(self.blackboard.geometry)], 
                   "path_profile" : profiles.MovementProfiles.Break,
                   "orientation_profile" : profiles.DirectionProfiles.Aim,
                   "sync" : False,
                   "path_kwargs" : {}, 
                   "orientation_kwargs" : {"theta" : 0}}

        # ---------------------------
        # ----------- END -----------

        # Recalculating route if behaviour tree changes commands.
        # Using Dict comprehensions to ignore obstacles while comparing...
        if ({k: v for k,v in self.current_command.items() if k not in ["obstacles"]} != {k: v for k,v in command.items() if k not in ["obstacles"]}): # Excluing obstacles comparisons, review this...
            from_vision = True
        else:
            from_vision = False # Using ideal predicted states over mesuared to enforce same trajectory.
        
        self.behaviour_command = command

        if self.correct_positioning(self.acceptance_radius):
            from_vision = True
        
        self.update_trajectory(from_vision)

    def update_trajectory(self, from_vision: bool = False) -> None:
        ''' Updates or enforces robot trajectory and checking and solving for collision and area limitations conflits '''
        result = self.move(self.get_state(from_vision), **self.behaviour_command)
        # Result can be either two paths if accepted or a acceptor status and a obstacle if not.
        
        if(result[0] == AcceptorStatus.INSIDEAREA):
            self.exit_area(result[1]) # result[1] is a obstacle area.
            self.current_status = AcceptorStatus.INSIDEAREA

        elif(result[0] == AcceptorStatus.COLLISION):
            bypass_point = self.solve_collision()
        
        elif type(result) != AcceptorStatus:
            self.current_status = None
            self.current_command = self.behaviour_command

            self.path_trajectory, self.orientation_trajectory = result

        self.trajectory_start_time = time()
        return None

    def correct_positioning(self, acceptance_radius) -> bool:
        ''' Corrects position error when finalizing trajectory '''
        if self.get_relative_time() > self.path_trajectory.duration * 0.9:
            expected_position, _ = self.get_state(from_vision=False)
            real_position, _ = self.get_state(from_vision=True)

            diff = sqrt((expected_position[0] - real_position[0]) ** 2 + (expected_position[1] - real_position[1]) ** 2)
            if diff > acceptance_radius:
                self.get_logger().info(f"CORRECTION {expected_position, real_position} == {diff}")
                return True

        return False

    def solve_collision(self):
        pass

    def exit_area(self, obs):
        if self.current_status == AcceptorStatus.INSIDEAREA:
            from_vision = False

        else:
            point = obs.closest_outside_point(self.get_state(from_vision=True)[0][:2])
            self.get_logger().info(f"{point}")
            from_vision = True

            self.exit_area_command["path_kwargs"] = {"goal_state" : (point[0], point[1])}

        if self.correct_positioning(acceptance_radius = 10):
            from_vision = True

        self.path_trajectory, self.orientation_trajectory = self.move(self.get_state(from_vision), **self.exit_area_command)

        self.current_command = self.exit_area_command
        
    def get_state(self, from_vision: bool):
        ''' Retuns a predicted state from current trajectory or use measured state from vision '''
        if from_vision:
            robot = self.blackboard.ally_robots[self.id]

            state = ([robot.position_x, robot.position_y, robot.orientation],
                     [robot.velocity_x, robot.velocity_y, robot.velocity_orientation])

        else:
            state = self.path_trajectory.at_time(self.get_relative_time())[:2]
            ideal_orientation = self.orientation_trajectory.at_time(self.get_relative_time())[:2]

            state[0].append(ideal_orientation[0][0])
            state[1].append(ideal_orientation[1][0])

            state = (state[0], state[1])

        return state

    def get_relative_time(self) -> float:
        ''' Retuns the relative time from trajectory time '''
        return time() - self.trajectory_start_time


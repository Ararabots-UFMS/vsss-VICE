from rclpy.node import Node

from time import time
from math import sqrt, pi, cos, sin
from random import uniform


from strategy.blackboard import Blackboard
from movement.move import Movement

from movement.path import path_profiles as profiles
from movement.path.path_acceptor import AcceptorStatus
from movement.obstacles.dynamic_obstacles import RobotObstacle
from movement.obstacles.static_obstacles import PenaltyAreaObstacles, BoundaryObstacles, WallObstacles

from movement.move import Movement, RobotStatus
from movement.path.path_profiles import MovementProfiles, DirectionProfiles
from strategy.robots.running.attacker import OurActionAttacker
from strategy.robots.halt.attacker import ActionAttacker
from strategy.robots.penalty.our_penalty.goalkeeper import OurGoalkeeperAction

from control.mpc import Controller

from ruckig import Trajectory

from time import time
from math import sqrt
import numpy as np

from movement.obstacles.dynamic_obstacles import RobotObstacle

class Robot(Node):
    def __init__(self, id: int, name: str, max_velocity = 3000, max_angular_vel = 1, max_acceleration = 1000, max_angular_acc = 0.5):
        super().__init__(f"{name}")
        self.id = id
        self.name = name

        self.blackboard = Blackboard()
        self.behaviour = None

        self.move = Movement(self.id, bypass_trys = 100, bypass_time = 0.5, bypass_max_radius = 2500)

        _, (self.path_trajectory, self.orientation_trajectory) = self.move(self.get_state(), obstacles = [],
                                                                      path_profile = MovementProfiles.Break,
                                                                      orientation_profile = DirectionProfiles.Break,
                                                                      sync = False,
                                                                      path_kwargs = {},
                                                                      orientation_kwargs = {})

        self.trajectory_start_time = time()
        self.acceptance_radius = 0

        self.controller = Controller(max_velocity, max_angular_vel)
        self.controller.set_initial_guess(self.get_state()[0])
        self.controller.set_trajectory(self.path_trajectory, self.orientation_trajectory)
        self.controller.reset_history()

        self.current_command = None
        self.status = RobotStatus.NORMAL

        self.running = self.create_timer(1/4, self.run)
        self.update_control = self.create_timer(1/16, self.update_control)

        self.velocities = np.array([0, 0, 0])

        self.test = time()

    def run(self):
        
        # if self.behaviour == None:
        #     self.behaviour = AttackerAction("Stop!!!!")
        
        # command = self.behaviour()
        self.behaviour = OurGoalkeeperAction("Goalkeeper")
        command = self.behaviour()
        # print(command)
        

        self.update_trajectory(command)

    def update_trajectory(self, command):
        status, (path_trajectory, orientation_trajectory) = self.move(self.get_state(from_vision=False), **command)

        if ((status != self.status or self.check_command_change(command)) or
            (not self.check_acceptance_radius and self.get_relative_time() > self.path_trajectory.duration)):
            if not (self.status == RobotStatus.COLLISION and status == RobotStatus.BYPASS_NOT_FOUND):
                self.path_trajectory = path_trajectory
                self.orientation_trajectory = orientation_trajectory
                self.trajectory_start_time = time()

                self.status = status
                self.current_command = command

                self.controller.set_initial_guess(self.get_state()[0])
                self.controller.set_trajectory(self.path_trajectory, self.orientation_trajectory)
                self.controller.reset_history()

                self.get_logger().info(f"Changing status to --- {self.status} ---")

        elif self.check_acceptance_radius() and self.get_relative_time() > self.path_trajectory.duration * 0.9:
            break_command = self.current_command
            break_command['path_profile'] = MovementProfiles.Break
            break_command['path_kwargs'] = {}

            status, (path_trajectory, orientation_trajectory) = self.move(self.get_state(), **break_command)

            self.path_trajectory = path_trajectory
            self.orientation_trajectory = orientation_trajectory

            self.status = status
            self.current_command = command

            self.controller.set_initial_guess(self.get_state()[0])
            self.controller.set_trajectory(self.path_trajectory, self.orientation_trajectory)

            self.get_logger().info(f"--- Final Position Reached, Breaking ---")

    def check_acceptance_radius(self):
        real_position, _ = self.get_state()
        expected_path_position, _, _ = self.path_trajectory.at_time(self.path_trajectory.duration)

        if self.current_command['path_profile'] == MovementProfiles.Normal or self.current_command['path_profile'] == MovementProfiles.GetInAngle:
            if sqrt((real_position[0] - expected_path_position[0])**2 + (real_position[1] - expected_path_position[1])**2) < self.acceptance_radius:
                return True

        return False

    def check_command_change(self, command):
        if self.current_command == None:
            return True

        elif ({k: v for k,v in self.current_command.items() if k not in ["obstacles"]} != {k: v for k,v in command.items() if k not in ["obstacles"]}): # Excluing obstacles comparisons, review this...
            return True

        return False
        
    def update_control(self):
        self.velocities = self.controller(self.get_state()[0])

    def set_behaviour(self, behaviour_tree):
        self.behaviour = behaviour_tree

    def get_state(self, from_vision = True):
        ''' Retuns robots position from blackboard '''
        if from_vision:
            robot = self.blackboard.ally_robots[self.id]

            return (np.array([robot.position_x, robot.position_y, robot.orientation]),
                np.array([robot.velocity_x, robot.velocity_y, robot.velocity_orientation]))

        state = self.path_trajectory.at_time(self.get_relative_time())[:2]
        ostate = self.orientation_trajectory.at_time(self.get_relative_time())[:2]

        state[0].append(ostate[0][0])
        state[1].append(ostate[1][0])

        return np.array(state)

    def get_relative_time(self):
        return time() - self.trajectory_start_time
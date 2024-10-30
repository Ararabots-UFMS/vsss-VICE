# from rclpy.node import Node

# from time import time

# from strategy.blackboard import Blackboard
# from movement.move import Movement

# from ruckig import Trajectory



# class Robot(Node):
#     def __init__(self, id, name) -> None:
#         super().__init__(f"robot_{name}")
#         self.blackboard = Blackboard()
#         self.name = name
#         self.id = id
#         self.behaviour_tree = None

#         self.move = Movement(self.id)
#         self.trajectory = Trajectory(3)  # Degrees of freedom = 3
#         self.trajectory_start_time = 0
#         self.kick = False

#         self.timer = self.create_timer(0.1, self.run)

#     def run(self):
#         if self.behaviour_tree != None and self.behaviour_tree != "None":
#             self.get_logger().info(f"Running robot {self.id}")
#             profile = self.behaviour_tree()
#             print(profile)
            # self.trajectory = self.move(profile)
            # self.trajectory_start_time = time()

### Fabio Code

from rclpy.node import Node

from time import time
from math import sqrt, pi, cos, sin
from random import uniform
from math import sqrt, pi, cos, sin
from random import uniform

from strategy.blackboard import Blackboard
from movement.move import Movement

from movement.path import path_profiles as profiles
from movement.path.path_acceptor import AcceptorStatus
from movement.obstacles.dynamic_obstacles import RobotObstacle
from movement.obstacles.static_obstacles import PenaltyAreaObstacles, BoundaryObstacles, WallObstacles


from movement.path import path_profiles as profiles
from movement.path.path_acceptor import AcceptorStatus
from movement.obstacles.dynamic_obstacles import RobotObstacle
from movement.obstacles.static_obstacles import PenaltyAreaObstacles, BoundaryObstacles, WallObstacles


from ruckig import Trajectory

class Robot(Node):
    def __init__(self, id, name) -> None:
        super().__init__(f"{name}")
        super().__init__(f"{name}")
        self.blackboard = Blackboard()
        self.name = name
        self.id = id
        self.behaviour_tree = None

        self.move = Movement(self.id)

        self.path_trajectory = Trajectory(2)  # Degrees of freedom = 2
        self.orientation_trajectory = Trajectory(1)

        self.trajectory_start_time = 0.0
        self.acceptance_radius = 100

        self.bypass_time = 2
        self.bypass_min_radius = 500
        self.bypass_max_radius = 2500
        self.bypass_trys = 20

        self.behaviour_command = {"obstacles" : None, 
                                "path_profile" : None,
                                "orientation_profile" : None,
                                "sync" : None,
                                "path_kwargs" : None, 
                                "orientation_kwargs" : None}

        self.exit_area_command = {"obstacles" : [],
                                "path_profile" : profiles.MovementProfiles.Normal,
                                "orientation_profile" : profiles.DirectionProfiles.Break,
                                "sync" : False,
                                "path_kwargs" : None,
                                "orientation_kwargs" : {}}

        self.bypass_command = {"obstacles" : None, 
                                "path_profile" : profiles.MovementProfiles.Normal,
                                "orientation_profile" : profiles.DirectionProfiles.Break,
                                "sync" : False,
                                "path_kwargs" : None, 
                                "orientation_kwargs" : {}}

        self.current_command = self.behaviour_tree

        self.current_status: AcceptorStatus = None

        self.running = self.create_timer(1/60, self.run)
        self.enforcer = self.create_timer(0.5, self.enforce_path)


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

        if self.behaviour_tree != None and self.behaviour_tree != "None":
            self.get_logger().info(f"Running robot {self.id}")
            self.behaviour_command = self.behaviour_tree()
            self.current_command = self.behaviour_tree()

        # command = {"obstacles" : [PenaltyAreaObstacles(self.blackboard.geometry), BoundaryObstacles(self.blackboard.geometry), WallObstacles(self.blackboard.geometry)], 
        #            "path_profile" : profiles.MovementProfiles.Normal,
        #            "orientation_profile" : profiles.DirectionProfiles.Break,
        #            "sync" : False,
        #            "path_kwargs" : {"goal_state" : (4000, 2800)}, 
        #            "orientation_kwargs" : {}}

        # ---------------------------
        # ----------- END -----------

        # Recalculating route if behaviour tree changes commands.
        # Using Dict comprehensions to ignore obstacles while comparing...
        if self.current_command != None and self.current_command != "None":
            if ({k: v for k,v in self.current_command.items() if k not in ["obstacles"]} != {k: v for k,v in self.current_command.items() if k not in ["obstacles"]}): # Excluing obstacles comparisons, review this...
                from_vision = True
            else:
                from_vision = False # Using ideal predicted states over mesuared to enforce same trajectory.

            if self.correct_positioning(self.acceptance_radius):
                self.get_logger().info("CONFERINDO")
                from_vision = True
            
            self.update_trajectory(from_vision)

    def enforce_path(self):
        if self.current_command != None and self.current_command != "None":
            self.get_logger().info("ENFORCING")
            result = self.move(self.get_state(from_vision=True), **self.current_command)
            if type(result[0]) != AcceptorStatus:
                self.path_trajectory, self.orientation_trajectory = result

    def update_trajectory(self, from_vision: bool = False) -> None:
        print(self.behaviour_command)
        ''' Updates or enforces robot trajectory and checking and solving for collision and area limitations conflits '''
        result = self.move(self.get_state(from_vision), **self.behaviour_command)
        print(result)
        # Result can be either two paths if accepted or a acceptor status and a obstacle if not.

        if(result[0] == AcceptorStatus.INSIDEAREA):
            # self.get_logger().info(f"INSIDEAREA")
            self.exit_area(result[1]) # result[1] is a obstacle area.

        elif(result[0] == AcceptorStatus.COLLISION):
            self.solve_collision()
        
        elif type(result[0]) != AcceptorStatus:
            #self.get_logger().info(f"NORMAL")
            self.current_status = None
            self.current_command = self.behaviour_command

            self.path_trajectory, self.orientation_trajectory = result
            self.trajectory_start_time = time()

        return None

    def correct_positioning(self, acceptance_radius) -> bool:
        ''' Corrects position error when finalizing trajectory '''
        # expected_position, expected_velocity = self.get_state(from_vision=False)
        # real_position, real_velocity = self.get_state(from_vision=True)

        # diff = sqrt((expected_position[0] - real_position[0]) ** 2 + (expected_position[1] - real_position[1]) ** 2)
        # vel_diff = sqrt((expected_velocity[0] - real_velocity[0]) ** 2 + (expected_velocity[1] - real_velocity[1]) ** 2)

        # if self.get_relative_time() > self.path_trajectory.duration * 0.1:
        #     if diff > acceptance_radius or vel_diff > acceptance_radius:
        #         self.get_logger().info(f"CORRECTION {expected_position, real_position} == {diff}")
        #         return True

        return False

    def solve_collision(self):
        if self.current_status == AcceptorStatus.COLLISION and self.get_relative_time() > self.path_trajectory.duration:
            if self.correct_positioning(acceptance_radius=self.acceptance_radius):
                result = self.move(self.get_state(from_vision=True), **self.bypass_command)
                if type(result[0]) != AcceptorStatus:
                    self.path_trajectory, self.orientation_trajectory = result

                self.current_command = self.bypass_command
                self.trajectory_start_time = time()

            else:
                self.current_status = None
                return

        if self.current_status == AcceptorStatus.COLLISION and self.get_relative_time() < self.path_trajectory.duration:
            from_vision = False

        else:
            real_position, _ = self.get_state(from_vision=True)

            best_command = None
            best_time = float("inf")

            for _ in range(self.bypass_trys):
                random_angle = uniform(-pi, pi)
                random_radius = uniform(self.bypass_min_radius, self.bypass_max_radius)
                random_point = [real_position[0] + random_radius * cos(random_angle), real_position[1] + random_radius * sin(random_angle)]

                command = self.bypass_command
                command["path_profile"] = profiles.MovementProfiles.Normal
                command["obstacles"] = self.behaviour_command["obstacles"]
                command["path_kwargs"] = {"goal_state" : random_point}

                bypass_result = self.move(self.get_state(from_vision=True), **command)
                if type(bypass_result[0]) == AcceptorStatus:
                    continue

                state = bypass_result[0].at_time(bypass_result[0].duration)[:2]
                ostate = bypass_result[1].at_time(bypass_result[1].duration)[:2]

                state[0].append(ostate[0][0])
                state[1].append(ostate[1][0])

                new_result = self.move(state, **self.behaviour_command)

                if type(new_result[0]) == AcceptorStatus:
                    continue

                duration = new_result[0].duration
                if duration < best_time:
                    best_command = command
                    best_time = duration


            if not best_command:
                self.current_status = None
                self.bypass_command = {"obstacles" : [], 
                            "path_profile" : profiles.MovementProfiles.Break,
                            "orientation_profile" : profiles.DirectionProfiles.Break,
                            "sync" : False,
                            "path_kwargs" : {}, 
                            "orientation_kwargs" : {}}

            else:
                self.get_logger().info(f"BEST COMMMAND {state}")
                self.bypass_command = best_command
                self.current_status = AcceptorStatus.COLLISION

            from_vision = True

        # if self.correct_positioning(acceptance_radius = 10):
        #     from_vision = True

        result = self.move(self.get_state(from_vision), **self.bypass_command)
        if type(result[0]) != AcceptorStatus:
            self.path_trajectory, self.orientation_trajectory = result

        self.current_command = self.bypass_command
        self.trajectory_start_time = time()

    def exit_area(self, obs):
        if self.current_status == AcceptorStatus.INSIDEAREA and self.get_relative_time() < self.path_trajectory.duration:
            from_vision = False

        else:
            point = obs.closest_outside_point(self.get_state(from_vision=True)[0][:2])
            from_vision = True

            self.exit_area_command["path_kwargs"] = {"goal_state" : (point[0], point[1])}

        if self.correct_positioning(acceptance_radius = 10):
            from_vision = True

        self.path_trajectory, self.orientation_trajectory = self.move(self.get_state(from_vision), **self.exit_area_command)

        self.current_command = self.exit_area_command
        self.current_status = AcceptorStatus.INSIDEAREA
        self.trajectory_start_time = time()
        
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
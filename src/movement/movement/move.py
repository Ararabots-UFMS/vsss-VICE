from movement.obstacles.interfaces import Obstacle, StaticObstacle, DynamicObstacle
from movement.path.path import PathGenerator
from movement.path.path_acceptor import PathAcceptor, AcceptorStatus
from movement.path.path_profiles import MovementProfiles, DirectionProfiles
from strategy.blackboard import Blackboard
from system_interfaces.msg import Robots

from ruckig import InputParameter, OutputParameter, Result, Ruckig, Trajectory, Synchronization

from typing import List, Tuple
from enum import Enum

from math import cos, sin, pi, atan2
from random import uniform

class RobotStatus(Enum):
    NORMAL = 1
    COLLISION = 2
    EXIT_AREA = 3
    BYPASS_NOT_FOUND = 4
    
class Movement:
    def __init__(self, robot_id: int, bypass_trys: int, bypass_time: float, bypass_max_radius: float):
        self.id = robot_id
        self.blackboard = Blackboard()

        self.bypass_trys = bypass_trys
        self.bypass_time = bypass_time
        self.bypass_max_radius = bypass_max_radius

        self.path_generator = PathGenerator()
        self.acceptor = PathAcceptor()

        self.path_otg = Ruckig(2)
        self.orientation_otg = Ruckig(1)

    def __call__(
        self, init_state, obstacles: List[Obstacle], path_profile: MovementProfiles, orientation_profile: DirectionProfiles, sync: bool, **kwargs
    ) -> Tuple:
        # **kwargs need to be two different dicts of parameters. In this case, {path_kwargs} and {orientation_kwargs} inside kwargs.

        path_trajectory = Trajectory(2)
        orientation_trajectory = Trajectory(1)

        # Using ruckig notations
        path_inp, orientation_inp = self.path_generator.generate_input(init_state, path_profile, orientation_profile, **kwargs)

        self.calculate_path(path_inp, path_trajectory, self.path_otg)
        self.calculate_path(orientation_inp, orientation_trajectory, self.orientation_otg)

        status, collision_obs = self.acceptor.check(path_trajectory, obstacles)

        if status == AcceptorStatus.INSIDEAREA:
            return RobotStatus.EXIT_AREA, self.exit_area(init_state, collision_obs)

        elif status == AcceptorStatus.COLLISION:
            if isinstance(collision_obs, StaticObstacle):
                if collision_obs.is_colission(path_trajectory.at_time(path_trajectory.duration)[0][:2]):
                    return RobotStatus.NORMAL, self.guard_area(init_state, collision_obs, path_trajectory, orientation_profile, **kwargs)
            if isinstance(collision_obs, DynamicObstacle) and collision_obs.is_colission(0.3, path_trajectory.at_time(0.01)[0][:2]):
                return RobotStatus.NORMAL, (path_trajectory, orientation_trajectory)
            return self.solve_collision(init_state, obstacles, path_profile, orientation_profile, **kwargs)

        elif status == AcceptorStatus.ACCEPTED:
            return RobotStatus.NORMAL, (path_trajectory, orientation_trajectory)

    def solve_collision(
        self, init_state, obstacles: List[Obstacle], path_profile: MovementProfiles, orientation_profile: DirectionProfiles, **kwargs):
        # Try to find a bypass trajectory and bypass orientation, if not found, break
        new_trajectory = Trajectory(2)
        new_otrajectory = Trajectory(1)

        best_trajectory = None
        best_time = float("inf")

        for _ in range(self.bypass_trys):
            random_radius1 = uniform(-self.bypass_max_radius, self.bypass_max_radius)
            random_radius2 = uniform(-self.bypass_max_radius, self.bypass_max_radius)

            random_point = [init_state[0][0] + random_radius1, init_state[0][1] + random_radius2]

            angle_to_target = atan2(((kwargs["path_kwargs"]["goal_state"])[1]) - random_point[1], ((kwargs["path_kwargs"]["goal_state"])[0]) - random_point[0])
            random_error_angle = uniform(angle_to_target - angle_to_target * 0.2, angle_to_target + angle_to_target * 0.2)

            new_inp, new_oinp = self.path_generator.generate_input(
            init_state,
            MovementProfiles.GetInAngle,
            orientation_profile,
            path_kwargs = {'goal_state':(random_point[0], random_point[1]), "theta": random_error_angle}, 
            orientation_kwargs = kwargs['orientation_kwargs'])

            self.calculate_path(new_inp, new_trajectory, self.path_otg)
            self.calculate_path(new_oinp, new_otrajectory, self.orientation_otg)

            status, collision_obs = self.acceptor.check(new_trajectory, obstacles)

            if status == AcceptorStatus.ACCEPTED:
                step = 0.2
                for i in range(int(new_trajectory.duration / 0.2)):
                    final_path = Trajectory(2)
                    ofinal_path = Trajectory(1)

                    state = new_trajectory.at_time(i * step)[:2]
                    ostate = new_otrajectory.at_time(i * step)[:2]

                    state[0].append(ostate[0][0])
                    state[1].append(ostate[1][0])

                    new_inp, new_oinp = self.path_generator.generate_input(
                    state,
                    path_profile,
                    orientation_profile,
                    **kwargs)

                    self.calculate_path(new_inp, final_path, self.path_otg)
                    self.calculate_path(new_oinp, ofinal_path, self.orientation_otg)

                    status, collision_obs = self.acceptor.check(final_path, obstacles)

                    if isinstance(collision_obs, StaticObstacle) and collision_obs.is_colission(final_path.at_time(final_path.duration)[0][:2]):
                        final_path, ofinal_path = self.guard_area(state, collision_obs, final_path, orientation_profile, **kwargs)

                    status, collision_obs = self.acceptor.check(final_path, obstacles)

                    if status == AcceptorStatus.ACCEPTED:
                        path_time = new_trajectory.duration + final_path.duration
                        if path_time < best_time:
                            point = random_point
                            best_trajectory = RobotStatus.COLLISION, (new_trajectory, new_otrajectory)
                        continue

            if best_trajectory == None:
                new_inp, new_oinp = self.path_generator.generate_input(
                init_state,
                MovementProfiles.Break,
                orientation_profile,
                path_kwargs = {}, orientation_kwargs = kwargs['orientation_kwargs'])

                self.calculate_path(new_inp, new_trajectory, self.path_otg)
                self.calculate_path(new_oinp, new_otrajectory, self.orientation_otg)

                best_trajectory = RobotStatus.BYPASS_NOT_FOUND, (new_trajectory, new_otrajectory)

            return best_trajectory

    def guard_area(self, init_state, obstacle, path_trajectory, orientation_profile, **kwargs):
        new_trajectory = Trajectory(2)
        new_otrajectory = Trajectory(1)

        outside_point = obstacle.closest_outside_point(path_trajectory.at_time(path_trajectory.duration)[0][:2])

        new_inp, new_oinp = self.path_generator.generate_input(
            init_state,
            MovementProfiles.Normal,
            orientation_profile,
            path_kwargs = {"goal_state" : outside_point},
            orientation_kwargs = kwargs["orientation_kwargs"])

        self.calculate_path(new_inp, new_trajectory, self.path_otg)
        self.calculate_path(new_oinp, new_otrajectory, self.orientation_otg)

        return new_trajectory, new_otrajectory

    def exit_area(self, init_state, area: StaticObstacle):
        new_trajectory = Trajectory(2)
        new_otrajectory = Trajectory(1)

        outside_point = area.closest_outside_point(init_state[0])

        # TODO: Implement a DirectionPRofile Normal and pass the parameters here...
        new_inp, new_oinp = self.path_generator.generate_input(
            init_state,
            MovementProfiles.Normal,
            DirectionProfiles.Break,
            path_kwargs = {'goal_state': (outside_point[0], outside_point[1])}, orientation_kwargs = {})

        self.calculate_path(new_inp, new_trajectory, self.path_otg)
        self.calculate_path(new_oinp, new_otrajectory, self.orientation_otg)

        return new_trajectory, new_otrajectory

    # I'm getting a lot of error from ruckig, so, if a error occur, gonna just ignore it
    def calculate_path(self, inp, traj, otg):
        try:
            otg.calculate(inp, traj)
        except:
            try:
                inp.synchronization = Synchronization.No
                otg.calculate(inp, traj)
                inp.synchronization = Synchronization.Yes
            except:
                pass
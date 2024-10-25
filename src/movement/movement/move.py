from movement.obstacles.interfaces import Obstacle, StaticObstacle
from movement.path.path import PathGenerator
from movement.path.path_acceptor import PathAcceptor, AcceptorStatus
from movement.path.path_profiles import MovementProfiles, DirectionProfiles
from strategy.blackboard import Blackboard
from system_interfaces.msg import Robots

from ruckig import InputParameter, OutputParameter, Result, Ruckig, Trajectory, Synchronization

from typing import List, Tuple


class Movement:
    def __init__(self, robot_id: int):
        self.id = robot_id
        self.blackboard = Blackboard()

        self.path_generator = PathGenerator()
        self.acceptor = PathAcceptor()

        self.path_otg = Ruckig(2)
        self.orientation_otg = Ruckig(1)

    def __call__(
        self, init_state: Tuple[List[float], List[float]], obstacles: List[Obstacle], path_profile: MovementProfiles, orientation_profile: DirectionProfiles, sync: bool, **kwargs
    ) -> Tuple:
        # **kwargs need to be two different dicts of parameters. In this case, {path_kwargs} and {orientation_kwargs} inside kwargs.

        path_trajectory = Trajectory(2)
        orientation_trajectory = Trajectory(1)

        # Using ruckig notations
        path_inp, orientation_inp = self.path_generator.generate_input(init_state, path_profile, orientation_profile, **kwargs)

        self.calculate_path(path_inp, path_trajectory, self.path_otg)
        self.calculate_path(orientation_inp, orientation_trajectory, self.orientation_otg)

        status, collision_obs = self.acceptor.check(path_trajectory, obstacles)

        # TODO INSIDEAREA is taking priority, it may lead to some issues
        # If inside area, any trajectory is overwriten to a trajectory to get of the area.
        if status == AcceptorStatus.INSIDEAREA:
            return AcceptorStatus.INSIDEAREA, collision_obs

        elif status == AcceptorStatus.ACCEPTED or (
            path_profile != MovementProfiles.Normal
            and path_profile != MovementProfiles.GetInAngle
        ): 
            if sync:
                if path_trajectory.duration < orientation_trajectory.duration:
                    path_inp.minimum_duration = orientation_trajectory.duration
                    self.calculate_path(path_inp, path_trajectory, self.path_otg)
                else:
                    orientation_inp.minimum_duration = path_trajectory.duration
                    self.calculate_path(orientation_inp, orientation_trajectory, self.orientation_otg)

            return path_trajectory, orientation_trajectory

        else:
            # TODO make parameters changeable.
            return AcceptorStatus.COLLISION
            # return self.solve_collision(init_state, obstacles, trys=10, bypass_time=1.5, path_profile=path_profile, **kwargs)
        

    def solve_collision(
        self, init_state, obstacles: List[Obstacle], trys: int, bypass_time: float, path_profile: MovementProfiles, **kwargs
    ):
        # Try to find a bypass trajectory and bypass orientation, if not found, break
        for i in range(trys):

            bypass_trajectory = Trajectory(2)
            obypass_trajectory = Trajectory(1)

            bypass_inp, obypass_inp = self.path_generator.generate_input(
                init_state, MovementProfiles.Bypass, DirectionProfiles.Normal, path_kwargs = {}, orientation_kwargs = {'current_state': init_state}
            )

            self.calculate_path(bypass_inp, bypass_trajectory, self.path_otg)

            status, obs = self.acceptor.check(bypass_trajectory, obstacles)

            if status == AcceptorStatus.ACCEPTED:
                new_path = Trajectory(2)

                new_position = bypass_trajectory.at_time(bypass_time)
                new_position[0].append(0)
                new_position[1].append(0)

                new_inp, _ = self.path_generator.generate_input(
                    new_position, path_profile, DirectionProfiles.Break, path_kwargs = kwargs["path_kwargs"], orientation_kwargs = {}
                )

                self.calculate_path(new_inp, new_path, self.path_otg)

                status, obs = self.acceptor.check(new_path, obstacles)

                if status == AcceptorStatus.ACCEPTED:
                    calculate(obypass_inp, obypass_trajectory, self.orientation_otg)

                    return bypass_trajectory, obypass_trajectory
            
            elif status == AcceptorStatus.INSIDEAREA:
                return self.exit_area(obs)

        break_inp, obreak_inp = self.path_generator.generate_input(
            init_state, MovementProfiles.Break, DirectionProfiles.Break, path_kwargs={}, orientation_kwargs={}
        )

        self.calculate_path(break_inp, bypass_trajectory, self.path_otg)
        self.calculate_path(obreak_inp, obypass_trajectory, self.orientation_otg)

        return bypass_trajectory, obypass_trajectory

    def exit_area(self, init_state, area: StaticObstacle):
        new_trajectory = Trajectory(2)
        new_otrajectory = Trajectory(1)

        outside_point = area.closest_outside_point(init_state)

        # TODO: Implement a DirectionPRofile Normal and pass the parameters here...
        new_inp, new_oinp = self.path_generator.generate_input(
            init_state,
            MovementProfiles.Normal,
            DirectionProfiles.Normal,
            path_kwargs = {'goal_state':(
                (outside_point[0], outside_point[1]),
            )},
            orientation_kwargs = {'current_state': init_state}
        )

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
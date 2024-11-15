from utils.math_utils import orientation_solver
from movement.path.path_profiles import PathProfile, OrientationProfile

from ruckig import InputParameter, OutputParameter, Result, Ruckig, Trajectory

from typing import List, Optional, Tuple


class PathGenerator:
    def __init__(
        self,
        constrainsts: Tuple[List[float]] = ([800, 800, 1], [800, 800, 0.5]),
    ):
        self.vel_constrainst = constrainsts[0]
        self.acc_constrainst = constrainsts[1]

    def generate_input(
        self,
        init_state: Tuple[List[float]],
        path_profile: PathProfile,
        orientation_profile: OrientationProfile,
        **kwargs
    ) -> Tuple[InputParameter, InputParameter]:

        # **kwargs need to be two different dicts of parameters. In this case, {path_kwargs} and {orientation_kwargs} inside kwargs.
        inp_path = InputParameter(2)
        inp_orientation = InputParameter(1)

        inp_path.current_position = init_state[0][:2]
        inp_path.current_velocity = init_state[1][:2]

        inp_orientation.current_position = [init_state[0][2]]
        inp_orientation.current_velocity = [init_state[1][2]]

        inp_path.max_velocity = self.vel_constrainst[:2]
        inp_path.max_acceleration = self.acc_constrainst[:2]

        inp_orientation.max_velocity = [self.vel_constrainst[2]]
        inp_orientation.max_acceleration = [self.acc_constrainst[2]]

        # kwargs["path_kwargs"]["inp"] = inp_path
        # kwargs["orientation_kwargs"]["inp"] = inp_orientation

        path_profile.generate(inp_path, **kwargs["path_kwargs"])
        orientation_profile.generate(inp_orientation, **kwargs["orientation_kwargs"])

        inp_orientation.current_position[0], inp_orientation.target_position[0] = (
            orientation_solver(
                inp_orientation.current_position[0], inp_orientation.target_position[0]
            )
        )

        return inp_path, inp_orientation

import rclpy.executors
from rclpy.node import Node
import rclpy

from control_unit.robot import Robot
from strategy.blackboard import Blackboard
from strategy.play.strategy_node import make_bt
from py_trees import logging as log_tree


class Coach(Node):
    def __init__(self, _executor, behaviour_tree) -> None:
        super().__init__("coach")

        self._executor = _executor
        self.behaviour_tree = behaviour_tree

        # get blackboard
        self.blackboard = Blackboard()

        self.robots = {}

        # TODO: experiment with other timer rates
        self.timer = self.create_timer(0.5, self.update)
        self.timer = self.create_timer(0.1, self.run)

    def update(self):
        # If a robot is in the blackboard and it doens't exist, it is created
        for ally_robot in self.blackboard.gui.robots:
            try:
                self.robots[ally_robot.id]
            except:
                self.robots[ally_robot.id] = Robot(ally_robot.id, ally_robot.name)
                self._executor.add_node(self.robots[ally_robot.id])

        # If a robot is not in the blackboard, it is destroyed
        # The list is used to avoid "RuntimeError: dictionary changed size during iteration"
        if len(self.robots) > self.blackboard.gui.robot_count:
            for robot in list(self.robots.values()):
                if robot.id not in [
                    ally_robot.id for ally_robot in self.blackboard.gui.robots
                ]:
                    self._executor.remove_node(self.robots[robot.id])
                    self.robots[robot.id].destroy_node()
                    self.robots.pop(robot.id)

    def run(self):
        self.get_logger().info(f"Running")
        # The code below just create a simple behaviour tree which is available in strategy
        # log_tree.level = log_tree.Level.DEBUG
        # tree = make_bt(self.blackboard.referee.command)
        # tree.tick_once()
        # self.behaviour_tree.run(self.blackboard)


def main(args=None):
    rclpy.init(args=args)
    coach = Coach(None)
    rclpy.spin(coach)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from pynput import keyboard
from threading import Thread
from time import time

from system_interfaces.msg import TeamCommand, RobotCommand


class ManualCommand(Node):
    def __init__(self) -> None:
        super().__init__("manual_command")
        self.command_publisher = self.create_publisher(TeamCommand, "commandTopic", 10)
        self.create_timer(1 / 60, self.publish_command)
        self.key_press_times = {}
        self.velocities = {"w": 0.0, "s": 0.0, "a": 0.0, "d": 0.0, "q": 0.0, "e": 0.0}
        self.max_velocity = 1.0  # Maximum velocity limit
        self.acceleration_rate = 0.1  # Rate at which velocity increases
        self.decay+_rate = 0.01  # Rate at which velocity decays

    def publish_command(self):
        self.get_logger().info("Publishing command")
        command = self.create_command()
        self.command_publisher.publish(command)

    def create_command(self):
        vx = self.calculate_velocity("w", False) + self.calculate_velocity("s", True)
        vy = self.calculate_velocity("a", False) + self.calculate_velocity("d", True)
        vt = self.calculate_velocity("q", False) + self.calculate_velocity("e", True)
        print(vx, vy, vt)
        robot_command = RobotCommand()
        robot_command.robot_id = 0
        robot_command.linear_velocity_x = vx
        robot_command.linear_velocity_y = vy
        robot_command.angular_velocity = vt
        robot_command.kick = False
        command = TeamCommand()
        command.robots.append(robot_command)

        return command

    def calculate_velocity(self, key, is_negative_key):
        current_time = time()
        if key in self.key_press_times:
            elapsed_time = current_time - self.key_press_times[key]
            self.velocities[key] = min(
                self.max_velocity, elapsed_time * self.acceleration_rate
            )
            if is_negative_key:
                self.velocities[key] *= -1
        else:
            # Apply decay
            if self.velocities[key] > 0:
                self.velocities[key] = max(0.0, self.velocities[key] - self.decay_rate)
            elif self.velocities[key] < 0:
                self.velocities[key] = min(0.0, self.velocities[key] + self.decay_rate)
        return self.velocities[key]

    def on_press(self, key):
        try:
            if (
                key.char in ["w", "a", "s", "d", "q", "e"]
                and key.char not in self.key_press_times
            ):
                self.key_press_times[key.char] = time()
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key.char in self.key_press_times:
                del self.key_press_times[key.char]
        except AttributeError:
            pass


def main():
    rclpy.init()
    manual_command = ManualCommand()
    thread = Thread(target=rclpy.spin, args=(manual_command,))
    thread.start()
    listener = keyboard.Listener(
        on_press=manual_command.on_press, on_release=manual_command.on_release
    )
    listener.start()
    listener.join()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

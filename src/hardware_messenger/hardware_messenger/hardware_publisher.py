from rclpy.node import Node
import rclpy
import serial
import struct

from grsim_messenger.protobuf.grSim_Packet_pb2 import grSim_Packet
from grsim_messenger.protobuf.grSim_Commands_pb2 import grSim_Robot_Command

from system_interfaces.msg import TeamCommand, GUIMessage
from grsim_messenger.inverse_kinematics import apply_inverse_kinematics


import time


class HardwarePublisher(Node):
    def __init__(self) -> None:
        super().__init__("hardware_publisher")

        # Parameters settings.
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 230400)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )

        # Format string tells struct how to pack the data
        # B: unsigned char (1 byte)
        # f: float (4 bytes)
        name = "B"
        velocities_format = "fff"
        kick = "B"
        control_values = "fff"
        self.unused_count = (
            32
            - len(name)
            - len(kick)
            - (len(control_values) + len(velocities_format)) * 4
        )
        not_used = "B" * self.unused_count
        self.robot_command_format = (
            name + velocities_format + kick + control_values + not_used
        )

        self.robots = {}

        self._command = []
        self.robot_count = 0

        # self.serial_writer = serial.Serial(self.port, self.baudrate)

        self.commandSubscriber = self.create_subscription(
            TeamCommand, "commandTopic", self.translate_command, 10
        )

        self.guiSubscriber = self.create_subscription(
            GUIMessage, "guiTopic", self.gui_callback, 10
        )

        self.timer = self.create_timer(
            1 / 60, self.publish_command
        )  # publish to serial at 60Hz

    def publish_command(self):
        if len(self._command) == 0:
            return
        print(self._command)
        # self.serial_writer.write(struct.pack(self.format_string, *self._command))

    def gui_callback(self, message):
        self.robot_count = message.robot_count

        for robot in message.robots:
            self.robots[robot.id] = robot

    def translate_command(self, command):
        self._command = []

        self.format_string = "=" + self.robot_command_format * self.robot_count

        unused_bytes = [0 for _ in range(self.unused_count)]

        for i in range(self.robot_count):
            if i < len(command.robots):
                robot = command.robots[i]
                robot_command = [
                    ord(self.robots[robot.robot_id].name[0]),
                    float(robot.linear_velocity_x),
                    float(robot.linear_velocity_y),
                    float(robot.angular_velocity),
                    int(robot.kick),
                    float(self.robots[robot.robot_id].kp),
                    float(self.robots[robot.robot_id].ki),
                    float(self.robots[robot.robot_id].kd),
                ]
            else:
                robot_command = [
                    ord("Z"),
                    0.0,
                    0.0,
                    0.0,
                    0,
                    0.0,
                    0.0,
                    0.0,
                ]

            robot_command.extend(unused_bytes)

            self._command.extend(robot_command)


def main(args=None):
    rclpy.init(args=args)
    node = HardwarePublisher()

    rclpy.spin(node)


if __name__ == "__main__":
    main()

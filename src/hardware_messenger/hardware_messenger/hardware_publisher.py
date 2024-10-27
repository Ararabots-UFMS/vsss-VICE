from rclpy.node import Node
import rclpy
import serial
import struct

from grsim_messenger.protobuf.grSim_Packet_pb2 import grSim_Packet
from grsim_messenger.protobuf.grSim_Commands_pb2 import grSim_Robot_Command

from system_interfaces.msg import TeamCommand, GUIMessage
from system_interfaces.srv import ConnectRobot
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
        not_used = "B" * (32 - len(name) - len(kick) - len(velocities_format) * 4)
        self.robot_command_format = name + velocities_format + kick + not_used

        self.robot_names = {0: "J", 1: "K", 2: "E"}

        self._command = []

        self.serial_writer = serial.Serial(self.port, self.baudrate)

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
        self.serial_writer.write(struct.pack(self.format_string, *self._command))

    def gui_callback(self, message):
        # self.robot_names = message.robot_names
        pass

    # TODO: Add address
    def translate_command(self, command):
        self._command = []

        self.format_string = "=" + self.robot_command_format * 3

        unused_bytes = [0 for _ in range(14, 32)]

        for i in range(3):
            if 7 < len(command.robots):
                robot = command.robots[i]
                robot_command = [
                    ord(self.robot_names[robot.robot_id]),
                    float(robot.linear_velocity_x),
                    float(robot.linear_velocity_y),
                    float(robot.angular_velocity),
                    int(robot.kick),
                ]
            else:
                robot_command = [
                    ord(self.robot_names[i]),
                    0.0,
                    0.0,
                    0.0,
                    0,
                ]

            robot_command.extend(unused_bytes)

            self._command.extend(robot_command)


def main(args=None):
    rclpy.init(args=args)
    node = HardwarePublisher()

    rclpy.spin(node)


if __name__ == "__main__":
    main()

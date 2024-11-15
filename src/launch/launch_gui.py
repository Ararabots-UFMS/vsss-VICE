from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Node(
            #     package="gui",
            #     executable="run_gui",
            #     name="gui",
            # ),
            Node(
                package="gui_interpreter",
                executable="apiNode",
                name="apiNode",
            ),
            Node(
                package="control_unit",
                executable="mainNode",
                name="main",
            ),
            Node(
                package="referee",
                executable="referee_node",
                name="referee",
            ),
            Node(
                package="vision",
                executable="visionNode",
                name="vision",
            ),
        ]
    )

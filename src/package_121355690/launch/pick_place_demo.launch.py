from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "panda", package_name="project4_moveit_config"
    ).to_dict()

    demo_node = Node(
        package="package_121355690",
        executable="package_121355690",
        output="screen",
        parameters=[
            moveit_config,
            {"use_sim_time": False},
        ],
    )

    # Delay 8 seconds to give move_group time to fully initialize
    # before the MoveGroupInterface tries to connect
    return LaunchDescription([
        TimerAction(period=8.0, actions=[demo_node])
    ])

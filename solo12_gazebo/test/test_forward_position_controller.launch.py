from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("solo12_gazebo"),
            "config",
            "solo12_forward_position_publisher.yaml",
        ]
    )

    return LaunchDescription(
        [
            # sudo apt install ros-humble-ros2-controllers-test-nodes
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_forward_position_controller",
                name="publisher_forward_position_controller",
                parameters=[position_goals],
                output="both",
            )
        ]
    )

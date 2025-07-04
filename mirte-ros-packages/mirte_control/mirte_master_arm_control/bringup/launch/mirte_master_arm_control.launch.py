# Copyright 2023 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    start_controller_manager = LaunchConfiguration("start_controller_manager")
    start_state_publishers = LaunchConfiguration("start_state_publishers")

    declared_arguments = [
        DeclareLaunchArgument(
            "frame_prefix",
            default_value="",
            description="An arbitrary prefix to add to the published tf2 frames. Defaults to the empty string.",
        ),
        DeclareLaunchArgument(
            "start_ros2_control",
            default_value="true",
            description="A boolean whether this launchfile needs to start the ros2 controller manager. Defaults to true.",
        ),
        DeclareLaunchArgument(
            "start_state_publishers",
            default_value="true",
            description="A boolean whether this launchfile needs to start the state publisher and joint boardcaster. Defaults to true.",
        ),
    ]

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("mirte_master_description"),
                    "urdf",
                    "mirte_master.xacro",
                ]
            ),
        ]
    )

    robot_description = {
        "robot_description": robot_description_content,
        "frame_prefix": LaunchConfiguration("frame_prefix"),
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("mirte_master_arm_control"),
            "config",
            "mirte_master_arm_control.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ParameterFile(robot_controllers, allow_substs=True)],
        output="both",
        remappings=[
            ("~/robot_description", "robot_description"),
            ("~/tf_odometry", "/tf"),
        ],
        condition=IfCondition(start_controller_manager),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        condition=IfCondition(start_state_publishers),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        condition=IfCondition(start_state_publishers),
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mirte_master_arm_controller", "mirte_master_gripper_controller"],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)

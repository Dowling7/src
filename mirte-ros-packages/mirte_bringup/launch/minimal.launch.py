import platform

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "machine_namespace",
                default_value=TextSubstitution(
                    text=""  # platform.node().replace("-", "_").lower()
                ),
                description="The namespace containing all Robot specific ROS communication",
            ),
            DeclareLaunchArgument(
                "hardware_namespace",
                default_value="io",
                description="The namespace for the Telemetrix Node and the hardware peripherals",
            ),
        ],
    )

    machine_namespace = LaunchConfiguration("machine_namespace")
    hardware_namespace = LaunchConfiguration("hardware_namespace")
    frame_prefix = ""  # LaunchConfiguration( # No frame prefixes as that does not work with moveit/nav2 and the odom topic must be prefixed instead of the frames.
    #     "_frame_prefix", default=[machine_namespace, "/"]
    # )

    # Make configurable
    telemetrix = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("mirte_telemetrix_cpp"),
                "launch",
                "telemetrix.launch.py",
            ]
        ),
        launch_arguments={
            "config_path": PathJoinSubstitution(
                [
                    FindPackageShare("mirte_bringup"),
                    "telemetrix_config",
                    "mirte_user_config.yaml",
                ]
            ),
            "hardware_namespace": hardware_namespace,
            "frame_prefix": frame_prefix,
        }.items(),
    )

    diff_drive_control = GroupAction(
        actions=[
            SetRemap(
                dst="/mirte_base_controller/cmd_vel",
                src="/mirte_base_controller/cmd_vel_unstamped",
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare("mirte_control"), "launch", "mirte.launch.py"]
                ),
                launch_arguments={"frame_prefix": frame_prefix}.items(),
            ),
        ]
    )

    usb_cam = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("mirte_bringup"), "launch", "camera.launch.py"]
        ),
    )

    rosbridge = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml",
            ]
        ),
    )
    # Instead of this, we could add a conditional to the launch argument declarations
    # to only launch when the condition is not set. By means of LaunchConfigurationEquals
    ld.add_action(
        GroupAction(
            [
                PushRosNamespace(machine_namespace),
                telemetrix,
                diff_drive_control,
                rosbridge,
                usb_cam,
            ],
            launch_configurations={
                arg.name: LaunchConfiguration(arg.name)
                for arg in ld.get_launch_arguments()
            },
            forwarding=False,
        )
    )
    return ld

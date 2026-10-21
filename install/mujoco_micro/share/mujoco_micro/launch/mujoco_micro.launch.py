import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('mujoco_micro')
    default_config = os.path.join(package_share, 'config', 'mujoco_micro.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to the controller YAML file.',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='true: calculate and publish debug data, but keep all six motors disabled.',
        ),
        DeclareLaunchArgument(
            'require_rc',
            default_value='true',
            description='Require valid DJI RC data for calibration and arming.',
        ),
        DeclareLaunchArgument(
            'velocity_command_enable',
            default_value='true',
            description='Enable forward velocity commands from the RC stick.',
        ),
        DeclareLaunchArgument(
            'policy_enable',
            default_value='true',
            description='Enable the ONNX residual balance policy.',
        ),
        DeclareLaunchArgument(
            'recovery_enable',
            default_value='true',
            description='Run recovery_policy.onnx before normal balance when switch 3 is selected.',
        ),
        Node(
            package='mujoco_micro',
            executable='mujoco_micro_node',
            name='mujoco_micro_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('config'),
                {
                    'control.dry_run': ParameterValue(
                        LaunchConfiguration('dry_run'), value_type=bool),
                    'safety.require_rc': ParameterValue(
                        LaunchConfiguration('require_rc'), value_type=bool),
                    'rc.velocity_command_enable': ParameterValue(
                        LaunchConfiguration('velocity_command_enable'), value_type=bool),
                    'policy.enable': ParameterValue(
                        LaunchConfiguration('policy_enable'), value_type=bool),
                    'recovery.enable': ParameterValue(
                        LaunchConfiguration('recovery_enable'), value_type=bool),
                },
            ],
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
   return LaunchDescription([
      DeclareLaunchArgument(
            'enable_recording',
            default_value='False'
        ),

        Node(
            package='roomba',
            executable='walker',
            name='roomba',
            output='screen',
        ),

    # record all topic (chatter, service_node)
      ExecuteProcess(
            condition=IfCondition(
                PythonExpression([
                    LaunchConfiguration('enable_recording')
                ])
            ),
            cmd=[[
                'rm -rf rosbag_recording/ && ros2 bag record -a -o rosbag_recording -x "/camera/*" ',
            ]],
            shell=True
        ),

   ])
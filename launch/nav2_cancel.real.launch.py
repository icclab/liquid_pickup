from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='liquid_pickup',
            executable='cancel_nav2_node',
            name='nav2_cancel',
            namespace='summit',
            parameters=[{"use_sim_time": False}],
            output='screen',
            emulate_tty=True,
        ),
    ])

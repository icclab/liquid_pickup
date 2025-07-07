from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    coordinates = LaunchConfiguration('coordinates')    

    coordinates_launch_arg = DeclareLaunchArgument(
        'coordinates',
        default_value='"[[0.5, 0], [0.8, 0], [1.0, 0.0]]"'
    )

    return LaunchDescription([

        coordinates_launch_arg,

        Node(
            package='liquid_pickup',
            executable='liquid_pickup_node',
            name='bt_node',
            namespace='summit',
            parameters=[{"bt_xml": "liquid_pickup.xml"}, {"use_sim_time": True}, {"coordinates": coordinates}, {"moveit_velocity_scale": 0.09}, {"moveit_acceleration_scale": 0.09}],
            output='screen',
            remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static')],
            emulate_tty=True,
        ),
    ])

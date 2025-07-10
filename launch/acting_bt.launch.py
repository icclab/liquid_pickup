from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    coordinates = LaunchConfiguration('coordinates')   
    bt_xml = LaunchConfiguration('bt_xml')  

    coordinates_launch_arg = DeclareLaunchArgument(
        'coordinates',
        default_value='"[[0.5, 0], [0.8, 0], [1.0, 0.0]]"'
    )

    bt_launch_arg = DeclareLaunchArgument(
        'bt_xml',
        default_value="acting_tree.xml"
    )

    return LaunchDescription([

        coordinates_launch_arg,
        bt_launch_arg,

        Node(
            package='liquid_pickup',
            executable='liquid_pickup_node',
            name='bt_node',
            namespace='summit',
            # parameters=[{"bt_xml": bt_xml}, {"use_sim_time": False}, {"coordinates": coordinates}, {"moveit_velocity_scale": 0.18}, {"moveit_acceleration_scale": 0.12}],
            parameters=[{"bt_xml": bt_xml}, {"use_sim_time": False}, {"coordinates": coordinates}],
            output='screen',
            remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static')],
            emulate_tty=True,
        ),
    ])

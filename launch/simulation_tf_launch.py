from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # # for testing
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_footprint_to_liquid_frame',
        #     namespace='summit',
        #     parameters=[{'use_sim_time': True}],
        #     # arguments = ['--x', '0.6', '--y', '0.01', '--z', '0.5', '--roll', '0.01', '--pitch', '3.14', '--yaw', '0.01', '--frame-id', 'base_footprint', '--child-frame-id', 'liquid'], # for manipulation
        #     arguments = ['--x', '3.0', '--y', '0.0', '--z', '0.0', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'map', '--child-frame-id', 'liquid'], # for navigation
        #     remappings=[('/tf_static', '/summit/tf_static')],
        # ),
        
        # # for simulation testing
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_footprint_to_liquid_frame',
        #     namespace='summit',
        #     parameters=[{'use_sim_time': True}],
        #     arguments = ['--x', '3.6', '--y', '0.0', '--z', '0.5', '--roll', '0.0', '--pitch', '3.14', '--yaw', '0.0', '--frame-id', 'map', '--child-frame-id', 'liquid_sample'], # for simulation
        #     remappings=[('/tf_static', '/summit/tf_static')],
        # ),

        # for port to map static tf
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='port_to_map_frame',
            namespace='summit',
            parameters=[{'use_sim_time': True}],
            arguments = ['--x', '0.0', '--y', '0.0', '--z', '0.0', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'port', '--child-frame-id', 'map'],
            remappings=[('/tf_static', '/summit/tf_static')],
        ),

        # for port to deploy sensor s1 static tf
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='port_to_s1_frame',
            namespace='summit',
            parameters=[{'use_sim_time': True}],
            arguments = ['--x', '1.0', '--y', '0.0', '--z', '0.0', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'port', '--child-frame-id', 's1'],
            remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static')],
        ),

        # for port to deploy sensor s2 static tf
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='port_to_s2_frame',
            namespace='summit',
            parameters=[{'use_sim_time': True}],
            arguments = ['--x', '2.0', '--y', '0.0', '--z', '0.0', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'port', '--child-frame-id', 's2'],
            remappings=[('/tf_static', '/summit/tf_static')],
        ),

        # for port to deploy sensor s3 static tf
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='port_to_s3_frame',
            namespace='summit',
            parameters=[{'use_sim_time': True}],
            arguments = ['--x', '3.0', '--y', '0.0', '--z', '0.0', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'port', '--child-frame-id', 's3'],
            remappings=[('/tf_static', '/summit/tf_static')],
        ),
    ])

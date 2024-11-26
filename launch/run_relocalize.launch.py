from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    parameter_navsat_file = LaunchConfiguration('navsat_transform_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    #rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')
    rviz_config_file = os.path.join(share_dir, 'config', 'base.rviz')
    #rviz_config_file = os.path.join(share_dir, 'config', 'video.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    print("urdf_file_name : {}".format(xacro_path))

    return LaunchDescription([

        DeclareLaunchArgument(
            'navsat_transform_file',
            default_value=os.path.join(
               share_dir, 'config', 'navsat_transform.yaml'),
           description='Navsat transform configuration file'),
        params_declare,

        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path])
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
        # Aggiunta del nodo EKF GPS
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_gps',
            respawn=True,
            output='screen',
            parameters=[parameter_navsat_file],
            remappings=[
                ('odometry/filtered', '/odometry/navsat'),
            ],
            #arguments=['--ros-args', '--log-level', 'debug']
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[parameter_navsat_file],
            remappings=[
                ('imu', '/oxts/imu'),  # Input IMU
                ('gps/fix', '/oxts/nav_sat_fix'),  # Input GPS
                ('odometry/filtered', '/odometry/navsat'),  # Input Odometry
                ('gps/filtered', 'gps/filtered'),  # GPS filtrato (output)
                ('odometry/gps', 'odometry/gps'),  # Odometry GPS (output)
            ]
        ),
        Node(
            package='lio_sam',  
            executable='global_localize',  
            name='global_localize',
            output='screen',
            parameters=[parameter_file],
        ),
    ])
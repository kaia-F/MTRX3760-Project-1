import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    sim_map = LaunchConfiguration('sim_map', default='closedmaze')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf_file_name = 'turtlebot3_burger_cam.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name)
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
        
    rviz_config_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'rviz', 
        'tb3_cartographer.rviz')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("turtlebot3_gazebo"), '/launch/', sim_map, '.launch.py'
        ])
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("turtlebot3_cartographer"), '/launch/cartographer.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    
    camera_node = Node(
        package='turtlebot3_gazebo',
        executable='camera_node',
        name='camera_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    pose_trajectory_node = Node(
        package='turtlebot3_gazebo',
        executable='pose_trajectory_node',
        name='pose_trajectory_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    turtlebot3_drive_node = Node(
        package='turtlebot3_gazebo',
        executable='turtlebot3_drive',
        name='turtlebot3_drive',
        parameters=[{'use_sim_time': use_sim_time}]
    )
                                                 
    return LaunchDescription([
     
        DeclareLaunchArgument('sim_map', default_value='closedmaze', description='Name of the Gazebo world file'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        
        # Launch files and nodes
        gazebo_launch,
        robot_state_publisher_node,
        cartographer_launch,
        rviz_node,
        
        # Your custom nodes
        camera_node,
        pose_trajectory_node,
        turtlebot3_drive_node
    ])

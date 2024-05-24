import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_turtlebot4')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    # Start a Webots simulation instance
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    # Create the robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    # wheel_drop_left and right
    tf_wheel_drop_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wheel_drop_left_to_base_link',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'wheel_drop_left'],
    )

    tf_wheel_drop_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wheel_drop_right_to_base_link',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'wheel_drop_right'],
    )


    # ROS control spawners
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yaml')
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    ros_control_spawners = [joint_state_broadcaster_spawner, diffdrive_controller_spawner]
    mappings = [
        ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), 
        ('/diffdrive_controller/odom', '/odom'),
        ('/Turtlebot4/rplidar', '/scan'),
        ('/Turtlebot4/oakd_stereo_camera/point_cloud', '/depth_camera')
        ]    

    # Create a ROS node interacting with the simulated robot
    robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot4.urdf')
    robot_driver = WebotsController(
        robot_name='Turtlebot4',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True
             },
            ros2_control_params
        ],
        remappings=mappings
    )

    # Wait for the simulation to be ready to start navigation nodes
    waiting_nodes = WaitForControllerConnection(
        target_driver=robot_driver,
        nodes_to_start= ros_control_spawners
    )    

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='turtlebot4_world.wbt',
            description='Choose one of the world files from `/webots_ros2_turtlebot/world` directory'
        ),           
        webots,
        webots._supervisor,
        robot_state_publisher,
        tf_wheel_drop_left,
        tf_wheel_drop_right,
        robot_driver,
        waiting_nodes,
        # The following action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
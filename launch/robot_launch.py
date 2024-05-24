import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_turtlebot4')
    

    # Start a Webots simulation instance
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'turtlebot4_world.wbt')
    )


    # ROS control spawners
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yaml')

    # Create a ROS node interacting with the simulated robot
    robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot4.urdf')
    robot_driver = WebotsController(
        robot_name='Turtlebot4',
        parameters=[
            {'robot_description': robot_description_path},
            ros2_control_params
        ]
    )

    return LaunchDescription([
        webots,
        robot_driver,

        # The following action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
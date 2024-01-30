from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    rviz_config =  Path(get_package_share_directory('intro_robot'), 'rviz', 'giraff_robot.rviz').resolve()
    assert rviz_config.is_file()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf_file = Path(get_package_share_directory('intro_robot'), 'urdf', 'giraff_robot.urdf').resolve()
    assert urdf_file.is_file()

    world_path = Path(get_package_share_directory('intro_robot'), 'worlds', 'world.world').resolve()
    assert world_path.is_file()

    # Declare launch arguments
    world_ = DeclareLaunchArgument(
        'use_world',
        description='Which world to use for Gazebo',
        default_value=str(world_path),
    )


    urdf = DeclareLaunchArgument(
        'use_urdf',
        description='Full path to urdf',
        default_value=str(urdf_file),
    )

    sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )


    gazebo_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'giraff_robot',
            '-file', str(urdf_file),
      ])



    robot_controller_node = Node(
        package='intro_robot',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        emulate_tty=True,  # Keeps color in the terminal
    )


    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config)],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    tf_publishing_node = Node(
        package='intro_robot',  # Replace with the actual name of your ROS 2 package
        executable='giraff_robot',  # Replace with the name of your node executable
        name='giraff_robot',
        output='screen',
	parameters=[{'robot_description' : LaunchConfiguration('use_urdf')}],

    )

    return LaunchDescription([
       	urdf,
        world_,
        sim_time,
        gazebo_node,
        robot_controller_node,
        rviz_node,
        tf_publishing_node,
        # Used to launch gazebo with the world file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch'), 
                '/gazebo.launch.py']),
                launch_arguments=[
                ('world', LaunchConfiguration('use_world')),
            ],
        ),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'wayland'),
    ])

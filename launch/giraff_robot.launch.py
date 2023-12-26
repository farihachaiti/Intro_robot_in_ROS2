from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import sys
from intro_robot.giraff_robot import FramePublisher

def generate_launch_description():

    rviz_config =  Path(get_package_share_directory('intro_robot'), 'rviz', 'giraff_robot.rviz').resolve()
    assert rviz_config.is_file()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf_file = Path(get_package_share_directory('intro_robot'), 'urdf', 'giraff_robot.urdf').resolve()
    assert urdf_file.is_file()

    #tf_publisher = Path(get_package_share_directory('intro_robot'), 'src', 'giraff_robot.py').resolve()
    #assert tf_publisher.is_file()



    # Declare launch arguments


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

    #tf_publisher_arg = DeclareLaunchArgument(
    #    'use_tf_publisher',
    #     default_value='giraff_robot.py',
    #     description='Use simulation (Gazebo) clock if true',
    #)


    state_publisher_node = Node(
         package='robot_state_publisher',
         executable='robot_state_publisher',
         name='robot_state_publisher',
         output='screen',
         parameters=[{'use_sim_time': use_sim_time},
                     {'robot_description': LaunchConfiguration('use_urdf')}],
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
        #arguments=['-d', str(tf_publisher)],
	parameters=[{'robot_description' : LaunchConfiguration('use_urdf')}],

    )

    return LaunchDescription([
       	urdf,
        sim_time,
	#tf_publisher_arg,
        #state_publisher_node,
        rviz_node,
        tf_publishing_node,
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'wayland'),
    ])

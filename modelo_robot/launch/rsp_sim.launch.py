import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument



from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg_name = 'modelo_robot'
    file_subpath = 'urdf/robot.urdf.xacro'
    world_file_name = 'worlds/worldH.world'


    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    world = os.path.join(get_package_share_directory(pkg_name), world_file_name)

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'world': world}.items(),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'jetbot'],
                    output='screen')
    
    sim_mode_dec = DeclareLaunchArgument('sim_mode', default_value='false')
    sim_mode = LaunchConfiguration('sim_mode')
    tracker_params_sim = os.path.join(get_package_share_directory(pkg_name),'config','ball_tracker_params_sim.yaml')
    tracker_params_robot = os.path.join(get_package_share_directory(pkg_name),'config','ball_tracker_params_robot.yaml')
    

    params_path = PythonExpression(['"',tracker_params_sim, '" if "true" == "', sim_mode, '" else "', tracker_params_robot, '"'])

    tracker_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ball_tracker'), 'launch', 'ball_tracker.launch.py')]),
                    launch_arguments={'params_file': params_path,
                                    'image_topic': '/my_camera/image_raw',
                                    'cmd_vel_topic': '/cmd_vel_tracker',
                                    'enable_3d_tracker': 'true'}.items())


    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        sim_mode_dec,
        tracker_launch,
    ])



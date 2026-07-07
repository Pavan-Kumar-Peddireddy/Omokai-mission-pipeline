import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Paths to our package resources
    bringup_dir = get_package_share_directory('mission_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 1. Environment Configurations
    world_path = os.path.join(bringup_dir, 'worlds', 'turtlebot3_house.sdf')
    map_yaml_file = os.path.join(bringup_dir, 'maps', 'home.yaml')
    nav2_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    urdf_file = os.path.join(bringup_dir, 'urdf', 'diff_bot.urdf')
    models_dir = os.path.join(bringup_dir, 'models')
    bridge_config_file = os.path.join(bringup_dir, 'config', 'gz_bridge.yaml')

    # turtlebot3_house.sdf references <uri>model://turtlebot3_house</uri>.
    # Without GZ_SIM_RESOURCE_PATH pointing at our models/ dir, Gazebo can't
    # resolve that URI. Preserve any resource path already set rather than
    # clobbering it.
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = os.pathsep.join(filter(None, [models_dir, existing_resource_path]))
    set_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path)

    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF File not found at shared package path: {urdf_file}")

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 2. Nodes & Launch Inclusions

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # Gazebo Sim (Gazebo Harmonic, ROS 2 Jazzy's official pairing).
    # '-s' = server only, no GUI -- avoids a GUI/sensors render-thread race
    # that segfaults gz-sim-sensors-system on this machine's hybrid graphics.
    # RViz2 (below) handles visualization instead.
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=list({'gz_args': f'-r -s {world_path}'}.items()),
    )

    rviz_config_file = os.path.join(bringup_dir, 'config', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'diff_bot',
            '-file', urdf_file,
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # YAML-based bridge config (not CLI args) so we can rename Gazebo's
    # default per-model TF topic (model/diff_bot/tf) to ROS's standard /tf.
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{'config_file': bridge_config_file}],
        output='screen'
    )

    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments=list({
            'map': map_yaml_file,
            'params_file': nav2_params_file,
            'use_sim_time': 'True'
        }.items()),
    )

    return LaunchDescription([
        LogInfo(msg="Spawning Omokai Simulation + Nav2 stack (no LLM/executor yet)..."),
        set_resource_path,
        robot_state_publisher_node,
        gazebo_sim,
        rviz_node,
        spawn_robot,
        gz_bridge,
        nav2_stack,
    ])
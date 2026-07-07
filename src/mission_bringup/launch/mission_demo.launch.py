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
    bridge_config_file = os.path.join(bringup_dir, 'config', 'gz_bridge.yaml')
    
    # Environment Configurations
    world_path = os.path.join(bringup_dir, 'worlds', 'turtlebot3_house.sdf')
    map_yaml_file = os.path.join(bringup_dir, 'maps', 'home.yaml')
    nav2_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    
    # 1. NEW: Defined both URDF and SDF paths
    urdf_file = os.path.join(bringup_dir, 'urdf', 'diff_bot.urdf')
    sdf_file = os.path.join(bringup_dir, 'urdf', 'diff_bot.sdf') 
    models_dir = os.path.join(bringup_dir, 'models')

    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = os.pathsep.join(filter(None, [models_dir, existing_resource_path]))
    set_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path)

    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF File not found at shared package path: {urdf_file}")

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher consumes the URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=list({
            'gz_args': f' -r {world_path}'  # Render engine argument removed
        }.items()),
    
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

    # 2. FIXED: Spawn Robot now consumes the SDF instead of the URDF
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'diff_bot',
            '-file', urdf_file,  # Swapped from urdf_file to sdf_file
            '-x', '-7', '-y', '-1', '-z', '0.1'
        ],
        output='screen'
    )

    # 3. FIXED: Forced use_sim_time on the bridge

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # '/diff_bot/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # '/model/diff_bot/tf@tf2_msgs/msg/TFMessage[rosgraph_msgs/msg/Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/diff_bot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[('/model/diff_bot/tf', '/tf')],
        # remappings=[
        #     ('/diff_bot/odom', '/odom'),
        #     ('/model/diff_bot/tf', '/tf')
        # ],
        output='screen'
    )


    # 4. REMOVED: odom_to_tf_node was completely deleted to prevent TF conflicts.

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
        LogInfo(msg="Spawning Omokai Simulation + Nav2 stack (Fixed TF & Clock syncing)..."),
        set_resource_path,
        robot_state_publisher_node,
        gazebo_sim,
        rviz_node,
        spawn_robot,
        gz_bridge,
        nav2_stack,
        
   
    
    ])
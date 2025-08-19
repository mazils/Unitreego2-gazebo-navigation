from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Paths
    default_world = '/home/mazils/Desktop/ros2test/world.sdf'  # adjust if you keep old name

    args = [
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Point cloud & rate
        DeclareLaunchArgument('pointcloud_topic', default_value='/velodyne_points'),
        DeclareLaunchArgument('publish_rate_hz', default_value='10.0'),

        # Geometry
        DeclareLaunchArgument('front_half_angle_deg', default_value='20.0'),
        DeclareLaunchArgument('side_half_angle_deg', default_value='80.0'),
        DeclareLaunchArgument('side_gap_deg', default_value='5.0'),

        # Filters
        DeclareLaunchArgument('min_height', default_value='-1.0'),
        DeclareLaunchArgument('max_height', default_value='2.0'),
        DeclareLaunchArgument('min_radius', default_value='0.05'),
        DeclareLaunchArgument('max_radius', default_value='0.0'),
        DeclareLaunchArgument('max_considered_range', default_value='8.0'),

        # Forward decision
        DeclareLaunchArgument('forward_clear_min', default_value='0.60'),
        DeclareLaunchArgument('forward_advantage_ratio', default_value='1.18'),
        DeclareLaunchArgument('forward_margin', default_value='0.35'),
        DeclareLaunchArgument('extra_forward_slack', default_value='0.15'),
        DeclareLaunchArgument('min_forward_speed', default_value='0.08'),

        # Sides
        DeclareLaunchArgument('side_diff_min', default_value='0.12'),

        # Speeds
        DeclareLaunchArgument('max_linear_speed', default_value='0.55'),
        DeclareLaunchArgument('max_angular_speed', default_value='1.60'),
        DeclareLaunchArgument('min_turn_angular_speed', default_value='0.45'),

        # Scaling
        DeclareLaunchArgument('pivot_front_thresh', default_value='2.2'),
        DeclareLaunchArgument('linear_scale_stop', default_value='1.0'),
        DeclareLaunchArgument('disable_forward_scaling', default_value='false'),
        DeclareLaunchArgument('linear_override', default_value='0.0'),

        # Bias & creep
        DeclareLaunchArgument('prefer_left', default_value='true'),
        DeclareLaunchArgument('turn_creep_enabled', default_value='true'),
        DeclareLaunchArgument('turn_creep_fraction', default_value='0.15'),
        DeclareLaunchArgument('turn_creep_after_cycles', default_value='6'),
        DeclareLaunchArgument('turn_creep_front_min', default_value='0.90'),

        # Smoothing / sampling
        DeclareLaunchArgument('smoothing_alpha', default_value='0.35'),
        DeclareLaunchArgument('downsample_stride', default_value='1'),

        # Logging
        DeclareLaunchArgument('debug', default_value='true'),
        DeclareLaunchArgument('log_every_n', default_value='10'),
    ]

    # Gazebo Classic launch (gzserver + gzclient)
    # If you use Ignition/Garden/Fortress, replace with appropriate include.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.getenv('COLCON_PREFIX_PATH', '/opt/ros/humble').split(':')[0],
                'share', 'gazebo_ros', 'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    navigator = Node(
        package='front_obstacle_detector',
        executable='front_obstacle_detector',
        name='space_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            'front_half_angle_deg': LaunchConfiguration('front_half_angle_deg'),
            'side_half_angle_deg': LaunchConfiguration('side_half_angle_deg'),
            'side_gap_deg': LaunchConfiguration('side_gap_deg'),
            'min_height': LaunchConfiguration('min_height'),
            'max_height': LaunchConfiguration('max_height'),
            'min_radius': LaunchConfiguration('min_radius'),
            'max_radius': LaunchConfiguration('max_radius'),
            'max_considered_range': LaunchConfiguration('max_considered_range'),
            'forward_clear_min': LaunchConfiguration('forward_clear_min'),
            'forward_advantage_ratio': LaunchConfiguration('forward_advantage_ratio'),
            'forward_margin': LaunchConfiguration('forward_margin'),
            'extra_forward_slack': LaunchConfiguration('extra_forward_slack'),
            'min_forward_speed': LaunchConfiguration('min_forward_speed'),
            'side_diff_min': LaunchConfiguration('side_diff_min'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'min_turn_angular_speed': LaunchConfiguration('min_turn_angular_speed'),
            'pivot_front_thresh': LaunchConfiguration('pivot_front_thresh'),
            'linear_scale_stop': LaunchConfiguration('linear_scale_stop'),
            'disable_forward_scaling': LaunchConfiguration('disable_forward_scaling'),
            'linear_override': LaunchConfiguration('linear_override'),
            'prefer_left': LaunchConfiguration('prefer_left'),
            'turn_creep_enabled': LaunchConfiguration('turn_creep_enabled'),
            'turn_creep_fraction': LaunchConfiguration('turn_creep_fraction'),
            'turn_creep_after_cycles': LaunchConfiguration('turn_creep_after_cycles'),
            'turn_creep_front_min': LaunchConfiguration('turn_creep_front_min'),
            'smoothing_alpha': LaunchConfiguration('smoothing_alpha'),
            'downsample_stride': LaunchConfiguration('downsample_stride'),
            'debug': LaunchConfiguration('debug'),
            'log_every_n': LaunchConfiguration('log_every_n'),
        }]
    )

    # OPTIONAL: If later you have a real occupancy map YAML (NOT this SDF), uncomment below:
    # from launch_ros.actions import Node as RosNode
    # map_server = RosNode(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{'yaml_filename': '/path/to/real_map.yaml',
    #                  'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )

    return LaunchDescription(args + [gazebo_launch, navigator])
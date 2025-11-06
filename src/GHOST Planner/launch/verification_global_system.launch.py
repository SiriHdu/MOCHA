import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ghost_planner')
    params_file = os.path.join(pkg_share, 'config', 'global_planner.yaml')
    rviz_file = os.path.join(pkg_share, 'rviz', 'global_planner.rviz')

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')

    # Nodes
    # 5 agents: robot_1 .. robot_5
    planners = []
    trackers = []
    tf_nodes = []
    starts = [(0.0, 0.0), (2.0, 0.0), (0.0, 2.0), (4.0, 0.0), (0.0, 4.0)]
    colors = [
        (0.9, 0.1, 0.1),
        (0.1, 0.9, 0.1),
        (0.1, 0.1, 0.9),
        (0.9, 0.9, 0.1),
        (0.9, 0.1, 0.9),
    ]
    # List of all robot IDs for multi-agent collision avoidance
    all_robot_ids = [f"robot_{i+1}" for i in range(5)]
    
    for idx in range(5):
        rid = f"robot_{idx+1}"
        planners.append(Node(
            package='ghost_planner',
            executable='global_planner',
            namespace=rid,
            name='global_planner',
            output='screen',
            parameters=[
                params_file,
                {'self_robot_id': rid},
                {'robot_base_frame': f'{rid}/base_link'},
                {'global_frame': 'map'},
                {'all_robot_ids': all_robot_ids},
                {'idle_robot_radius': 0.3}
            ]
        ))
        trackers.append(Node(
            package='ghost_planner',
            executable='verifation_global_planner',
            name=rid,
            output='screen',
            parameters=[
                {'global_frame': 'map'}, 
                {'playback_dt': 0.02},
                {'start.x': starts[idx][0]},
                {'start.y': starts[idx][1]},
                {'color.r': colors[idx][0]},
                {'color.g': colors[idx][1]},
                {'color.b': colors[idx][2]},
            ],
        ))
        tf_nodes.append(Node(
            package='ghost_planner',
            executable='fake_robot_tf_publisher',
            name=f'{rid}_tf',
            output='screen',
            parameters=[{'start.x': starts[idx][0], 'start.y': starts[idx][1], 'child_frame_id': f'{rid}/base_link'}]
        ))

    tf_pub = Node(
        package='ghost_planner',
        executable='fake_robot_tf_publisher',
        name='fake_robot_tf_publisher',
        output='screen',
        parameters=[{'start.x': -1.0, 'start.y': 0.0, 'child_frame_id': 'base_link'}]
    )

    obstacles = Node(
        package='ghost_planner',
        executable='random_cylinder_publisher',
        name='random_cylinder_publisher',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'obstacles.use_fixed': True,
            'obstacles.fixed_x': [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 16.0, 24.0, 36.0, 44.0, 56.0, 64.0, 76.0, 84.0, 28.0, 52.0, 68.0],
            'obstacles.fixed_y': [10.0, 16.0, 24.0, 36.0, 20.0, 50.0, 80.0, 60.0, 44.0, 28.0, 40.0, 52.0, 64.0, 76.0, 32.0, 48.0, 24.0, 72.0, 68.0, 56.0],
            'obstacles.fixed_r': [2.4, 3.6, 4.4, 3.0, 4.0, 3.8, 3.4, 2.2, 3.2, 4.6, 4.0, 3.6, 4.6, 2.4, 3.8, 4.2, 3.4, 2.0, 3.6, 3.2],
        }]
    )

    tracker = Node(
        package='ghost_planner',
        executable='verifation_global_planner',
        name='verifation_global_planner',
        output='screen',
        parameters=[{'global_frame': 'map'}, {'playback_dt': 0.02}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        obstacles,
        *tf_nodes,
        *planners,
        *trackers,
        rviz,
    ])



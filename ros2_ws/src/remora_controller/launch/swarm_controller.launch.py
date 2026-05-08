from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pattern = LaunchConfiguration('pattern').perform(context)
    robot_count = int(LaunchConfiguration('robot_count').perform(context))

    nodes = []

    hardware_protection_param_file = PathJoinSubstitution([
        FindPackageShare('remora_controller'),
        'config',
        'swarm',
        'burger',
        'hardware_protection_layer.yaml'
    ])

    movement_param_file = PathJoinSubstitution([
        FindPackageShare('remora_controller'),
        'config',
        'swarm',
        'burger',
        'movement_pattern',
        'basic',
        f'{pattern}.yaml'
    ])

    for i in range(robot_count):
        movement_pattern_node = Node(
            package="ros2swarm",
            executable=pattern,
            namespace=f'remora{i}',
            parameters=[movement_param_file],
            output="screen"
        )

        # Hardware protection node - remapped to use /laserscan (all obstacles)
        hardware_protection_node = Node(
            package="ros2swarm",
            executable='hardware_protection_layer',
            namespace=f'remora{i}',
            parameters=[hardware_protection_param_file],
            remappings=[
                ('range_data', 'obstacle_ranges')  # Use laserscan instead of range_data
            ],
            output="screen"
        )

        nodes += [movement_pattern_node, hardware_protection_node]

    return nodes


def generate_launch_description():
    robot_count_arg = DeclareLaunchArgument(
        'robot_count',
        default_value='1',
        description='Number of robots in the swarm',
    )

    pattern_arg = DeclareLaunchArgument(
        'pattern',
        default_value='drive_pattern',
        description='Movement pattern to use',
        choices=[
            'drive_pattern', 'random_walk_pattern', 'dispersion_pattern',
            'aggregation_pattern', 'attraction_pattern', 'attraction_pattern2',
            'minimalist_flocking_pattern', 'adaptive_sampling_pattern'
        ]
    )

    return LaunchDescription([
        robot_count_arg,
        pattern_arg,
        OpaqueFunction(function=launch_setup)
    ])

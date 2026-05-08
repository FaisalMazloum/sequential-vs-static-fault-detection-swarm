from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration




def generate_launch_description():

    tcp_ip_arg = DeclareLaunchArgument(name='ip', default_value="192.168.1.159")
    tcp_ip = LaunchConfiguration('ip')

    
    # endpoint_pkg = get_package_share_directory("ros_tcp_endpoint")
    
    endpoint_node = Node (
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        # parameters=[{"ROS_IP": "192.168.8.141"}],
        # parameters=[{"ROS_IP": "192.168.1.159"}],
        parameters=[{"ROS_IP": "0.0.0.0"}],
    )


    return LaunchDescription(
        [
            endpoint_node,
        ]
    )

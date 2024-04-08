'''
This launch file is equivalent of running the following commands:

ros2 launch qualisys_driver qualisys.launch.py
ros2 run mocap_interface qualisys_pubsub <id> --ros-args --remap __ns:=<robot_name>
ros2 run mocap_interface pose_to_odom --ros-args --remap __ns:=<robot_name>

The launch file starts the qualisys driver, qualisys_pubsub and pose_to_odom nodes.
The qualisys_pubsub node subscribes to the /rigid_bodies topic and publishes the pose of requested id to /pose topic. (robot_name/pose in the case of the launch file)
The pose_to_odom node subscribes to the /pose topic, calculates odometry and publishes the odometry to /odometry topic. (robot_name/odometry in the case of the launch file)
If run from this launch file, the nodes will be namespaced to the robot_name.

Usage:
ros2 launch mocap_interface startup.launch.py id:=<id> robot_name:=<robot_name>

Topics:
- /<robot_name>/pose: Pose of the robot in the world frame
- /<robot_name>/odometry: Odometry of the robot in the world frame

Example:
ros2 launch mocap_interface startup.launch.py id:=11 robot_name:=laura

Topics:
- /laura/pose
- /laura/odometry

'''

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'id',
            default_value='11',
            description='Id of the robot obtained from the qualisys MoCap software'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='laura',
            description='Name of the robot to be used in the namespace'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(get_package_share_directory('qualisys_driver') + '/launch/qualisys.launch.py')
        ),
        Node(
            package='mocap_interface',
            executable='qualisys_pubsub',
            name='qualisys_pubsub',
            namespace=LaunchConfiguration('robot_name'),
            arguments=[LaunchConfiguration('id')]         #, LaunchConfiguration('topic_name')]
        ),
        Node(
            package='mocap_interface',
            executable='pose_to_odom',
            name='pose_to_odom',
            namespace=LaunchConfiguration('robot_name'),
            output='screen',
        )
    ])

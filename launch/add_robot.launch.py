'''
This launch file is used to add a robot to the MoCap system. (In addition to the start.launch.py file)

This launch file is equivalent of running the following commands:

ros2 run mocap_interface qualisys_pubsub <id> --ros-args --remap __ns:=<robot_name>
ros2 run mocap_interface pose_to_odom --ros-args --remap __ns:=<robot_name>

The launch file starts the qualisys_pubsub and pose_to_odom nodes.

The qualisys_pubsub node subscribes to the /rigid_bodies topic and publishes the pose of requested id to /pose topic. (robot_name/pose in the case of the launch file)
The pose_to_odom node subscribes to the /pose topic, calculates odometry and publishes the odometry to /odometry topic. (robot_name/odometry in the case of the launch file)
If run from this launch file, the nodes will be namespaced to the robot_name.

Usage:
ros2 launch mocap_interface add_robot.launch.py id:=<id> robot_name:=<robot_name>

Topics:
- /<robot_name>/pose: Pose of the robot in the world frame
- /<robot_name>/odometry: Odometry of the robot in the world frame

Example:
ros2 launch mocap_interface add_robot.launch.py id:=13 robot_name:=jackal

Topics:
- /jackal/pose
- /jackal/odometry

'''
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'id',
            default_value='13',
            description='Id of the robot obtained from the qualisys MoCap software'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='jackal',
            description='Name of the robot to be used in the namespace'
        ),
        Node(
            package='mocap_interface',
            executable='qualisys_pubsub',
            name='qualisys_pubsub',
            namespace=LaunchConfiguration('robot_name'),
            arguments=[LaunchConfiguration('id')]         
        ),
        Node(
            package='mocap_interface',
            executable='pose_to_odom',
            name='pose_to_odom',
            namespace=LaunchConfiguration('robot_name'),
            output='screen',
        )
    ])

'''
This node subscribes to the /rigid_bodies topic and publishes the pose of requested id to /pose topic. (robot_name/pose in the case of the launch file)
The rigid body id is given as command line argument.

Usage:
ros2 run mocap_interface qualisys_pubsub <id> 

Example:
ros2 run mocap_interface qualisys_subscriber 11  # This will publish the pose of the rigid body with id 11 to /pose topic
'''

import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies
from geometry_msgs.msg import PoseStamped
import sys

class Qualisys(Node):
    def __init__(self):
        super().__init__('qualisys_pubsub')
        self.rigid_body_name = sys.argv[1]
        self.publish_topic = 'pose'
        self.subscription = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(PoseStamped, self.publish_topic, 10)

    def listener_callback(self, msg):
        for rigid_body in msg.rigidbodies:
            if rigid_body.rigid_body_name == self.rigid_body_name:
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                pose_msg.pose = rigid_body.pose
                self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)

    qualisys_pubsub = Qualisys()

    rclpy.spin(qualisys_pubsub)

    qualisys_pubsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

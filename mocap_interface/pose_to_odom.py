'''
This node subscribes to the /pose topic, calculates odometry and publishes the odometry to /odometry topic.
This node should be used in conjunction with the qualisys_pubsub node and should be namespaced to the robot name.

'''


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import numpy as np

class PoseToOdom(Node):
    def __init__(self):
        super().__init__('pose_to_odom')
        self.publisher_ = self.create_publisher(Odometry, 'odometry', 10)
        self.subscription = self.create_subscription(
            PoseStamped,
            'pose',
            self.listener_callback,
            10)
        self.subscription
        self.prev_msg = None

    def listener_callback(self, msg):
        if self.prev_msg is None:
            self.prev_msg = msg
            return
        
        # check if the position is a NaN position
        if np.isnan([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]).any():
            self.get_logger().warn('Received NaN position, skipping this message')
            return

        # check if the quaternion is a NaN quaternion
        if np.isnan([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]).any():
            self.get_logger().warn('Received NaN quaternion, skipping this message')
            return
        
        # Check if the quaternion is a zero quaternion
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        if np.allclose(quat, [0, 0, 0, 0], atol=1e-3): 
            self.get_logger().warn('Received zero quaternion, skipping this message')
            return

        odometry = Odometry()
        dt = 0.1  # Time difference is 1/10 because the messages are published at 10 Hz (qualisys system frequency)

        # Set the header
        odometry.header = msg.header

        # Set the child frame id
        odometry.child_frame_id = 'base_link' 

        # Set the pose
        odometry.pose.pose = msg.pose

        # Calculate and set the linear velocity
        odometry.twist.twist.linear.x = (msg.pose.position.x - self.prev_msg.pose.position.x) / dt
        odometry.twist.twist.linear.y = (msg.pose.position.y - self.prev_msg.pose.position.y) / dt
        odometry.twist.twist.linear.z = (msg.pose.position.z - self.prev_msg.pose.position.z) / dt

        # Calculate and set the angular velocity
        angular_velocity = self.calculate_angular_velocity(
            [self.prev_msg.pose.orientation.x, self.prev_msg.pose.orientation.y, self.prev_msg.pose.orientation.z, self.prev_msg.pose.orientation.w],
            [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w],
            dt)

        odometry.twist.twist.angular.x = angular_velocity[0]
        odometry.twist.twist.angular.y = angular_velocity[1]
        odometry.twist.twist.angular.z = angular_velocity[2]

        self.publisher_.publish(odometry)
        self.prev_msg = msg

    def calculate_angular_velocity(self, q1, q2, dt):
        r1 = R.from_quat(q1)
        r2 = R.from_quat(q2)
        relative_rotation = r2 * r1.inv()
        rotvec = relative_rotation.as_rotvec()
        angle = np.linalg.norm(rotvec)
        axis = rotvec / angle
        angular_velocity = axis * angle / dt
        return angular_velocity

def main(args=None):
    rclpy.init(args=args)
    pose_to_odom = PoseToOdom()
    rclpy.spin(pose_to_odom)
    pose_to_odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
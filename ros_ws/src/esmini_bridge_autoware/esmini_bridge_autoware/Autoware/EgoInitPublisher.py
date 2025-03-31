import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


class EgoInitPublisher(Node):
    def __init__(self):
        super().__init__("egoinit_publisher")
        self.logger = self.get_logger()
        self._init_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._goal_publisher = self.create_publisher(
            PoseStamped, "/planning/mission_planning/goal", 10
        )

    def publish_initial_pose(self, x, y):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        self._init_publisher.publish(msg)

        self.publish_goal_pose()

    def publish_goal_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        # .456665, y: -92.682770, z: 0.000000
        # qx: 0.000000, qy: 0.000000, qz: -0.712069, qw: 0.702110
        msg.pose.position.x = 267.456665
        msg.pose.position.y = -92.682770
        msg.pose.orientation.z = -0.712069
        msg.pose.orientation.w = 0.702110
        self.logger.info(f"Publish goal {msg}")
        self._goal_publisher.publish(msg)

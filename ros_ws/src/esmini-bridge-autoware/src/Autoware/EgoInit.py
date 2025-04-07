import time
import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from autoware_adapi_v1_msgs.srv import ChangeOperationMode


class EgoInit(Node):
    def __init__(self):
        super().__init__("egoinit")
        self.logger = self.get_logger()
        self._init_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._goal_publisher = self.create_publisher(
            PoseStamped, "/planning/mission_planning/goal", 10
        )
        self._switch_mode_client = self.create_client(
            ChangeOperationMode,
            "/api/operation_mode/change_to_autonomous",
        )

    def publish_initial_pose(self, x, y, h):
        self.logger.info("heading: " + str(h))
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        quat = quaternion_from_euler(0, 0, h)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        self._init_publisher.publish(msg)

    def publish_goal_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = 267.456665
        msg.pose.position.y = -92.682770
        msg.pose.orientation.z = -0.712069
        msg.pose.orientation.w = 0.702110
        self.logger.info(f"Publish goal {msg}")
        self._goal_publisher.publish(msg)

    def switch_auto_mode(self):
        self.logger.info(f"Wait Autoware a second to plan a route")
        time.sleep(1)
        done = False
        while not done:
            while not self._switch_mode_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("service not available, waiting again...")
            req = ChangeOperationMode.Request()
            future = self._switch_mode_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.logger.debug(
                f"Calling {self._switch_mode_client.srv_name} got result {future.result()}"
            )
            if future.result().status.success:
                self.logger.info("Switch mode success")
                done = True
            else:
                self.logger.info(
                    f"Switch mode failed with msg: {future.result().status.message}, retrying..."
                )
                time.sleep(1)

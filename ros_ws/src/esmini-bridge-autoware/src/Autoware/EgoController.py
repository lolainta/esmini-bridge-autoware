import rclpy
from rclpy.node import Node

from autoware_control_msgs.msg import Control


class EgoController(Node):
    def __init__(self):
        super().__init__("egocontroller")
        self.logger = self.get_logger()
        self._controll_command_susbscriber = self.create_subscription(
            Control,
            "/control/command/control_cmd",
            self._subscriber_callback,
            10,
        )
        self.velocity = 0

    def _subscriber_callback(self, msg):
        self.velocity = msg.longitudinal.velocity

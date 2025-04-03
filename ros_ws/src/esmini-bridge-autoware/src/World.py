import ctypes

import rclpy
from rclpy.node import Node

from .Autoware.EgoInit import EgoInit
from .Autoware.EgoController import EgoController
from .SE.ObjectState import SEScenarioObjectState


class World(Node):
    def __init__(self) -> None:
        super().__init__("world")

        self.se = ctypes.CDLL("/esmini/bin/libesminiLib.so")
        xosc = "/esmini/scripts/udp_driver/one_car_on_road.xosc"
        self.logger = self.get_logger()

        self.logger.info(f"Loading {xosc}")

        self.se.SE_Init(xosc.encode("ascii"), 1, 1, 0, 0)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.ego_controller = EgoController()

        self.start_autoware()

    def timer_callback(self):
        if self.se.SE_GetQuitFlag() != 0:
            self.logger.info("Quit flag detected")
            exit(0)

        self.se.SE_ReportObjectSpeed.argtypes = [ctypes.c_int, ctypes.c_float]

        assert (
            self.se.SE_ReportObjectSpeed(
                self.se.SE_GetId(0),
                float(self.ego_controller.velocity),
            )
            == 0
        ), "SE_ReportObjectSpeed"

        self.se.SE_Step()

    def start_autoware(self):
        ego_init = EgoInit()
        self.logger.info("Initializing Autoware")
        obj_state = SEScenarioObjectState()
        self.se.SE_GetObjectState(self.se.SE_GetId(0), ctypes.byref(obj_state))

        ego_init.publish_initial_pose(obj_state.x, obj_state.y)
        ego_init.publish_goal_pose()  # hard coded destination
        ego_init.switch_auto_mode()

        self.logger.info("Success: Autoware start driving")

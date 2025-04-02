import ctypes

from rclpy.node import Node

from .Autoware.EgoInit import EgoInit
from .SE.ObjectState import SEScenarioObjectState


class World(Node):
    def __init__(self) -> None:
        super().__init__("world")

        self.se = ctypes.CDLL("/esmini/bin/libesminiLib.so")
        xosc = "/esmini/scripts/udp_driver/one_car_on_road.xosc"
        self.logger = self.get_logger()

        self.logger.info(f"Loading {xosc}")

        self.se.SE_Init(xosc.encode("ascii"), 0, 1, 0, 0)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.ego_init = EgoInit()

        self.start_autoware()

    def timer_callback(self):
        if self.se.SE_GetQuitFlag() != 0:
            self.logger.info("Quit flag detected")
            exit(0)
        self.se.SE_Step()

    def start_autoware(self):
        self.logger.info("Initializing Autoware")
        obj_state = SEScenarioObjectState()
        self.se.SE_GetObjectState(self.se.SE_GetId(0), ctypes.byref(obj_state))

        print(
            f"id: {obj_state.id} (x,y,z) = ({obj_state.x}, {obj_state.y}, {obj_state.z})"
        )
        self.ego_init.publish_initial_pose(obj_state.x, obj_state.y)
        self.ego_init.publish_goal_pose()  # hard coded destination
        self.ego_init.switch_auto_mode()

        self.logger.info("Success: Autoware start driving")

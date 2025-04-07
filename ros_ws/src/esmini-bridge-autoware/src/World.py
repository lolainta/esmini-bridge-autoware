import ctypes

import rclpy
from rclpy.node import Node

from .Autoware.EgoInit import EgoInit
from .Autoware.EgoController import EgoController
from .SE.ObjectState import SEScenarioObjectState
from .SE.SimpleVehicleState import SESimpleVehicleState


class World(Node):
    def __init__(self) -> None:
        super().__init__("world")
        self.logger = self.get_logger()

        self.se = ctypes.CDLL("/esmini/bin/libesminiLib.so")
        self._set_se_proto()

        self.rm = ctypes.CDLL("/esmini/bin/libesminiRMLib.so")

        xosc = "/esmini/scripts/udp_driver/one_car_on_road.xosc"
        xosc = "/esmini/resources/xosc/cut-in.xosc"
        # xosc = "/esmini/resources/xosc/lane_change_simple.xosc"

        self.logger.info(f"Loading {xosc}")
        self.se.SE_Init(xosc.encode("ascii"), 1, 1, 0, 0)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.ego_controller = EgoController()

        # self.se.SE_SetLockOnLane(0, False)
        # self.rm.RM_SetAlignModeH(0, 0)

        self.start_autoware()

        ego_state = SEScenarioObjectState()
        self.se.SE_GetObjectState(0, ctypes.byref(ego_state))
        self.logger.info(f"Got ego state: {ego_state.x=}, {ego_state.y=}")
        self.ego_handle = self.se.SE_SimpleVehicleCreate(
            ego_state.x,
            ego_state.y,
            ego_state.h,
            ego_state.length,
            ego_state.speed,
        )
        self.se.SE_SimpleVehicleSetThrottleDisabled(self.ego_handle, False)

    def _set_se_proto(self):
        self.se.SE_ReportObjectSpeed.argtypes = [ctypes.c_int, ctypes.c_float]
        self.se.SE_ReportObjectWheelStatus.argtypes = [
            ctypes.c_int,  # object_id
            ctypes.c_float,  # rotation
            ctypes.c_float,  # angle
        ]
        self.se.SE_ReportObjectAngularAcc.argtypes = [
            ctypes.c_int,  # object_id
            ctypes.c_float,  # timestamp
            ctypes.c_float,  # h_acc
            ctypes.c_float,  # p_acc
            ctypes.c_float,  # r_acc
        ]
        self.se.SE_ReportObjectPosXYH.argtypes = [
            ctypes.c_int,  # object_id
            ctypes.c_float,  # timestamp
            ctypes.c_float,  # x
            ctypes.c_float,  # y
            ctypes.c_float,  # h
        ]
        self.se.SE_SimpleVehicleCreate.argtypes = [
            ctypes.c_float,  # x
            ctypes.c_float,  # y
            ctypes.c_float,  # h
            ctypes.c_float,  # length
            ctypes.c_float,  # speed
        ]
        self.se.SE_SimpleVehicleCreate.restype = ctypes.POINTER(SESimpleVehicleState)
        self.se.SE_SimpleVehicleSteeringRate.argtypes = [
            ctypes.POINTER(SESimpleVehicleState),
            ctypes.c_float,
        ]
        self.se.SE_SimpleVehicleControlAnalog.argtypes = [
            ctypes.POINTER(SESimpleVehicleState),
            ctypes.c_float,  # dt
            ctypes.c_float,  # throttle
            ctypes.c_float,  # steering
        ]
        self.se.SE_SimpleVehicleSetSpeed.argtypes = [
            ctypes.POINTER(SESimpleVehicleState),
            ctypes.c_float,
        ]

    def timer_callback(self):
        if self.se.SE_GetQuitFlag() != 0:
            self.logger.info("Quit flag detected")
            self.destroy_node()
            exit(0)

        # throttle_weight = 0.1
        # throttle = throttle_weight * (self.ego_controller.velocity - ego_state.speed)
        # self.logger.info(f"{throttle=}")

        self.se.SE_SimpleVehicleSetSpeed(self.ego_handle, self.ego_controller.velocity)

        self.se.SE_SimpleVehicleControlAnalog(
            self.ego_handle, 1, 0, self.ego_controller.angle
        )

        vehicle_state = SESimpleVehicleState()
        self.se.SE_SimpleVehicleGetState(self.ego_handle, ctypes.byref(vehicle_state))
        # self.logger.info(
        #     f"Got vehicle_state: {vehicle_state.x=:.3f}, {vehicle_state.y=:.3f}, {vehicle_state.speed=:.3f}"
        # )

        self.se.SE_ReportObjectPosXYH(
            0, 0, vehicle_state.x, vehicle_state.y, vehicle_state.h
        )
        # self.se.SE_ReportObjectSpeed(0, self.ego_controller.velocity)
        self.se.SE_ReportObjectWheelStatus(
            0, vehicle_state.wheel_rotation, vehicle_state.wheel_angle
        )

        self.se.SE_Step()

    def start_autoware(self):
        ego_init = EgoInit()
        self.logger.info("Initializing Autoware")
        obj_state = SEScenarioObjectState()
        self.se.SE_GetObjectState(0, ctypes.byref(obj_state))

        ego_init.publish_initial_pose(obj_state.x, obj_state.y, obj_state.h)
        # ego_init.publish_goal_pose()  # hard coded destination
        # ego_init.switch_auto_mode()

        self.logger.info("Success: Autoware start driving")

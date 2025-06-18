#include "cpp_esmini_bridge_autoware/World.hpp"
#include "cpp_esmini_bridge_autoware/AutowareHandler.hpp"
#include "esminiLib.hpp"
#include <chrono>
using namespace std::chrono_literals;

World::World() : Node("World") {
    RCLCPP_INFO(this->get_logger(), "World Node Initialized");
    this->esmini_init();
    RCLCPP_INFO(this->get_logger(), "Esmini Initialized");
    SE_ScenarioObjectState ego_state;
    SE_GetObjectState(0, &ego_state);
    this->ego = std::make_shared<AutowareHandler>(
        ego_state.x, ego_state.y, ego_state.h, 6.5, 299.6, 1.57);
    // this->ego = std::make_shared<AutowareHandler>(
    //     ego_state.x, ego_state.y, ego_state.h, 52.5, 11.6, 6.28);
    timer_ =
        this->create_wall_timer(10ms, std::bind(&World::timer_callback, this));
}

void World::esmini_init() {
    // SE_Init("/esmini/resources/xosc/cut-in.xosc", 0, 1, 0, 0);
    // SE_Init("/resources/xosc/chengyu/SinD_test1.xosc", 1, 1, 0, 0);
    SE_AddPath("/esmini/resources/xosc/");
    SE_Init("/resources/xosc/yusheng/148.xosc", 1, 1, 0, 1);
    // SE_Init("/resources/xosc/chengyu/1.xosc", 1, 1, 0, 1);
    SE_CollisionDetection(true);
    SE_GetObjectState(0, &objectState);
    vehicleHandle =
        SE_SimpleVehicleCreate(objectState.x, objectState.y, objectState.h,
                               objectState.length, objectState.speed);
    SE_SimpleVehicleSteeringRate(vehicleHandle, 9.0f);
    SE_SimpleVehicleSetThrottleDisabled(vehicleHandle, true);
    // SE_ViewerShowFeature(4 + 8, true);
}

void World::timer_callback() {
    float dt = SE_GetSimTimeStep();
    SE_SimpleVehicleControlAnalog(vehicleHandle, dt, 0,
                                  this->ego->get_rotation());
    SE_SimpleVehicleSetSpeed(vehicleHandle, this->ego->get_velocity());
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Throttle: %f, Rotation: %f",
                         this->ego->get_velocity(), this->ego->get_rotation());
    SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Number of objects: %d", SE_GetNumberOfObjects());
    for (int i = 1; i < SE_GetNumberOfObjects(); i++) {
        SE_ScenarioObjectState objectState;
        SE_GetObjectState(SE_GetId(i), &objectState);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Object %d: %f, %f, %f, Speed: %f", objectState.id,
                             objectState.x, objectState.y, objectState.h,
                             objectState.speed);
        this->ego->set_object(objectState.id, objectState.x, objectState.y,
                              objectState.h, objectState.speed);
    }
    SE_ReportObjectPosXYH(0, 0, vehicleState.x, vehicleState.y, vehicleState.h);
    SE_ReportObjectWheelStatus(0, vehicleState.wheel_rotation,
                               vehicleState.wheel_angle);
    SE_ReportObjectSpeed(0, vehicleState.speed);
    SE_StepDT(dt);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Vehicle State: %f, %f, %f", vehicleState.x,
                         vehicleState.y, vehicleState.h);
    this->ego->set_ego_state(vehicleState.x, vehicleState.y, vehicleState.h);
}

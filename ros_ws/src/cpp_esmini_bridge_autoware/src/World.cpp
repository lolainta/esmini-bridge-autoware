#include "cpp_esmini_bridge_autoware/World.hpp"
#include "cpp_esmini_bridge_autoware/AutowareHandler.hpp"
#include "esminiLib.hpp"
#include <chrono>
using namespace std::chrono_literals;

World::World() : Node("EgoCreateNode") {
    timer_ =
        this->create_wall_timer(10ms, std::bind(&World::timer_callback, this));

    this->esmini_init();

    SE_ScenarioObjectState ego_state;
    SE_GetObjectState(0, &ego_state);
    this->ego = std::make_unique<AutowareHandler>(ego_state.x, ego_state.y,
                                                  ego_state.h);
}

void World::esmini_init() {
    SE_Init("/esmini/resources/xosc/cut-in.xosc", 0, 1, 0, 0);
    SE_GetObjectState(0, &objectState);
    vehicleHandle = SE_SimpleVehicleCreate(objectState.x, objectState.y,
                                           objectState.h, 4.0, 0.0);
    // SE_SimpleVehicleSteeringRate(vehicleHandle, 6.0f);
    SE_ViewerShowFeature(4 + 8, true);
}

void World::timer_callback() {
    float dt = SE_GetSimTimeStep();
    SE_SimpleVehicleControlAnalog(vehicleHandle, dt, 0.1, 1);
    SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
    SE_ReportObjectPosXYH(0, 0, vehicleState.x, vehicleState.y, vehicleState.h);
    SE_ReportObjectWheelStatus(0, vehicleState.wheel_rotation,
                               vehicleState.wheel_angle);
    SE_ReportObjectSpeed(0, vehicleState.speed);
    SE_StepDT(dt);
}

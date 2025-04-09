#pragma once

#include "cpp_esmini_bridge_autoware/AutowareHandler.hpp"
#include "esminiLib.hpp"
#include "rclcpp/rclcpp.hpp"

class World : public rclcpp::Node {
  public:
    World();

  private:
    rclcpp::TimerBase::SharedPtr timer_;

    void *vehicleHandle = 0;
    SE_SimpleVehicleState vehicleState = {0, 0, 0, 0, 0, 0, 0, 0};
    SE_ScenarioObjectState objectState;
    std::unique_ptr<AutowareHandler> ego;

    void esmini_init();
    void timer_callback();
};

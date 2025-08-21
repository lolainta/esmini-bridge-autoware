#pragma once

#include "cpp_esmini_bridge_autoware/AutowareHandler.hpp"
#include "cpp_esmini_bridge_autoware/EgoState.hpp"
#include "esminiLib.hpp"
#include "rclcpp/rclcpp.hpp"

class World : public rclcpp::Node {
  public:
    World();

    std::shared_ptr<AutowareHandler> get_ego() { return ego; }

  private:
    std::shared_ptr<AutowareHandler> ego;
    EgoState ego_state;

    rclcpp::TimerBase::SharedPtr timer_;

    void *vehicleHandle = 0;
    SE_SimpleVehicleState vehicleState = {0, 0, 0, 0, 0, 0, 0, 0};
    SE_ScenarioObjectState objectState;

    void esmini_init();
    void timer_callback();
};

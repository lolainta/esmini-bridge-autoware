#pragma once

#include "cpp_esmini_bridge_autoware/AutowareHandler.hpp"
#include "cpp_esmini_bridge_autoware/EgoState.hpp"
#include "cpp_esmini_bridge_autoware/RateLimiter.hpp"
#include "esminiLib.hpp"
#include "rclcpp/rclcpp.hpp"

typedef enum class WorldState {
    AV_CONNECTING,
    WAITING_FOR_PLANNING,
    ENGAGING,
    RUNNING,
    STOPPED,
} WorldState;

class World : public rclcpp::Node {
  public:
    World();

    std::shared_ptr<AutowareHandler> get_ego() { return ego; }

  private:
    std::shared_ptr<AutowareHandler> ego;
    WorldState world_state;

    rclcpp::TimerBase::SharedPtr timer_;
    RateLimiter limiter;

    void *vehicleHandle = 0;
    SE_SimpleVehicleState vehicleState = {0, 0, 0, 0, 0, 0, 0, 0};
    SE_ScenarioObjectState objectState;

    void esmini_opts();
    void esmini_init();
    void esmini_close();
    void timer_callback();

    void set_ego_route();
    void tick();
};

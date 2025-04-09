#include "esminiLib.hpp"
#include "rclcpp/rclcpp.hpp"

class World : public rclcpp::Node {
  public:
    World();

  private:
    void *vehicleHandle = 0;
    SE_SimpleVehicleState vehicleState = {0, 0, 0, 0, 0, 0, 0, 0};
    SE_ScenarioObjectState objectState;

    void esmini_init();
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
};

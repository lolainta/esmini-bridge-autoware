#pragma once

#include "autoware_adapi_v1_msgs/srv/change_operation_mode.hpp"
#include "esminiLib.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class AutowareHandler : public rclcpp::Node {
  public:
    AutowareHandler(float x, float y, float h);

  private:
    SE_ScenarioObjectState ego_state;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        initialpose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        goalpose_publisher_;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr
        engage_autoware_client_;

    void publish_initialpose_(float x, float y, float h);
    void publish_goalpose_(float x, float y, float h);
    void engage_autoware_();
};

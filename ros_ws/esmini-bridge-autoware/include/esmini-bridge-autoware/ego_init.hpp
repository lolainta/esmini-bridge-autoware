#pragma once

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using geometry_msgs::msg::PoseWithCovarianceStamped;

class EgoInitNode : public rclcpp::Node {
  public:
    EgoInitNode(const rclcpp::NodeOptions &);

  private:
    rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_initialpose_;
    void run();
};

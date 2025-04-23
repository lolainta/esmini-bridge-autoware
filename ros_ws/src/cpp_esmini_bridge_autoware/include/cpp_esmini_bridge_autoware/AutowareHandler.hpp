#pragma once

#include "autoware_adapi_v1_msgs/srv/change_operation_mode.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "esminiLib.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class AutowareHandler : public rclcpp::Node {
  public:
    AutowareHandler(float x, float y, float h);

    float get_velocity() const { return velocity; }
    float get_rotation() const { return rotation; }

  private:
    SE_ScenarioObjectState ego_state;
    sensor_msgs::msg::Imu imu_state;

    float velocity = 0.0;
    float rotation = 0.0;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        initialpose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        goalpose_publisher_;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr
        engage_autoware_client_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
        pub_steering_status_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
        pub_velocity_status_;
    rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr
        pub_accel_;

    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr
        control_command_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

    void set_imu_state_(geometry_msgs::msg::Vector3,
                        geometry_msgs::msg::Vector3);

    void publish_initialpose_(float x, float y, float h);
    void publish_goalpose_(float x, float y, float h);
    void publish_steering_();
    void publish_velocity_();
    void publish_accel_();

    void engage_autoware_();

    void control_command_callback_(
        const autoware_control_msgs::msg::Control::SharedPtr msg);
    void timer_callback();
};

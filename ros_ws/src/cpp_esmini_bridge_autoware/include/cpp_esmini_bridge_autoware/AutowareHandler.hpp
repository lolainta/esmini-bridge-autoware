#pragma once
#include <unordered_map>

#include "autoware_adapi_v1_msgs/srv/change_operation_mode.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "autoware_perception_msgs/msg/object_classification.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/srv/control_mode_command.hpp"
#include "esminiLib.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

typedef struct EgoState {
    float x = 0;
    float y = 0;
    float h = 0;
    float time = 0;
} EgoState;

class AutowareHandler : public rclcpp::Node {
  public:
    AutowareHandler(float, float, float, float, float, float);

    float get_velocity() const { return velocity; }
    float get_rotation() const { return rotation; }
    void set_ego_state(float, float, float);
    void set_object(int, float, float, float, float);

  private:
    float init_x, init_y, init_h;
    float goal_x, goal_y, goal_h;

    EgoState ego_state;
    EgoState prev_ego_state;
    EgoState prev_ego_state2;
    sensor_msgs::msg::Imu imu_state;

    std::unordered_map<int, autoware_perception_msgs::msg::DetectedObject>
        detected_objects_map;
    std::unordered_map<int, autoware_perception_msgs::msg::PredictedObject>
        predicted_objects_map;

    autoware_perception_msgs::msg::PredictedObjects predicted_objects;

    float velocity = 0.0;
    float rotation = 0.0;
    bool engaged = false;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        initialpose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        goalpose_publisher_;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr
        engage_autoware_client_;

    rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr
        pub_control_mode_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr
        pub_gear_report_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
        pub_steering_status_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
        pub_velocity_status_;

    rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr
        pub_accel_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_state_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_kinematic_state_;

    rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr
        pub_detected_objects_;
    rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::
        SharedPtr pub_predicted_objects_;

    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr
        control_command_subscriber_;

    rclcpp::Service<autoware_vehicle_msgs::srv::ControlModeCommand>::SharedPtr
        srv_control_mode_command_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_publish_initialpose_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_publish_goalpose_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr engage_timer_;

    void calc_imu_state_();

    void publish_initialpose_(float, float, float);
    void publish_goalpose_(float, float, float);
    void publish_initialpose_callback_(
        const std_srvs::srv::Trigger::Request::ConstSharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr);
    void publish_goalpose_callback_(
        const std_srvs::srv::Trigger::Request::ConstSharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr);
    void engage_autoware_();

    void publish_control_mode_();
    void publish_gear_report_();
    void publish_steering_();
    void publish_velocity_();

    void publish_accel_();
    void publish_tf_();
    void publish_pose_();
    void publish_imu_state_();
    void publish_kinematic_state_();

    void publish_objects_();

    void control_command_callback_(
        const autoware_control_msgs::msg::Control::SharedPtr msg);

    // void control_mode_command_callback_(
    //     const autoware_vehicle_msgs::srv::ControlModeCommand::Request::
    //         SharedPtr,
    //     autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr);
    void control_mode_command_callback_(
        const autoware_vehicle_msgs::srv::ControlModeCommand::Request::
            ConstSharedPtr,
        autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr);
    void timer_callback();
};

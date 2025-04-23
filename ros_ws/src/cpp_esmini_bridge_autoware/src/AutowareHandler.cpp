#include "cpp_esmini_bridge_autoware/AutowareHandler.hpp"
#include <chrono>

using namespace std::chrono_literals;

AutowareHandler::AutowareHandler(float x, float y, float h)
    : Node("AutowareHandler") {
    this->initialpose_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);
    this->goalpose_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/planning/mission_planning/goal", 10);
    this->engage_autoware_client_ =
        this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
            "/api/operation_mode/change_to_autonomous");
    this->pub_steering_status_ =
        this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
            "/vehicle/status/steering_status", 10);
    this->pub_velocity_status_ =
        this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", 10);
    this->pub_tf_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    this->pub_accel_ =
        this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
            "/localization/acceleration", 10);
    this->pub_kinematic_state_ =
        this->create_publisher<nav_msgs::msg::Odometry>(
            "/localization/kinematic_state", 10);

    this->pub_predicted_objects_ =
        this->create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
            "/perception/object_recognition/objects", 10);

    this->control_command_subscriber_ =
        this->create_subscription<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd", 10,
            std::bind(&AutowareHandler::control_command_callback_, this,
                      std::placeholders::_1));

    this->timer_ = this->create_wall_timer(
        10ms, std::bind(&AutowareHandler::timer_callback, this));

    this->publish_initialpose_(x, y, h);
    this->publish_goalpose_(6.5, 299.6, 1.57);

    this->engage_timer_ = this->create_wall_timer(
        1s, std::bind(&AutowareHandler::engage_autoware_, this));
}

void AutowareHandler::set_ego_state(float x, float y, float h) {
    this->prev_ego_state2 = this->prev_ego_state;
    this->prev_ego_state = this->ego_state;
    this->ego_state.x = x;
    this->ego_state.y = y;
    this->ego_state.h = h;
    this->ego_state.time = this->now().seconds();
}

void AutowareHandler::calc_imu_state_() {
    float dt1 = this->now().seconds() - this->prev_ego_state.time;
    float dt2 = this->prev_ego_state.time - this->prev_ego_state2.time;
    float dt = (dt1 + dt2) / 2.0;

    geometry_msgs::msg::Vector3 cur_v;
    cur_v.x = (this->ego_state.x - this->prev_ego_state.x) / dt1;
    cur_v.y = (this->ego_state.y - this->prev_ego_state.y) / dt1;
    cur_v.z = 0.0;
    geometry_msgs::msg::Vector3 prev_v;
    prev_v.x = (this->prev_ego_state.x - this->prev_ego_state2.x) / dt2;
    prev_v.y = (this->prev_ego_state.y - this->prev_ego_state2.y) / dt2;
    prev_v.z = 0.0;
    geometry_msgs::msg::Vector3 linear_acceleration;
    linear_acceleration.x = (cur_v.x - prev_v.x) / dt;
    linear_acceleration.y = (cur_v.y - prev_v.y) / dt;
    linear_acceleration.z = 0.0;

    geometry_msgs::msg::Vector3 angular_velocity;
    angular_velocity.x = 0.0;
    angular_velocity.y = 0.0;
    angular_velocity.z = (this->ego_state.h - this->prev_ego_state.h) / dt;
    this->imu_state.linear_acceleration = linear_acceleration;
    this->imu_state.angular_velocity = angular_velocity;
    this->imu_state.header.stamp = this->now();
    this->imu_state.header.frame_id = "base_link";
}

void AutowareHandler::publish_initialpose_(float x, float y, float h) {
    geometry_msgs::msg::PoseWithCovarianceStamped initialpose;
    initialpose.header.frame_id = "map";
    initialpose.header.stamp = this->now();
    initialpose.pose.pose.position.x = x;
    initialpose.pose.pose.position.y = y;
    initialpose.pose.pose.position.z = 0.0;
    initialpose.pose.pose.orientation.x = 0.0;
    initialpose.pose.pose.orientation.y = 0.0;
    initialpose.pose.pose.orientation.z = sin(h / 2);
    initialpose.pose.pose.orientation.w = cos(h / 2);
    this->initialpose_publisher_->publish(initialpose);
}

void AutowareHandler::publish_goalpose_(float x, float y, float h) {
    geometry_msgs::msg::PoseStamped goalpose;
    goalpose.header.frame_id = "map";
    goalpose.header.stamp = this->now();
    goalpose.pose.position.x = x;
    goalpose.pose.position.y = y;
    goalpose.pose.position.z = 0.0;
    goalpose.pose.orientation.x = 0.0;
    goalpose.pose.orientation.y = 0.0;
    goalpose.pose.orientation.z = sin(h / 2);
    goalpose.pose.orientation.w = cos(h / 2);
    this->goalpose_publisher_->publish(goalpose);
}

void AutowareHandler::publish_steering_() {
    autoware_vehicle_msgs::msg::SteeringReport steering_report;
    steering_report.stamp = this->now();
    steering_report.steering_tire_angle = this->rotation;
    this->pub_steering_status_->publish(steering_report);
}

void AutowareHandler::publish_velocity_() {
    autoware_vehicle_msgs::msg::VelocityReport velocity_report;
    velocity_report.header.stamp = this->now();
    velocity_report.header.frame_id = "base_link";
    velocity_report.longitudinal_velocity = this->velocity;
    this->pub_velocity_status_->publish(velocity_report);
}

void AutowareHandler::publish_accel_() {
    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    accel.header.stamp = this->now();
    accel.header.frame_id = "base_link";
    accel.accel.accel.linear = this->imu_state.linear_acceleration;
    accel.accel.accel.angular = this->imu_state.angular_velocity;
    this->pub_accel_->publish(accel);
}

void AutowareHandler::publish_tf_() {
    tf2_msgs::msg::TFMessage tf;
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = this->ego_state.x;
    transform.transform.translation.y = this->ego_state.y;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = sin(this->ego_state.h / 2);
    transform.transform.rotation.w = cos(this->ego_state.h / 2);
    tf.transforms.push_back(transform);
    this->pub_tf_->publish(tf);
}

void AutowareHandler::publish_kinematic_state_() {
    nav_msgs::msg::Odometry kinematic_state;
    kinematic_state.header.stamp = this->now();
    kinematic_state.header.frame_id = "map";
    kinematic_state.child_frame_id = "base_link";
    kinematic_state.pose.pose.position.x = this->ego_state.x;
    kinematic_state.pose.pose.position.y = this->ego_state.y;
    kinematic_state.pose.pose.position.z = 0.0;
    kinematic_state.pose.pose.orientation.x = 0.0;
    kinematic_state.pose.pose.orientation.y = 0.0;
    kinematic_state.pose.pose.orientation.z = sin(this->ego_state.h / 2);
    kinematic_state.pose.pose.orientation.w = cos(this->ego_state.h / 2);
    kinematic_state.twist.twist.linear.x = this->velocity;
    kinematic_state.twist.twist.angular.z = this->rotation;
    this->pub_kinematic_state_->publish(kinematic_state);
}

void AutowareHandler::engage_autoware_() {
    if (this->engaged) {
        return;
    }
    while (!this->engage_autoware_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(),
                    "Waiting for the service engage_autoware...");
    }
    auto request = std::make_shared<
        autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
    auto future = this->engage_autoware_client_->async_send_request(
        request,
        [this](
            rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::
                SharedFuture result) {
            auto response = result.get();
            if (response->status.success) {
                RCLCPP_INFO(this->get_logger(),
                            "Engaged Autoware successfully");
                this->engaged = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed: %s",
                             response->status.message.c_str());
            }
        });
}

void AutowareHandler::control_command_callback_(
    const autoware_control_msgs::msg::Control::SharedPtr msg) {
    this->rotation = msg->lateral.steering_tire_angle;
    this->velocity = msg->longitudinal.velocity;
}

void AutowareHandler::timer_callback() {
    this->publish_steering_();
    this->publish_velocity_();
    this->calc_imu_state_();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Velocity: %f, Rotation: %f", this->velocity,
                         this->rotation);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Ego State: %f, %f, %f", this->ego_state.x,
                         this->ego_state.y, this->ego_state.h);
    this->publish_accel_();
    this->publish_tf_();
    // this->publish_pose_();
    this->publish_kinematic_state_();
}

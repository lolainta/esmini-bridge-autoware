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

    this->control_command_subscriber_ =
        this->create_subscription<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd", 10,
            std::bind(&AutowareHandler::control_command_callback_, this,
                      std::placeholders::_1));
    this->pub_accel_ =
        this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
            "/localization/acceleration", 10);

    this->timer_ = this->create_wall_timer(
        25ms, std::bind(&AutowareHandler::timer_callback, this));

    this->publish_initialpose_(x, y, h);
    this->publish_goalpose_(10.2, 299.6, 1.57);
    this->engage_autoware_();
}

void AutowareHandler::set_imu_state_(
    geometry_msgs::msg::Vector3 linear_accel,
    geometry_msgs::msg::Vector3 angular_velocity) {
    this->imu_state.linear_acceleration = linear_accel;
    this->imu_state.angular_velocity = angular_velocity;
    this->imu_state.header.stamp = this->now();
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

void AutowareHandler::engage_autoware_() {
    while (!this->engage_autoware_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(),
                    "Waiting for the service engage_autoware...");
    }
    while (rclcpp::ok()) {
        auto request = std::make_shared<
            autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
        auto future =
            this->engage_autoware_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                               future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to call service engage_autoware");
            return;
        }
        auto response = future.get();
        if (response->status.success) {
            RCLCPP_INFO(this->get_logger(), "Engaged Autoware successfully");
            break;

        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to engage Autoware %s",
                         response->status.message.c_str());
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void AutowareHandler::control_command_callback_(
    const autoware_control_msgs::msg::Control::SharedPtr msg) {
    this->rotation = msg->lateral.steering_tire_angle;
    this->velocity = msg->longitudinal.velocity;
}

void AutowareHandler::timer_callback() {
    this->publish_steering_();
    this->publish_velocity_();
    this->publish_accel_();
}

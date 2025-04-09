#include "cpp_esmini_bridge_autoware/AutowareHandler.hpp"

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

    this->publish_initialpose_(x, y, h);
    this->publish_goalpose_(6.3, 273, 1.57);
    this->engage_autoware_();
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

void AutowareHandler::engage_autoware_() {
    auto request = std::make_shared<
        autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
    auto result = this->engage_autoware_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to call service engage_autoware");
    }
}

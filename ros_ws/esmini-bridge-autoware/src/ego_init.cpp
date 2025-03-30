#include "ego_init.hpp"

EgoInitNode::EgoInitNode(const rclcpp::NodeOptions &options)
    : Node("EgoInitNode", options) {
    RCLCPP_INFO(this->get_logger(), "EgoInitNode Constructed");
    this->pub_initialpose_ =
        this->create_publisher<PoseWithCovarianceStamped>("/initialpose", 10);

    // sleep 1
    std::this_thread::sleep_for(std::chrono::seconds(1));
    this->run();
}

void EgoInitNode::run() {
    auto msg = PoseWithCovarianceStamped();
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "map";
    // msg.header.stamp = this->now();
    msg.pose.pose.position.x = 51.0;
    msg.pose.pose.position.y = -1;
    msg.pose.pose.position.z = 0;
    msg.pose.pose.orientation.x = 0;
    msg.pose.pose.orientation.y = 0;
    msg.pose.pose.orientation.z = 0;
    msg.pose.pose.orientation.w = 1;
    this->pub_initialpose_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "EgoInitNode Published");
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(EgoInitNode)

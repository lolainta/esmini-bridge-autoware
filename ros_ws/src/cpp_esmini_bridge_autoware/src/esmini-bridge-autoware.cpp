#include <cstdio>

#include <cpp_esmini_bridge_autoware/World.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto world = std::make_shared<World>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(world);
    executor.add_node(world->get_ego());
    try {
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(world->get_logger(), "Exception: %s", e.what());
    }
    rclcpp::shutdown();

    return 0;
}

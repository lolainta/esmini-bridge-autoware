#include <cstdio>

#include <cpp_esmini_bridge_autoware/World.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<World>());
    rclcpp::shutdown();
    return 0;
}

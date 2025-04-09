#include <cstdio>

#include <cpp_esmini_bridge_autoware/world.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<World>());
    rclcpp::shutdown();
    return 0;
}

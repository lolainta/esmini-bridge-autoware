cmake_minimum_required(VERSION 3.8)
project(esmini-bridge-autoware)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


include_directories(
  include/esmini-bridge-autoware/
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)


add_library(ego_init_component SHARED src/ego_init.cpp)

ament_target_dependencies(ego_init_component
  rclcpp
  rclcpp_components
  geometry_msgs
)

rclcpp_components_register_node(
  ego_init_component
  PLUGIN "EgoInitNode"
  EXECUTABLE ego_init
)

ament_export_targets(export_ego_init_component)
install(
  TARGETS ego_init_component
  EXPORT export_ego_init_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# add_executable(esmini-bridge-autoware src/esmini-bridge-autoware.cpp)
# target_include_directories(esmini-bridge-autoware PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(esmini-bridge-autoware PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS esmini-bridge-autoware
#   DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()

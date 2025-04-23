cmake_minimum_required(VERSION 3.8)
project(cpp_esmini_bridge_autoware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ESMINI_INCLUDE_DIR /esmini/EnvironmentSimulator/Libraries/esminiLib)
set(ESMINI_LIBRARY_DIR /esmini/bin)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(esmini-bridge-autoware
  src/esmini-bridge-autoware.cpp
  src/World.cpp
  src/AutowareHandler.cpp
)
target_include_directories(esmini-bridge-autoware PUBLIC
  ${ESMINI_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(esmini-bridge-autoware PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(esmini-bridge-autoware
  rclcpp
  geometry_msgs
  autoware_adapi_v1_msgs
  autoware_control_msgs
  autoware_vehicle_msgs
  sensor_msgs
  tf2_msgs
)

find_library(ESMINI_LIB esminiLib PATHS ${ESMINI_LIBRARY_DIR})
target_link_libraries(esmini-bridge-autoware ${ESMINI_LIB})
target_link_options(esmini-bridge-autoware PRIVATE
  "-Wl,-rpath,${ESMINI_LIBRARY_DIR}"
)

install(TARGETS esmini-bridge-autoware
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

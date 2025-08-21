#include "cpp_esmini_bridge_autoware/AutowareHandler.hpp"
#include <chrono>

using namespace std::chrono_literals;

AutowareHandler::AutowareHandler(float init_x, float init_y, float init_h,
                                 float goal_x, float goal_y, float goal_h)
    : Node("AutowareHandler"), init_x(init_x), init_y(init_y), init_h(init_h),
      goal_x(goal_x), goal_y(goal_y), goal_h(goal_h) {
    this->initialpose_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);
    this->goalpose_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/planning/mission_planning/goal", 10);
    this->engage_autoware_client_ =
        this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
            "/api/operation_mode/change_to_autonomous");

    this->pub_control_mode_ =
        this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
            "/vehicle/status/control_mode", 10);
    this->pub_gear_report_ =
        this->create_publisher<autoware_vehicle_msgs::msg::GearReport>(
            "/vehicle/status/gear_status", 10);
    this->pub_steering_status_ =
        this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
            "/vehicle/status/steering_status", 10);
    this->pub_velocity_status_ =
        this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", 10);
    this->pub_accel_ =
        this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
            "/localization/acceleration", 10);
    this->pub_tf_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    this->pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/localization/pose_twist_fusion_filter/pose", 10);
    this->pub_imu_state_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/sensing/imu/imu_data", 10);
    this->pub_kinematic_state_ =
        this->create_publisher<nav_msgs::msg::Odometry>(
            "/localization/kinematic_state", 10);

    this->pub_predicted_objects_ =
        this->create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
            "/perception/object_recognition/objects", 10);

    this->sub_control_command_ =
        this->create_subscription<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd", 10,
            std::bind(&AutowareHandler::control_command_callback_, this,
                      std::placeholders::_1));
    this->sub_autoware_state_ =
        this->create_subscription<autoware_system_msgs::msg::AutowareState>(
            "/autoware/state", 10,
            std::bind(&AutowareHandler::autoware_state_callback_, this,
                      std::placeholders::_1));

    this->srv_control_mode_command_ =
        this->create_service<autoware_vehicle_msgs::srv::ControlModeCommand>(
            "/control/control_mode_request",
            std::bind(&AutowareHandler::control_mode_command_callback_, this,
                      std::placeholders::_1, std::placeholders::_2));
    this->srv_publish_initialpose_ =
        this->create_service<std_srvs::srv::Trigger>(
            "/publish_initialpose",
            std::bind(&AutowareHandler::publish_initialpose_callback_, this,
                      std::placeholders::_1, std::placeholders::_2));
    this->srv_publish_goalpose_ = this->create_service<std_srvs::srv::Trigger>(
        "/publish_goalpose",
        std::bind(&AutowareHandler::publish_goalpose_callback_, this,
                  std::placeholders::_1, std::placeholders::_2));

    this->timer_ = this->create_wall_timer(
        10ms, std::bind(&AutowareHandler::timer_callback, this));

    this->publish_initialpose_(init_x, init_y, init_h);
    this->publish_goalpose_(goal_x, goal_y, goal_h);
    this->engage_timer_ = this->create_wall_timer(
        10s, std::bind(&AutowareHandler::engage_autoware_, this));
}

void AutowareHandler::set_ego_pose(float x, float y, float h) {
    this->prev_ego_pose2 = this->prev_ego_pose;
    this->prev_ego_pose = this->ego_pose;
    this->ego_pose.x = x;
    this->ego_pose.y = y;
    this->ego_pose.h = h;
    this->ego_pose.time = this->now().seconds();
}

void AutowareHandler::calc_imu_state_() {
    float dt1 = this->now().seconds() - this->prev_ego_pose.time;
    float dt2 = this->prev_ego_pose.time - this->prev_ego_pose2.time;
    float dt = (dt1 + dt2) / 2.0;

    geometry_msgs::msg::Vector3 cur_v;
    cur_v.x = (this->ego_pose.x - this->prev_ego_pose.x) / dt1;
    cur_v.y = (this->ego_pose.y - this->prev_ego_pose.y) / dt1;
    cur_v.z = 0.0;
    geometry_msgs::msg::Vector3 prev_v;
    prev_v.x = (this->prev_ego_pose.x - this->prev_ego_pose2.x) / dt2;
    prev_v.y = (this->prev_ego_pose.y - this->prev_ego_pose2.y) / dt2;
    prev_v.z = 0.0;
    geometry_msgs::msg::Vector3 linear_acceleration;
    linear_acceleration.x = (cur_v.x - prev_v.x) / dt;
    linear_acceleration.y = (cur_v.y - prev_v.y) / dt;
    linear_acceleration.z = 0.0;

    geometry_msgs::msg::Vector3 angular_velocity;
    angular_velocity.x = 0.0;
    angular_velocity.y = 0.0;
    angular_velocity.z = (this->ego_pose.h - this->prev_ego_pose.h) / dt;
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
    initialpose.pose.covariance[0] = 0.25;
    initialpose.pose.covariance[6 * 1 + 1] = 0.25;
    initialpose.pose.covariance[6 * 5 + 5] = 0.06853891945200965;
    RCLCPP_INFO(this->get_logger(), "Publishing initialpose... %f %f %f", x, y,
                h);
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
    RCLCPP_INFO(this->get_logger(), "Publishing goalpose... %f %f %f", x, y, h);
    this->goalpose_publisher_->publish(goalpose);
}

void AutowareHandler::publish_initialpose_callback_(
    const std_srvs::srv::Trigger::Request::ConstSharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
    RCLCPP_INFO(this->get_logger(), "Publishing initialpose...");
    this->publish_initialpose_(init_x, init_y, init_h);
    response->success = true;
    response->message = "Initialpose published";
}

void AutowareHandler::publish_goalpose_callback_(
    const std_srvs::srv::Trigger::Request::ConstSharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
    RCLCPP_INFO(this->get_logger(), "Publishing goalpose...");
    this->publish_goalpose_(goal_x, goal_y, goal_h);
    response->success = true;
    response->message = "Goalpose published";
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

void AutowareHandler::publish_control_mode_() {
    autoware_vehicle_msgs::msg::ControlModeReport control_mode_report;
    control_mode_report.stamp = this->now();
    control_mode_report.mode =
        autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    this->pub_control_mode_->publish(control_mode_report);
}

void AutowareHandler::publish_gear_report_() {
    autoware_vehicle_msgs::msg::GearReport gear_report;
    gear_report.stamp = this->now();
    gear_report.report = autoware_vehicle_msgs::msg::GearReport::DRIVE;
    this->pub_gear_report_->publish(gear_report);
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
    transform.transform.translation.x = this->ego_pose.x;
    transform.transform.translation.y = this->ego_pose.y;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = sin(this->ego_pose.h / 2);
    transform.transform.rotation.w = cos(this->ego_pose.h / 2);
    tf.transforms.push_back(transform);
    this->pub_tf_->publish(tf);
}

void AutowareHandler::publish_pose_() {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "map";
    pose.pose.position.x = this->ego_pose.x;
    pose.pose.position.y = this->ego_pose.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = sin(this->ego_pose.h / 2);
    pose.pose.orientation.w = cos(this->ego_pose.h / 2);
    this->pub_pose_->publish(pose);
}

void AutowareHandler::publish_imu_state_() {
    this->pub_imu_state_->publish(this->imu_state);
}

void AutowareHandler::publish_kinematic_state_() {
    nav_msgs::msg::Odometry kinematic_state;
    kinematic_state.header.stamp = this->now();
    kinematic_state.header.frame_id = "map";
    kinematic_state.child_frame_id = "base_link";
    kinematic_state.pose.pose.position.x = this->ego_pose.x;
    kinematic_state.pose.pose.position.y = this->ego_pose.y;
    kinematic_state.pose.pose.position.z = 0.0;
    kinematic_state.pose.pose.orientation.x = 0.0;
    kinematic_state.pose.pose.orientation.y = 0.0;
    kinematic_state.pose.pose.orientation.z = sin(this->ego_pose.h / 2);
    kinematic_state.pose.pose.orientation.w = cos(this->ego_pose.h / 2);
    kinematic_state.twist.twist.linear.x = this->velocity;
    kinematic_state.twist.twist.angular.z = this->rotation;
    this->pub_kinematic_state_->publish(kinematic_state);
}

void AutowareHandler::set_object(int id, float x, float y, float h, float v) {
    autoware_perception_msgs::msg::PredictedObject predict_obj;
    autoware_perception_msgs::msg::ObjectClassification classification;

    float diff_x = x - this->ego_pose.x;
    float diff_y = y - this->ego_pose.y;
    float rel_h = h - this->ego_pose.h;
    float rel_x =
        cos(-this->ego_pose.h) * diff_x - sin(-this->ego_pose.h) * diff_y;
    float rel_y =
        sin(-this->ego_pose.h) * diff_x + cos(-this->ego_pose.h) * diff_y;
    classification.label =
        autoware_perception_msgs::msg::ObjectClassification::CAR;
    classification.probability = 1.0;

    predict_obj.classification.push_back(classification);
    predict_obj.kinematics.initial_pose_with_covariance.pose.position.x = x;
    predict_obj.kinematics.initial_pose_with_covariance.pose.position.y = y;
    predict_obj.kinematics.initial_pose_with_covariance.pose.position.z = 0.0;
    predict_obj.kinematics.initial_pose_with_covariance.pose.orientation.x =
        0.0;
    predict_obj.kinematics.initial_pose_with_covariance.pose.orientation.y =
        0.0;
    predict_obj.kinematics.initial_pose_with_covariance.pose.orientation.z =
        sin(h / 2);
    predict_obj.kinematics.initial_pose_with_covariance.pose.orientation.w =
        cos(h / 2);
    predict_obj.kinematics.initial_twist_with_covariance.twist.linear.x = v;
    predict_obj.kinematics.initial_twist_with_covariance.twist.linear.y = 0.0;
    predict_obj.kinematics.initial_twist_with_covariance.twist.angular.z = 0.0;
    predict_obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    predict_obj.shape.dimensions.x = 4.5;
    predict_obj.shape.dimensions.y = 2.0;
    predict_obj.shape.dimensions.z = 1.5;
    this->predicted_objects_map[id] = predict_obj;
}

void AutowareHandler::publish_objects_() {
    autoware_perception_msgs::msg::PredictedObjects predicted_objects;
    predicted_objects.header.stamp = this->now();
    predicted_objects.header.frame_id = "base_link";
    for (auto &[i, obj] : this->predicted_objects_map) {
        predicted_objects.objects.push_back(obj);
    }
    this->pub_predicted_objects_->publish(predicted_objects);
}

void AutowareHandler::control_command_callback_(
    const autoware_control_msgs::msg::Control::SharedPtr msg) {
    this->rotation = msg->lateral.steering_tire_angle;
    this->velocity = msg->longitudinal.velocity;
}

void AutowareHandler::autoware_state_callback_(
    const autoware_system_msgs::msg::AutowareState::SharedPtr msg) {
    if (msg->state == autoware_system_msgs::msg::AutowareState::INITIALIZING) {
        this->ego_state = EgoState::INITIALIZING;
    } else if (msg->state == autoware_system_msgs::msg::AutowareState::
                                 WAITING_FOR_ROUTE ||
               msg->state ==
                   autoware_system_msgs::msg::AutowareState::PLANNING) {
        this->ego_state = EgoState::PLANNING;
    } else if (msg->state ==
               autoware_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE) {
        this->ego_state = EgoState::WAITING_FOR_ENGAGE;
    } else if (msg->state ==
               autoware_system_msgs::msg::AutowareState::DRIVING) {
        this->ego_state = EgoState::DRIVING;
    } else if (msg->state ==
                   autoware_system_msgs::msg::AutowareState::ARRIVED_GOAL ||
               msg->state ==
                   autoware_system_msgs::msg::AutowareState::FINALIZING) {
        this->ego_state = EgoState::FINALIZED;
    } else {
        this->ego_state = EgoState::UNKNOWN;
    }
}

void AutowareHandler::control_mode_command_callback_(
    const autoware_vehicle_msgs::srv::ControlModeCommand::Request::
        ConstSharedPtr request,
    autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr
        response) {
    RCLCPP_INFO(this->get_logger(), "Received control mode command: %d",
                request->mode);
    response->success = true;
}

void AutowareHandler::timer_callback() {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Velocity: %f, Rotation: %f", this->velocity,
                         this->rotation);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Ego State: %f, %f, %f", this->ego_pose.x,
                         this->ego_pose.y, this->ego_pose.h);
    this->publish_control_mode_();
    this->publish_gear_report_();
    this->publish_steering_();
    this->publish_velocity_();

    this->calc_imu_state_();
    this->publish_accel_();
    this->publish_tf_();
    this->publish_pose_();
    this->publish_imu_state_();
    this->publish_kinematic_state_();

    this->publish_objects_();
}

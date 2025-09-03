#include "cpp_esmini_bridge_autoware/World.hpp"
#include "cpp_esmini_bridge_autoware/AutowareHandler.hpp"
#include "esminiLib.hpp"
#include <chrono>

using namespace std::chrono_literals;

void parameter_callback(void *) {
    assert(SE_SetParameterDouble("Agent1_Offset", 5) == 0);
    size_t n = SE_GetNumberOfParameters();
    for (size_t i = 0; i < n; ++i) {
        int type;
        std::string name(SE_GetParameterName(i, &type));
        switch (type) {
        case 1:
            int value_int;
            assert(SE_GetParameterInt(name.c_str(), &value_int) == 0);
            break;
        case 2:
            double value_double;
            assert(SE_GetParameterDouble(name.c_str(), &value_double) == 0);
            break;
        case 3:
            const char *value_string;
            assert(SE_GetParameterString(name.c_str(), &value_string) == 0);
            break;
        case 4:
            bool value_bool;
            assert(SE_GetParameterBool(name.c_str(), &value_bool) == 0);
            break;
        default:
            break;
        }
    }
    return;
}

World::World()
    : Node("World"), ego_state(EgoState::INITIALIZING), limiter(10s) {
    RCLCPP_INFO(this->get_logger(), "World Node Initialized");
    this->esmini_init();
    RCLCPP_INFO(this->get_logger(), "Esmini Initialized");
    this->ego = std::make_shared<AutowareHandler>();
    timer_ =
        this->create_wall_timer(10ms, std::bind(&World::timer_callback, this));
}

void World::esmini_init() {
    SE_SetOptionValue("text_scale", "2");
    SE_SetWindowPosAndSize(0, 0, 1600, 900);
    SE_RegisterParameterDeclarationCallback(parameter_callback, 0);
    SE_AddPath("/esmini/resources/xosc/");

    // SE_Init("/esmini/resources/xosc/cut-in.xosc", 0, 1, 0, 0);
    // SE_Init("/resources/xosc/chengyu/SinD_test1.xosc", 1, 1, 0, 0);
    // SE_Init("/resources/xosc/yusheng/145.xosc", 1, 1, 0, 1);
    assert(SE_Init("/resources/xosc/chengyu/01FL-KEEP_02FR-TL_254.xosc", 1, 1,
                   0, 1) == 0);
    SE_CollisionDetection(true);
    SE_GetObjectState(0, &this->objectState);
    vehicleHandle = SE_SimpleVehicleCreate(
        this->objectState.x, this->objectState.y, this->objectState.h,
        this->objectState.length, this->objectState.speed);
    SE_SimpleVehicleSteeringRate(vehicleHandle, 9.0f);
    SE_SimpleVehicleSetThrottleDisabled(vehicleHandle, true);
    // SE_ViewerShowFeature(4 + 8, true);
}

void World::timer_callback() {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Ego State: %d",
                         static_cast<int>(this->ego->get_state()));
    switch (this->ego->get_state()) {
    case EgoState::INITIALIZING:
        limiter.call([this] { this->set_ego_route(); });
        break;
    case EgoState::PLANNING:
        break;
    case EgoState::WAITING_FOR_ENGAGE:
        limiter.call([this] { this->ego->engage(); });
        break;
    case EgoState::DRIVING:
        this->tick();
        break;
    case EgoState::FINALIZED:
        RCLCPP_INFO(this->get_logger(), "Ego State: FINALIZED");
        break;
    default:
        break;
    }
}

void World::set_ego_route() {
    SE_ScenarioObjectState ego_pose;
    SE_GetObjectState(0, &ego_pose);
    RCLCPP_DEBUG(this->get_logger(), "Ego State: %f, %f, %f", ego_pose.x,
                 ego_pose.y, ego_pose.h);
    this->ego->set_initial_pose(ego_pose.x, ego_pose.y, ego_pose.h);
    // this->ego->set_goal_pose(6.5, 299.6, 1.57);
    // this->ego->set_goal_pose(52.5, 11.6, 6.28);
    this->ego->set_goal_pose(678.5, -28.6, 1.7);
}

void World::tick() {
    float dt = SE_GetSimTimeStep();
    SE_SimpleVehicleControlAnalog(vehicleHandle, dt, 0,
                                  this->ego->get_rotation());
    SE_SimpleVehicleSetSpeed(vehicleHandle, this->ego->get_velocity());
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Throttle: %f, Rotation: %f",
                          this->ego->get_velocity(), this->ego->get_rotation());
    SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Number of objects: %d", SE_GetNumberOfObjects());
    for (int i = 1; i < SE_GetNumberOfObjects(); i++) {
        SE_GetObjectState(SE_GetId(i), &this->objectState);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Object %d: %f, %f, %f, Speed: %f",
                             this->objectState.id, this->objectState.x,
                             this->objectState.y, this->objectState.h,
                             this->objectState.speed);
        this->ego->set_object(this->objectState.id, this->objectState.x,
                              this->objectState.y, this->objectState.h,
                              this->objectState.speed);
    }
    SE_ReportObjectPosXYH(0, 0, vehicleState.x, vehicleState.y, vehicleState.h);
    SE_ReportObjectWheelStatus(0, vehicleState.wheel_rotation,
                               vehicleState.wheel_angle);
    SE_ReportObjectSpeed(0, vehicleState.speed);
    SE_StepDT(dt);
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                          "Vehicle State: %f, %f, %f", vehicleState.x,
                          vehicleState.y, vehicleState.h);
    this->ego->set_ego_pose(vehicleState.x, vehicleState.y, vehicleState.h);
}

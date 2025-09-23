#include "cpp_esmini_bridge_autoware/World.hpp"
#include "cpp_esmini_bridge_autoware/AutowareHandler.hpp"
#include "esminiLib.hpp"
#include <chrono>

using namespace std::chrono_literals;

void parameter_callback(void *) {
    // assert(SE_SetParameterDouble("Agent1_Offset", 5) == 0);
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
    : Node("World"), world_state(WorldState::AV_CONNECTING), limiter(10s) {
    RCLCPP_INFO(this->get_logger(), "World Node Initialized");
    this->declare_parameter("xosc", "default parameter not overridden");
    this->xosc = this->get_parameter("xosc").as_string();
    this->esmini_init();
    RCLCPP_INFO(this->get_logger(), "Esmini Initialized");
    this->ego = std::make_shared<AutowareHandler>();
    timer_ =
        this->create_wall_timer(10ms, std::bind(&World::timer_callback, this));
}

void World::esmini_opts() {
    SE_SetOptionValue("text_scale", "2");
    SE_SetWindowPosAndSize(1600, 900, 1600, 900);
    SE_RegisterParameterDeclarationCallback(parameter_callback, 0);
    SE_AddPath("/esmini/resources/xosc/");
    SE_CollisionDetection(true);
}

void World::esmini_init() {
    esmini_opts();
    // std::string xosc = "/esmini/resources/xosc/cut-in.xosc";
    // std::string xosc = "/resources/xosc/chengyu/SinD_test1.xosc";
    // std::string xosc = "/resources/xosc/yusheng/145.xosc";
    RCLCPP_INFO(this->get_logger(), "Loading XOSC: %s", this->xosc.c_str());
    assert(SE_Init(this->xosc.c_str(), 1, 0, 0, 1) == 0);
    int index = SE_GetPermutationIndex();
    RCLCPP_INFO(this->get_logger(), "Permutation Index: %d", index);
    SE_GetObjectState(0, &this->objectState);
    this->vehicleHandle = SE_SimpleVehicleCreate(
        this->objectState.x, this->objectState.y, this->objectState.h,
        this->objectState.length, this->objectState.speed);
    SE_SimpleVehicleSteeringRate(this->vehicleHandle, 9.0f);
    SE_SimpleVehicleSetThrottleDisabled(this->vehicleHandle, true);
}

void World::esmini_close() {
    SE_SimpleVehicleDelete(this->vehicleHandle);
    SE_Close();
}

void World::timer_callback() {
    switch (this->world_state) {
    case WorldState::AV_CONNECTING:
        if (this->ego->get_state() != EgoState::UNKNOWN) {
            RCLCPP_INFO(this->get_logger(),
                        "WorldState: AV_CONNECTING -> WAITING_FOR_PLANNING");
            this->world_state = WorldState::WAITING_FOR_PLANNING;
        }
        break;
    case WorldState::WAITING_FOR_PLANNING:
        limiter.call([this] { this->set_ego_route(); });
        if (this->ego->get_state() == EgoState::WAITING_FOR_ENGAGE) {
            this->world_state = WorldState::ENGAGING;
            RCLCPP_INFO(this->get_logger(),
                        "WorldState: WAITING_FOR_PLANNING -> ENGAGING");
        }
        break;
    case WorldState::ENGAGING:
        limiter.call([this] { this->ego->engage(); });
        if (this->ego->get_state() == EgoState::DRIVING) {
            this->world_state = WorldState::RUNNING;
            RCLCPP_INFO(this->get_logger(), "WorldState: ENGAGING -> RUNNING");
        }
        break;
    case WorldState::RUNNING:
        if (SE_GetQuitFlag()) {
            limiter.call([this] { this->ego->stop(); });
            this->world_state = WorldState::STOPPED;
        }
        this->tick();
        break;
    case WorldState::STOPPED:
        if (this->ego->get_state() != EgoState::DRIVING) {
            RCLCPP_INFO(this->get_logger(),
                        "WorldState: STOPPED -> AV_CONNECTING");
            this->world_state = WorldState::AV_CONNECTING;
            esmini_close();
            esmini_init();
        }
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
    this->ego->set_goal_pose(672, 41.6, 1.66);
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
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
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

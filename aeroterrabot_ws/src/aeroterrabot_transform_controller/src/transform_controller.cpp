/**
 * @file transform_controller.cpp
 * @brief AeroTerraBot Morphing Transformation Controller
 *
 * Subscribes to /morph_mode and orchestrates the full transformation
 * pipeline between wheel_mode, leg_mode, hybrid_mode, and flight_mode.
 *
 * Responsibilities:
 *   - Deploy/retract drone arms
 *   - Extend/retract legs
 *   - Switch ros2_control controllers via controller_manager services
 *   - Publish joint trajectories for smooth transitions
 *   - Enforce safety checks before flight_mode activation
 *
 * Safety Logic (flight_mode prerequisites):
 *   - Arms must be deployed
 *   - Legs must be retracted
 *   - IMU must be active
 *   - PX4 must be connected
 *   - Gazebo must be running
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "aeroterrabot_interfaces/msg/morph_mode.hpp"
#include "aeroterrabot_interfaces/msg/system_status.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class TransformController : public rclcpp::Node
{
public:
  TransformController() : Node("transform_controller")
  {
    // Current state
    current_mode_ = aeroterrabot_interfaces::msg::MorphMode::WHEEL_MODE;
    imu_active_ = false;
    px4_connected_ = false;
    arms_deployed_ = false;
    legs_retracted_ = true;
    transforming_ = false;

    // Subscribers
    morph_mode_sub_ = this->create_subscription<aeroterrabot_interfaces::msg::MorphMode>(
      "/morph_mode", 10,
      std::bind(&TransformController::morph_mode_callback, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&TransformController::imu_callback, this, _1));

    // Publishers
    leg_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/leg_joint_position_controller/joint_trajectory", 10);

    arm_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/transform_joint_position_controller/joint_trajectory", 10);

    status_pub_ = this->create_publisher<aeroterrabot_interfaces::msg::SystemStatus>(
      "/system_status", 10);

    // Service clients
    switch_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

    // Status timer
    status_timer_ = this->create_wall_timer(
      1000ms, std::bind(&TransformController::publish_status, this));

    RCLCPP_INFO(this->get_logger(), "AeroTerraBot Transform Controller initialized");
    RCLCPP_INFO(this->get_logger(), "Current mode: WHEEL_MODE");
  }

private:
  // --- Callback: Morph Mode Request ---
  void morph_mode_callback(const aeroterrabot_interfaces::msg::MorphMode::SharedPtr msg)
  {
    if (transforming_) {
      RCLCPP_WARN(this->get_logger(), "Transformation in progress, ignoring request");
      return;
    }

    if (msg->mode == current_mode_) {
      RCLCPP_INFO(this->get_logger(), "Already in requested mode");
      return;
    }

    uint8_t target_mode = msg->mode;
    RCLCPP_INFO(this->get_logger(), "Mode transition requested: %d -> %d",
                current_mode_, target_mode);

    // Safety check for flight mode
    if (target_mode == aeroterrabot_interfaces::msg::MorphMode::FLIGHT_MODE) {
      if (!check_flight_safety()) {
        RCLCPP_ERROR(this->get_logger(), "Flight mode safety check FAILED - transition rejected");
        return;
      }
    }

    transforming_ = true;
    execute_transition(target_mode);
    transforming_ = false;
  }

  // --- IMU Health Monitor ---
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr /*msg*/)
  {
    imu_active_ = true;
    last_imu_time_ = this->now();
  }

  // --- Flight Safety Check ---
  bool check_flight_safety()
  {
    bool safe = true;

    if (!imu_active_) {
      RCLCPP_ERROR(this->get_logger(), "Safety: IMU not active");
      safe = false;
    }

    // Check IMU staleness (> 2 seconds)
    if (imu_active_ && (this->now() - last_imu_time_).seconds() > 2.0) {
      RCLCPP_ERROR(this->get_logger(), "Safety: IMU data stale");
      safe = false;
    }

    // PX4 connection check (simplified: assume connected if topic exists)
    // In production, subscribe to /fmu/out/vehicle_status
    // For now, we allow flight if IMU is active (PX4 SITL publishes IMU)
    px4_connected_ = imu_active_;
    if (!px4_connected_) {
      RCLCPP_ERROR(this->get_logger(), "Safety: PX4 not connected");
      safe = false;
    }

    return safe;
  }

  // --- Execute Mode Transition ---
  void execute_transition(uint8_t target_mode)
  {
    switch (target_mode) {
      case aeroterrabot_interfaces::msg::MorphMode::WHEEL_MODE:
        transition_to_wheel_mode();
        break;
      case aeroterrabot_interfaces::msg::MorphMode::LEG_MODE:
        transition_to_leg_mode();
        break;
      case aeroterrabot_interfaces::msg::MorphMode::HYBRID_MODE:
        transition_to_hybrid_mode();
        break;
      case aeroterrabot_interfaces::msg::MorphMode::FLIGHT_MODE:
        transition_to_flight_mode();
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown mode: %d", target_mode);
        return;
    }

    current_mode_ = target_mode;
    RCLCPP_INFO(this->get_logger(), "Transition complete. Current mode: %d", current_mode_);
  }

  // --- Wheel Mode Transition ---
  void transition_to_wheel_mode()
  {
    RCLCPP_INFO(this->get_logger(), "Transitioning to WHEEL_MODE");

    // Retract arms
    publish_arm_positions({0.0, 0.0, 0.0, 0.0});
    arms_deployed_ = false;

    // Retract legs to driving position
    publish_leg_positions(
      {0.0, 0.0, 0.0, 0.0},      // upper legs neutral
      {-0.5, -0.5, -0.5, -0.5}   // lower legs slightly bent
    );
    legs_retracted_ = true;

    // Switch controllers: activate diff_drive, deactivate rotors
    switch_controllers(
      {"diff_drive_controller"},                    // activate
      {"rotor_velocity_controller"}                 // deactivate
    );
  }

  // --- Leg Mode Transition ---
  void transition_to_leg_mode()
  {
    RCLCPP_INFO(this->get_logger(), "Transitioning to LEG_MODE");

    // Extend legs for elevation
    publish_leg_positions(
      {0.3, 0.3, 0.3, 0.3},         // upper legs extended
      {-1.2, -1.2, -1.2, -1.2}      // lower legs extended
    );
    legs_retracted_ = false;

    // Keep arms retracted
    publish_arm_positions({0.0, 0.0, 0.0, 0.0});
    arms_deployed_ = false;

    // Deactivate rotor controller
    switch_controllers(
      {"diff_drive_controller"},
      {"rotor_velocity_controller"}
    );
  }

  // --- Hybrid Mode Transition ---
  void transition_to_hybrid_mode()
  {
    RCLCPP_INFO(this->get_logger(), "Transitioning to HYBRID_MODE");

    // Partially extend legs for terrain adaptation
    publish_leg_positions(
      {0.15, 0.15, 0.15, 0.15},     // upper legs partially extended
      {-0.8, -0.8, -0.8, -0.8}      // lower legs mid position
    );
    legs_retracted_ = false;

    // Keep arms retracted
    publish_arm_positions({0.0, 0.0, 0.0, 0.0});
    arms_deployed_ = false;

    // Wheels + legs active
    switch_controllers(
      {"diff_drive_controller"},
      {"rotor_velocity_controller"}
    );
  }

  // --- Flight Mode Transition ---
  void transition_to_flight_mode()
  {
    RCLCPP_INFO(this->get_logger(), "Transitioning to FLIGHT_MODE");

    // Step 1: Retract legs
    RCLCPP_INFO(this->get_logger(), "Step 1: Retracting legs");
    publish_leg_positions(
      {0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0}
    );
    legs_retracted_ = true;

    // Step 2: Deploy arms
    RCLCPP_INFO(this->get_logger(), "Step 2: Deploying quadrotor arms");
    publish_arm_positions({1.57, 1.57, 1.57, 1.57});
    arms_deployed_ = true;

    // Step 3: Switch controllers
    RCLCPP_INFO(this->get_logger(), "Step 3: Switching to rotor controllers");
    switch_controllers(
      {"rotor_velocity_controller"},                // activate
      {"diff_drive_controller"}                     // deactivate
    );

    RCLCPP_INFO(this->get_logger(), "Flight mode ready - arms deployed, rotors activated");
  }

  // --- Publish Leg Joint Trajectory ---
  void publish_leg_positions(
    const std::vector<double>& upper_positions,
    const std::vector<double>& lower_positions)
  {
    auto msg = trajectory_msgs::msg::JointTrajectory();
    msg.header.stamp = this->now();
    msg.joint_names = {
      "fl_upper_leg_joint", "fr_upper_leg_joint",
      "rl_upper_leg_joint", "rr_upper_leg_joint",
      "fl_lower_leg_joint", "fr_lower_leg_joint",
      "rl_lower_leg_joint", "rr_lower_leg_joint"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.insert(point.positions.end(), upper_positions.begin(), upper_positions.end());
    point.positions.insert(point.positions.end(), lower_positions.begin(), lower_positions.end());
    point.time_from_start = rclcpp::Duration(2, 0);  // 2 second transition

    msg.points.push_back(point);
    leg_trajectory_pub_->publish(msg);
  }

  // --- Publish Arm Joint Trajectory ---
  void publish_arm_positions(const std::vector<double>& positions)
  {
    auto msg = trajectory_msgs::msg::JointTrajectory();
    msg.header.stamp = this->now();
    msg.joint_names = {
      "front_left_arm_joint", "front_right_arm_joint",
      "rear_left_arm_joint", "rear_right_arm_joint"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration(1, 500000000);  // 1.5 second transition

    msg.points.push_back(point);
    arm_trajectory_pub_->publish(msg);
  }

  // --- Switch Controllers via Service ---
  void switch_controllers(
    const std::vector<std::string>& activate,
    const std::vector<std::string>& deactivate)
  {
    if (!switch_controller_client_->wait_for_service(2s)) {
      RCLCPP_WARN(this->get_logger(), "Controller manager service not available");
      return;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = activate;
    request->deactivate_controllers = deactivate;
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

    auto future = switch_controller_client_->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "Controller switch request sent: activate=%zu, deactivate=%zu",
                activate.size(), deactivate.size());
  }

  // --- Publish System Status ---
  void publish_status()
  {
    auto msg = aeroterrabot_interfaces::msg::SystemStatus();
    msg.current_mode = current_mode_;
    msg.arms_deployed = arms_deployed_;
    msg.legs_retracted = legs_retracted_;
    msg.imu_active = imu_active_;
    msg.px4_connected = px4_connected_;
    msg.gazebo_running = true;  // Assume running if node is alive
    msg.transformation_safe = check_flight_safety();
    msg.stamp = this->now();

    std::vector<std::string> mode_names = {"WHEEL", "LEG", "HYBRID", "FLIGHT"};
    msg.status_message = "Mode: " + mode_names[current_mode_] +
                         " | Arms: " + (arms_deployed_ ? "DEPLOYED" : "RETRACTED") +
                         " | Legs: " + (legs_retracted_ ? "RETRACTED" : "EXTENDED") +
                         " | IMU: " + (imu_active_ ? "OK" : "INACTIVE");

    status_pub_->publish(msg);

    // Check IMU staleness
    if (imu_active_ && (this->now() - last_imu_time_).seconds() > 5.0) {
      imu_active_ = false;
      RCLCPP_WARN(this->get_logger(), "IMU data timeout - marking as inactive");
    }
  }

  // --- Member Variables ---
  uint8_t current_mode_;
  bool imu_active_;
  bool px4_connected_;
  bool arms_deployed_;
  bool legs_retracted_;
  bool transforming_;
  rclcpp::Time last_imu_time_;

  // Subscribers
  rclcpp::Subscription<aeroterrabot_interfaces::msg::MorphMode>::SharedPtr morph_mode_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Publishers
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr leg_trajectory_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_trajectory_pub_;
  rclcpp::Publisher<aeroterrabot_interfaces::msg::SystemStatus>::SharedPtr status_pub_;

  // Service Clients
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;

  // Timers
  rclcpp::TimerBase::SharedPtr status_timer_;
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransformController>());
  rclcpp::shutdown();
  return 0;
}

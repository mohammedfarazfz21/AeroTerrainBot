/**
 * @file offboard_hover.cpp
 * @brief PX4 Offboard Control for AeroTerraBot Flight Mode
 *
 * Publishes trajectory setpoints to maintain hover at: x=0, y=0, z=1.5
 * Requires PX4 SITL and microRTPS/MicroXRCEAgent.
 */
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
  OffboardControl() : Node("offboard_hover")
  {
    offboard_control_mode_publisher_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ =
      this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    offboard_setpoint_counter_ = 0;

    auto timer_callback = [this]() -> void {
      if (offboard_setpoint_counter_ == 10) {
        // Change to Offboard mode after 10 setpoints
        this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        // Arm the vehicle
        this->arm();
      }

      // offboard_control_mode needs to be paired with trajectory_setpoint
      publish_offboard_control_mode();
      publish_trajectory_setpoint();

      if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
      }
    };
    timer_ = this->create_wall_timer(100ms, timer_callback);
  }

  void arm()
  {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
  }

  void disarm()
  {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

  uint64_t offboard_setpoint_counter_;

  void publish_offboard_control_mode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
  }

  void publish_trajectory_setpoint()
  {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, -1.5}; // NED Frame: -1.5m Down -> 1.5m Up
    msg.yaw = -3.14; // Default yaw
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
  }

  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
  }
};

int main(int argc, char* argv[])
{
  std::cout << "Starting offboard hover node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());

  rclcpp::shutdown();
  return 0;
}

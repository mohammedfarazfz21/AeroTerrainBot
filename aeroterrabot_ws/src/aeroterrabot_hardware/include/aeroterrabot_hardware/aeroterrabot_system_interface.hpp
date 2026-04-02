// Copyright 2026 AeroTerraBot Authors
// Licensed under the Apache License, Version 2.0

#ifndef AEROTERRABOT_HARDWARE__AEROTERRABOT_SYSTEM_INTERFACE_HPP_
#define AEROTERRABOT_HARDWARE__AEROTERRABOT_SYSTEM_INTERFACE_HPP_

#include <string>
#include <vector>
#include <cstdint>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace aeroterrabot_hardware
{

// ── Serial protocol constants ────────────────────────────────────────────────
constexpr uint8_t SYNC_BYTE_1 = 0xAA;
constexpr uint8_t SYNC_BYTE_2 = 0x55;

// 12 joints × 4 bytes each = 48 bytes payload
constexpr size_t NUM_JOINTS = 12;
constexpr size_t TX_PAYLOAD_SIZE = NUM_JOINTS * sizeof(float);  // 48
constexpr size_t TX_FRAME_SIZE  = 2 + TX_PAYLOAD_SIZE + 1;      // sync(2) + payload(48) + crc(1) = 51
constexpr size_t RX_PAYLOAD_SIZE = NUM_JOINTS * sizeof(float) + 1;  // 48 + mode_flag(1) = 49
constexpr size_t RX_FRAME_SIZE  = 2 + RX_PAYLOAD_SIZE + 1;      // sync(2) + payload(49) + crc(1) = 52

// Joint index ranges (within the 12-joint array)
constexpr size_t WHEEL_START  = 0;   // joints 0-3:  wheel motors
constexpr size_t WHEEL_COUNT  = 4;
constexpr size_t ROTOR_START  = 4;   // joints 4-7:  rotor motors
constexpr size_t ROTOR_COUNT  = 4;
constexpr size_t SERVO_START  = 8;   // joints 8-9:  morphing servos
constexpr size_t SERVO_COUNT  = 2;
constexpr size_t ACTUATOR_START = 10; // joints 10-11: linear actuators
constexpr size_t ACTUATOR_COUNT = 2;

class AeroTerraBotSystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AeroTerraBotSystemInterface)

  // ── Lifecycle callbacks ──────────────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── Interface export ─────────────────────────────────────────────────────
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── Read / Write cycle ───────────────────────────────────────────────────
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Serial helpers ───────────────────────────────────────────────────────
  bool open_serial_port();
  void close_serial_port();
  uint8_t compute_crc8(const uint8_t * data, size_t length);

  // ── Configuration ────────────────────────────────────────────────────────
  std::string serial_port_{"/dev/ttyACM0"};
  int baud_rate_{115200};
  int serial_fd_{-1};

  // ── Joint state storage (read from Teensy) ───────────────────────────────
  std::vector<double> hw_positions_;     // 12 values
  std::vector<double> hw_velocities_;    // 12 values

  // ── Joint command storage (sent to Teensy) ───────────────────────────────
  std::vector<double> hw_commands_;      // 12 values

  // ── Mode flag (from Teensy feedback) ─────────────────────────────────────
  bool is_drone_mode_{false};
};

}  // namespace aeroterrabot_hardware

#endif  // AEROTERRABOT_HARDWARE__AEROTERRABOT_SYSTEM_INTERFACE_HPP_

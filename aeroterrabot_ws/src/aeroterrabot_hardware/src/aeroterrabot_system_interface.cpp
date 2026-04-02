// Copyright 2026 AeroTerraBot Authors
// Licensed under the Apache License, Version 2.0

#include "aeroterrabot_hardware/aeroterrabot_system_interface.hpp"

#include <cstring>
#include <algorithm>
#include <cerrno>

// POSIX serial I/O
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aeroterrabot_hardware
{

// ════════════════════════════════════════════════════════════════════════════
// Lifecycle: on_init  —  Parse URDF <ros2_control> hardware parameters
// ════════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn AeroTerraBotSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read hardware parameters from URDF
  if (info_.hardware_parameters.count("serial_port")) {
    serial_port_ = info_.hardware_parameters.at("serial_port");
  }
  if (info_.hardware_parameters.count("baud_rate")) {
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  }

  // Validate joint count
  if (info_.joints.size() != NUM_JOINTS) {
    RCLCPP_FATAL(
      rclcpp::get_logger("AeroTerraBotSystemInterface"),
      "Expected %zu joints but got %zu in URDF.", NUM_JOINTS, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Resize state/command vectors
  hw_positions_.resize(NUM_JOINTS, 0.0);
  hw_velocities_.resize(NUM_JOINTS, 0.0);
  hw_commands_.resize(NUM_JOINTS, 0.0);

  RCLCPP_INFO(
    rclcpp::get_logger("AeroTerraBotSystemInterface"),
    "Initialized with serial_port=%s  baud_rate=%d  joints=%zu",
    serial_port_.c_str(), baud_rate_, info_.joints.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ════════════════════════════════════════════════════════════════════════════
// Lifecycle: on_configure  —  Prepare (no serial yet)
// ════════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn AeroTerraBotSystemInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Zero out all states and commands
  std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  is_drone_mode_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("AeroTerraBotSystemInterface"),
    "Configured. States and commands zeroed.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ════════════════════════════════════════════════════════════════════════════
// Lifecycle: on_activate  —  Open serial port to Teensy 4.0
// ════════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn AeroTerraBotSystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!open_serial_port()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AeroTerraBotSystemInterface"),
      "Failed to open serial port %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("AeroTerraBotSystemInterface"),
    "Serial port %s opened. Hardware activated.", serial_port_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ════════════════════════════════════════════════════════════════════════════
// Lifecycle: on_deactivate  —  Close serial port
// ════════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn AeroTerraBotSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_serial_port();

  RCLCPP_INFO(
    rclcpp::get_logger("AeroTerraBotSystemInterface"),
    "Serial port closed. Hardware deactivated.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ════════════════════════════════════════════════════════════════════════════
// export_state_interfaces  —  position + velocity for each of the 12 joints
// ════════════════════════════════════════════════════════════════════════════
std::vector<hardware_interface::StateInterface>
AeroTerraBotSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
  }
  return state_interfaces;
}

// ════════════════════════════════════════════════════════════════════════════
// export_command_interfaces
//   - Wheels (0-3)  & Rotors (4-7):  velocity commands
//   - Servos (8-9)  & Actuators (10-11):  position commands
// ════════════════════════════════════════════════════════════════════════════
std::vector<hardware_interface::CommandInterface>
AeroTerraBotSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    // Wheels and rotors use velocity; servos and actuators use position
    const std::string iface_type =
      (i < SERVO_START)
        ? hardware_interface::HW_IF_VELOCITY
        : hardware_interface::HW_IF_POSITION;

    command_interfaces.emplace_back(
      info_.joints[i].name,
      iface_type,
      &hw_commands_[i]);
  }
  return command_interfaces;
}

// ════════════════════════════════════════════════════════════════════════════
// read()  —  Receive feedback frame from Teensy
//
//  Frame layout (RX):
//    [0xAA] [0x55]  <12 × float32 states>  <uint8 mode_flag>  <CRC8>
//    Total = 52 bytes
// ════════════════════════════════════════════════════════════════════════════
hardware_interface::return_type AeroTerraBotSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (serial_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  uint8_t rx_buf[RX_FRAME_SIZE];

  // ── Synchronize on 0xAA 0x55 ──────────────────────────────────────────
  bool synced = false;
  uint8_t byte = 0;
  int max_scan = 256;  // prevent infinite spinning
  while (max_scan-- > 0) {
    ssize_t n = ::read(serial_fd_, &byte, 1);
    if (n != 1) {
      return hardware_interface::return_type::OK;  // no data yet
    }
    if (byte == SYNC_BYTE_1) {
      n = ::read(serial_fd_, &byte, 1);
      if (n == 1 && byte == SYNC_BYTE_2) {
        synced = true;
        break;
      }
    }
  }
  if (!synced) {
    return hardware_interface::return_type::OK;
  }

  // ── Read payload + CRC ────────────────────────────────────────────────
  const size_t remainder = RX_PAYLOAD_SIZE + 1;  // payload + crc
  size_t total_read = 0;
  while (total_read < remainder) {
    ssize_t n = ::read(serial_fd_, rx_buf + total_read, remainder - total_read);
    if (n <= 0) {
      return hardware_interface::return_type::OK;
    }
    total_read += static_cast<size_t>(n);
  }

  // ── Validate CRC ─────────────────────────────────────────────────────
  uint8_t received_crc = rx_buf[RX_PAYLOAD_SIZE];
  uint8_t computed_crc = compute_crc8(rx_buf, RX_PAYLOAD_SIZE);
  if (received_crc != computed_crc) {
    RCLCPP_WARN(
      rclcpp::get_logger("AeroTerraBotSystemInterface"),
      "CRC mismatch: expected 0x%02X, got 0x%02X", computed_crc, received_crc);
    return hardware_interface::return_type::OK;
  }

  // ── Unpack 12 × float32 joint states ──────────────────────────────────
  float values[NUM_JOINTS];
  std::memcpy(values, rx_buf, NUM_JOINTS * sizeof(float));

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    // For wheels and rotors, Teensy sends velocity feedback
    if (i < SERVO_START) {
      hw_velocities_[i] = static_cast<double>(values[i]);
    }
    // For servos and actuators, Teensy sends position feedback
    else {
      hw_positions_[i] = static_cast<double>(values[i]);
    }
  }

  // ── Unpack mode flag ──────────────────────────────────────────────────
  is_drone_mode_ = (rx_buf[NUM_JOINTS * sizeof(float)] != 0);

  return hardware_interface::return_type::OK;
}

// ════════════════════════════════════════════════════════════════════════════
// write()  —  Send command frame to Teensy
//
//  Frame layout (TX):
//    [0xAA] [0x55]  <12 × float32 commands>  <CRC8>
//    Total = 51 bytes
// ════════════════════════════════════════════════════════════════════════════
hardware_interface::return_type AeroTerraBotSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (serial_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  uint8_t tx_buf[TX_FRAME_SIZE];

  // ── Sync header ───────────────────────────────────────────────────────
  tx_buf[0] = SYNC_BYTE_1;
  tx_buf[1] = SYNC_BYTE_2;

  // ── Pack 12 × float32 commands ────────────────────────────────────────
  float cmd_floats[NUM_JOINTS];
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    cmd_floats[i] = static_cast<float>(hw_commands_[i]);
  }
  std::memcpy(tx_buf + 2, cmd_floats, TX_PAYLOAD_SIZE);

  // ── CRC ───────────────────────────────────────────────────────────────
  tx_buf[TX_FRAME_SIZE - 1] = compute_crc8(tx_buf + 2, TX_PAYLOAD_SIZE);

  // ── Write to serial ───────────────────────────────────────────────────
  ssize_t written = ::write(serial_fd_, tx_buf, TX_FRAME_SIZE);
  if (written != static_cast<ssize_t>(TX_FRAME_SIZE)) {
    RCLCPP_WARN(
      rclcpp::get_logger("AeroTerraBotSystemInterface"),
      "Serial write incomplete: %zd / %zu bytes", written, TX_FRAME_SIZE);
  }

  return hardware_interface::return_type::OK;
}

// ════════════════════════════════════════════════════════════════════════════
// Serial port helpers
// ════════════════════════════════════════════════════════════════════════════
bool AeroTerraBotSystemInterface::open_serial_port()
{
  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AeroTerraBotSystemInterface"),
      "Cannot open %s: %s", serial_port_.c_str(), strerror(errno));
    return false;
  }

  // Configure termios
  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));

  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AeroTerraBotSystemInterface"),
      "tcgetattr failed: %s", strerror(errno));
    close_serial_port();
    return false;
  }

  // Map baud rate
  speed_t speed = B115200;
  switch (baud_rate_) {
    case 9600:    speed = B9600;    break;
    case 19200:   speed = B19200;   break;
    case 38400:   speed = B38400;   break;
    case 57600:   speed = B57600;   break;
    case 115200:  speed = B115200;  break;
    case 230400:  speed = B230400;  break;
    case 460800:  speed = B460800;  break;
    case 921600:  speed = B921600;  break;
    default:
      RCLCPP_WARN(
        rclcpp::get_logger("AeroTerraBotSystemInterface"),
        "Unsupported baud %d, defaulting to 115200", baud_rate_);
      speed = B115200;
      break;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1, no hardware flow control
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
  tty.c_cflag |= (CLOCAL | CREAD);

  // Raw mode
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT |
                    PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  // Non-blocking reads
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;  // 100 ms timeout

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AeroTerraBotSystemInterface"),
      "tcsetattr failed: %s", strerror(errno));
    close_serial_port();
    return false;
  }

  // Flush any stale data
  tcflush(serial_fd_, TCIOFLUSH);

  return true;
}

void AeroTerraBotSystemInterface::close_serial_port()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

// ── CRC-8 (poly 0x07, init 0x00) ───────────────────────────────────────────
uint8_t AeroTerraBotSystemInterface::compute_crc8(
  const uint8_t * data, size_t length)
{
  uint8_t crc = 0x00;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
    }
  }
  return crc;
}

}  // namespace aeroterrabot_hardware

// ── Register plugin with class_loader ───────────────────────────────────────
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  aeroterrabot_hardware::AeroTerraBotSystemInterface,
  hardware_interface::SystemInterface)

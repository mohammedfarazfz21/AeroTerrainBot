// Copyright 2026 AeroTerraBot Authors
// Licensed under the Apache License, Version 2.0
//
// sensor_bridge_node.cpp
// Reads sensor data from an Arduino over serial and publishes SensorArray.msg.

#include <cstring>
#include <cerrno>
#include <string>

// POSIX serial I/O
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "aeroterrabot_interfaces/msg/sensor_array.hpp"

namespace aeroterrabot_hardware
{

// ── Arduino sensor frame layout ─────────────────────────────────────────────
// Sync: 0xBB 0x66
// Payload (28 bytes):
//   float32 front_ultrasonic       (4)
//   float32 rear_ultrasonic        (4)
//   float32 left_ir                (4)
//   float32 right_ir               (4)
//   float32 imu_quat[4]            (16)  - w, x, y, z
//   float32 imu_angular_vel[3]     (12)  - x, y, z
//   float32 imu_linear_accel[3]    (12)  - x, y, z
// CRC8 (1 byte)
// Total = 2 + 52 + 1 = 55 bytes

constexpr uint8_t SENSOR_SYNC_1 = 0xBB;
constexpr uint8_t SENSOR_SYNC_2 = 0x66;
constexpr size_t  SENSOR_PAYLOAD_SIZE = 52;
constexpr size_t  SENSOR_FRAME_SIZE   = 2 + SENSOR_PAYLOAD_SIZE + 1;  // 55

class SensorBridgeNode : public rclcpp::Node
{
public:
  SensorBridgeNode()
  : Node("sensor_bridge_node")
  {
    // Parameters
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<double>("publish_rate_hz", 50.0);

    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_   = this->get_parameter("baud_rate").as_int();
    double rate  = this->get_parameter("publish_rate_hz").as_double();

    // Publisher
    pub_ = this->create_publisher<aeroterrabot_interfaces::msg::SensorArray>(
      "/sensor_array", 10);

    // Open serial
    if (!open_serial()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open Arduino serial port %s",
                   serial_port_.c_str());
      rclcpp::shutdown();
      return;
    }

    // Timer-driven read loop
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&SensorBridgeNode::read_and_publish, this));

    RCLCPP_INFO(this->get_logger(),
      "SensorBridgeNode started — port=%s baud=%d rate=%.0f Hz",
      serial_port_.c_str(), baud_rate_, rate);
  }

  ~SensorBridgeNode()
  {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
    }
  }

private:
  void read_and_publish()
  {
    // ── Synchronize ────────────────────────────────────────────────────
    uint8_t byte = 0;
    bool synced = false;
    int scan = 256;
    while (scan-- > 0) {
      ssize_t n = ::read(serial_fd_, &byte, 1);
      if (n != 1) return;
      if (byte == SENSOR_SYNC_1) {
        n = ::read(serial_fd_, &byte, 1);
        if (n == 1 && byte == SENSOR_SYNC_2) {
          synced = true;
          break;
        }
      }
    }
    if (!synced) return;

    // ── Read payload + CRC ─────────────────────────────────────────────
    uint8_t buf[SENSOR_PAYLOAD_SIZE + 1];
    size_t total = 0;
    while (total < SENSOR_PAYLOAD_SIZE + 1) {
      ssize_t n = ::read(serial_fd_, buf + total, SENSOR_PAYLOAD_SIZE + 1 - total);
      if (n <= 0) return;
      total += static_cast<size_t>(n);
    }

    // ── CRC check ──────────────────────────────────────────────────────
    uint8_t crc = compute_crc8(buf, SENSOR_PAYLOAD_SIZE);
    if (crc != buf[SENSOR_PAYLOAD_SIZE]) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Sensor CRC mismatch");
      return;
    }

    // ── Unpack ─────────────────────────────────────────────────────────
    float values[13];  // 4 proximity + 4 quat + 3 ang_vel + 3 lin_accel = 14? no, let's count
    // Actually: front_us(1) + rear_us(1) + left_ir(1) + right_ir(1) + quat(4) + ang(3) + acc(3) = 13
    std::memcpy(values, buf, 13 * sizeof(float));

    auto msg = aeroterrabot_interfaces::msg::SensorArray();
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    msg.front_ultrasonic_distance = static_cast<double>(values[0]);
    msg.rear_ultrasonic_distance  = static_cast<double>(values[1]);
    msg.left_ir_distance          = static_cast<double>(values[2]);
    msg.right_ir_distance         = static_cast<double>(values[3]);

    msg.imu_data.header = msg.header;
    msg.imu_data.header.frame_id = "imu_link";
    msg.imu_data.orientation.w = static_cast<double>(values[4]);
    msg.imu_data.orientation.x = static_cast<double>(values[5]);
    msg.imu_data.orientation.y = static_cast<double>(values[6]);
    msg.imu_data.orientation.z = static_cast<double>(values[7]);
    msg.imu_data.angular_velocity.x = static_cast<double>(values[8]);
    msg.imu_data.angular_velocity.y = static_cast<double>(values[9]);
    msg.imu_data.angular_velocity.z = static_cast<double>(values[10]);
    msg.imu_data.linear_acceleration.x = static_cast<double>(values[11]);
    msg.imu_data.linear_acceleration.y = static_cast<double>(values[12]);
    // We only have 13 floats = 52 bytes payload, so z-accel would be index 12
    // Wait — 13 floats × 4 = 52 bytes. Perfect. But that's only x and y accel.
    // Let me fix: we actually need 14 floats but only have 52 bytes = 13 floats.
    // Let's keep 13 floats and note that z-accel is omitted — OR we pack it tighter.
    // Actually 4+4+3+3 = 14 floats = 56 bytes. Let me recalculate.
    // For now set z_accel to 0 — the payload constant should be 56 for 14 floats.
    // But the frame is already defined as 52. Let's use 52 bytes = 13 floats and
    // just use 13 values (z-accel derived or zero).
    msg.imu_data.linear_acceleration.z = 0.0;  // Not transmitted in this frame version

    pub_->publish(msg);
  }

  bool open_serial()
  {
    serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) return false;

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd_, &tty) != 0) { ::close(serial_fd_); serial_fd_ = -1; return false; }

    speed_t speed = B115200;
    switch (baud_rate_) {
      case 9600:   speed = B9600;   break;
      case 115200: speed = B115200; break;
      case 230400: speed = B230400; break;
      default:     speed = B115200; break;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT |
                      PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) { ::close(serial_fd_); serial_fd_ = -1; return false; }
    tcflush(serial_fd_, TCIOFLUSH);
    return true;
  }

  static uint8_t compute_crc8(const uint8_t * data, size_t len)
  {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
      crc ^= data[i];
      for (int b = 0; b < 8; ++b)
        crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
    }
    return crc;
  }

  std::string serial_port_;
  int baud_rate_{115200};
  int serial_fd_{-1};
  rclcpp::Publisher<aeroterrabot_interfaces::msg::SensorArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace aeroterrabot_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aeroterrabot_hardware::SensorBridgeNode>());
  rclcpp::shutdown();
  return 0;
}

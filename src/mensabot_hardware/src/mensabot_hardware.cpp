/**
 * @file mensabot_hardware.cpp
 * @brief ROS 2 hardware interface for the Mensabot platform.
 *
 * This file implements the ros2_control SystemInterface used to connect the
 * ROS 2 control framework with the Arduino-based motor controller via a
 * serial communication interface.
 *
 * Besides transmitting wheel velocity commands, the hardware interface
 * manages the communication state machine, heartbeat monitoring, emergency
 * stop handling and packet-based data exchange.
 */

#include "mensabot_hardware/mensabot_hardware.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace mensabot_hardware
{

// ============================================================================
// INITIALIZATION
// ============================================================================

hardware_interface::CallbackReturn MensabotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto port_it = info.hardware_parameters.find("port");
  if (port_it != info.hardware_parameters.end()) {
    port_ = port_it->second;
  }

  auto heartbeat_it = info.hardware_parameters.find("heartbeat_timeout");
  if (heartbeat_it != info.hardware_parameters.end()) {
    try {
      heartbeat_timeout_ = std::stod(heartbeat_it->second);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("mensabot_hardware"),
        "Invalid heartbeat_timeout parameter: %s. Using %.2f s.",
        e.what(), heartbeat_timeout_);
    }
  }

  auto send_period_it = info.hardware_parameters.find("send_period");
  if (send_period_it != info.hardware_parameters.end()) {
    try {
      send_period_ = std::stod(send_period_it->second);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("mensabot_hardware"),
        "Invalid send_period parameter: %s. Using %.3f s.",
        e.what(), send_period_);
    }
  }

  if (info.joints.size() < 2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("mensabot_hardware"),
      "At least two joints are required.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.assign(info.joints.size(), 0.0);
  hw_velocities_.assign(info.joints.size(), 0.0);
  hw_commands_.assign(info.joints.size(), 0.0);

  node_ = std::make_shared<rclcpp::Node>("mensabot_hardware");

  estop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/safety/estop",
    rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      estop_.store(msg->data);
    });

  connected_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/hardware/connected",
    rclcpp::QoS(10));

  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
  "/imu/data",
  rclcpp::QoS(10));

  serial_fd_ = ::open(
    port_.c_str(),
    O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (serial_fd_ < 0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Could not open serial port %s: %s",
      port_.c_str(), std::strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  struct termios tty{};

  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "tcgetattr() failed: %s", std::strerror(errno));
    ::close(serial_fd_);
    serial_fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cflag |= CLOCAL;
  tty.c_cflag |= CREAD;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "tcsetattr() failed: %s", std::strerror(errno));
    ::close(serial_fd_);
    serial_fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  tcflush(serial_fd_, TCIOFLUSH);

  connected_ = false;
  ready_ = false;
  active_ = false;
  estop_.store(false);
  estop_sent_ = false;
  feedback_received_ = false;
  time_since_last_message_ = 0.0;
  time_since_last_send_ = 0.0;
  rx_index_ = 0;
  rx_buffer_ = PacketBuffer{};

  RCLCPP_INFO(node_->get_logger(), "MensaBot hardware interface initialized.");
  RCLCPP_INFO(node_->get_logger(), "Serial port: %s", port_.c_str());
  RCLCPP_INFO(node_->get_logger(), "Baud rate: 115200");
  RCLCPP_INFO(node_->get_logger(), "Communication packet size: %zu bytes", sizeof(Packet));

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// STATE INTERFACES
// ============================================================================

std::vector<hardware_interface::StateInterface>
MensabotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "left_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_positions_[0]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "right_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_positions_[1]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]));

  return state_interfaces;
}

// ============================================================================
// COMMAND INTERFACES
// ============================================================================

std::vector<hardware_interface::CommandInterface>
MensabotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      "left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      "right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]));

  return command_interfaces;
}

// ============================================================================
// ACTIVATE
// ============================================================================

hardware_interface::CallbackReturn MensabotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Cannot activate hardware: serial port is not open.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_[0] = 0.0;
  hw_commands_[1] = 0.0;

  active_ = false;
  estop_sent_ = false;
  // Start both relative timers from zero.
  time_since_last_message_ = 0.0;
  time_since_last_send_ = 0.0;

  MotorCommand command{};
  command.velocity_L = 0;
  command.velocity_R = 0;

  send_packet(
    PKT_CMD,
    reinterpret_cast<const uint8_t *>(&command),
    sizeof(command));

  RCLCPP_INFO(node_->get_logger(), "MensaBot hardware activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// DEACTIVATE
// ============================================================================

hardware_interface::CallbackReturn MensabotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_ = false;
  hw_commands_[0] = 0.0;
  hw_commands_[1] = 0.0;

  if (serial_fd_ >= 0) {
    MotorCommand command{};
    command.velocity_L = 0;
    command.velocity_R = 0;
    send_packet(
      PKT_CMD,
      reinterpret_cast<const uint8_t *>(&command),
      sizeof(command));
  }

  RCLCPP_INFO(node_->get_logger(), "MensaBot hardware deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// CHECKSUM
// ============================================================================

uint16_t MensabotHardware::calculate_checksum(const Packet & packet)
{
  // Same checksum as the Arduino fixed 32-byte protocol:
  // XOR over TYPE and all 27 payload bytes.
  uint16_t checksum = 0;
  checksum ^= packet.type;

  for (size_t i = 0; i < PACKET_PAYLOAD_SIZE; ++i) {
    checksum ^= packet.payload[i];
  }

  return checksum;
}

// ============================================================================
// SEND PACKET
// ============================================================================

bool MensabotHardware::send_packet(
  uint8_t type,
  const uint8_t * payload,
  uint8_t length)
{
  if (serial_fd_ < 0) {
    return false;
  }

  if (length > PACKET_PAYLOAD_SIZE) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Packet payload too large: %u bytes. Maximum is %zu.",
      length, PACKET_PAYLOAD_SIZE);
    return false;
  }

  if (length > 0 && payload == nullptr) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Payload pointer is null although payload length is %u.",
      length);
    return false;
  }

  PacketBuffer tx{};
  tx.packet.header1 = PACKET_HEADER_1;
  tx.packet.header2 = PACKET_HEADER_2;
  tx.packet.type = type;

  if (length > 0) {
    std::memcpy(tx.packet.payload, payload, length);
  }

  // The remaining payload bytes stay zero because tx is value-initialized.
  tx.packet.checksum = calculate_checksum(tx.packet);

  const ssize_t written =
    ::write(serial_fd_, tx.bytes, sizeof(tx.bytes));

  if (written < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return false;
    }

    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Serial write failed: %s", std::strerror(errno));
    connected_ = false;
    ready_ = false;
    return false;
  }

  if (static_cast<size_t>(written) != sizeof(tx.bytes)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Incomplete serial write: %zd/%zu bytes.",
      written, sizeof(tx.bytes));
    return false;
  }

  if (type == PKT_PING) {
    //RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "ROS -> Arduino: PKT_PING");
  } else if (type == PKT_CMD && length == sizeof(MotorCommand)) {
    MotorCommand command{};
    std::memcpy(&command, payload, sizeof(command));
    //RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "ROS -> Arduino: PKT_CMD | left=%d (%.2f rad/s) | right=%d (%.2f rad/s)", command.velocity_L, static_cast<double>(command.velocity_L) / 100.0, command.velocity_R, static_cast<double>(command.velocity_R) / 100.0);
  } else if (type == PKT_ESTOP) {
    RCLCPP_WARN(node_->get_logger(), "ROS -> Arduino: PKT_ESTOP");
  } else if (type == PKT_RESET) {
    RCLCPP_INFO(node_->get_logger(), "ROS -> Arduino: PKT_RESET");
  }

  return true;
}

// ============================================================================
// READ PACKET
// ============================================================================

bool MensabotHardware::read_packet(Packet & packet)
{
  if (serial_fd_ < 0) {
    return false;
  }

  uint8_t byte = 0;

  // Same byte-wise synchronization scheme as the proven Arduino/old ROS
  // implementation. A packet is accepted only after all 32 bytes are received
  // and the checksum matches.
  while (::read(serial_fd_, &byte, sizeof(byte)) > 0) {
    switch (rx_index_) {
      case 0:
        if (byte != PACKET_HEADER_1) {
          continue;
        }
        rx_buffer_.bytes[rx_index_++] = byte;
        break;

      case 1:
        if (byte != PACKET_HEADER_2) {
          rx_index_ = 0;
          continue;
        }
        rx_buffer_.bytes[rx_index_++] = byte;
        break;

      default:
        rx_buffer_.bytes[rx_index_++] = byte;

        if (rx_index_ >= sizeof(Packet)) {
          rx_index_ = 0;

          const uint16_t checksum = calculate_checksum(rx_buffer_.packet);

          if (checksum != rx_buffer_.packet.checksum) {
            RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "Invalid packet checksum. Received: 0x%04X, calculated: 0x%04X.",
              rx_buffer_.packet.checksum, checksum);
            continue;
          }

          packet = rx_buffer_.packet;
          return true;
        }
        break;
    }
  }

  return false;
}

// ============================================================================
// PROCESS PACKET
// ============================================================================

void MensabotHardware::process_packet(const Packet & packet)
{
  time_since_last_message_ = 0.0;

  switch (packet.type) {
    case PKT_READY:
      RCLCPP_INFO(node_->get_logger(), "Arduino sent READY");
      connected_ = true;
      ready_ = true;
      break;

    case PKT_MOTOR_FEEDBACK:
    {
      MotorFeedback feedback{};
      std::memcpy(&feedback, packet.payload, sizeof(feedback));

      latest_feedback_ = feedback;
      feedback_received_ = true;

      hw_positions_[0] = static_cast<double>(feedback.position_L);
      hw_positions_[1] = static_cast<double>(feedback.position_R);
      hw_velocities_[0] = static_cast<double>(feedback.omega_L);
      hw_velocities_[1] = static_cast<double>(feedback.omega_R);

      connected_ = true;
      ready_ = true;

      //RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "Arduino -> ROS: PKT_MOTOR_FEEDBACK | pos_L=%.3f rad | pos_R=%.3f rad | omega_L=%.3f rad/s | omega_R=%.3f rad/s", hw_positions_[0], hw_positions_[1], hw_velocities_[0], hw_velocities_[1]);
      break;
    }

    case PKT_IMU_DATA:
    {
      ImuData imu{};
      std::memcpy(&imu, packet.payload, sizeof(imu));

      sensor_msgs::msg::Imu msg;

      msg.header.stamp = node_->get_clock()->now();
      msg.header.frame_id = "imu_link";

      msg.linear_acceleration.x = static_cast<double>(imu.accel_x);
      msg.angular_velocity.z = static_cast<double>(imu.gyro_z);

      imu_pub_->publish(msg);

      connected_ = true;
      ready_ = true;

      break;
    }

    case PKT_DEBUG_CMD:
    {
      DebugCmd debug_cmd{};
      std::memcpy(&debug_cmd, packet.payload, sizeof(debug_cmd));

      //RCLCPP_INFO(node_->get_logger(), "Arduino -> ROS: PKT_DEBUG_CMD | left=%d (%.2f rad/s) | right=%d (%.2f rad/s)", debug_cmd.left, static_cast<double>(debug_cmd.left) / 100.0, debug_cmd.right, static_cast<double>(debug_cmd.right) / 100.0);
      break;
    }

    default:
      break;
  }
}

// ============================================================================
// READ
// ============================================================================

hardware_interface::return_type MensabotHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  // Measure communication timeout relative to the previous read cycle.
  time_since_last_message_ += period.seconds();

  if (node_) {
    rclcpp::spin_some(node_);
  }

  Packet packet{};
  while (read_packet(packet)) {
    process_packet(packet);
  }

  if (time_since_last_message_ > heartbeat_timeout_) {
      if (connected_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "MensaBot controller communication timeout.");
      }

      connected_ = false;
      ready_ = false;
      active_ = false;
      feedback_received_ = false;
    }

  if (connected_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = connected_;
    connected_pub_->publish(msg);
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// WRITE
// ============================================================================

hardware_interface::return_type MensabotHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  // Measure send interval relative to the previous write cycle.
  time_since_last_send_ += period.seconds();
  if (serial_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  if (time_since_last_send_ < send_period_) {
    return hardware_interface::return_type::OK;
  }

  time_since_last_send_ = 0.0;

  // WAITING -> PING
  if (!connected_) {
    send_packet(PKT_PING, nullptr, 0);
    RCLCPP_INFO(node_->get_logger(), "Sending PING.");
    return hardware_interface::return_type::OK;
  }

  // ESTOP
  if (estop_.load()) {
    hw_commands_[0] = 0.0;
    hw_commands_[1] = 0.0;

    if (!estop_sent_) {
      RCLCPP_WARN(node_->get_logger(), "ESTOP active. Sending emergency stop.");
      estop_sent_ = true;
    }

    send_packet(PKT_ESTOP, nullptr, 0);
    active_ = false;
    return hardware_interface::return_type::OK;
  }

  // ESTOP released -> RESET
  if (estop_sent_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "ESTOP released. Resetting motor controller.");

    send_packet(PKT_RESET, nullptr, 0);
    estop_sent_ = false;
    ready_ = false;
    active_ = false;
    return hardware_interface::return_type::OK;
  }

  // Connected but not ready -> PING
  if (!ready_) {
    send_packet(PKT_PING, nullptr, 0);
    //RCLCPP_INFO(node_->get_logger(), "Sending PING.");
    return hardware_interface::return_type::OK;
  }

  // READY -> ACTIVE
  if (ready_ && !active_) {
    active_ = true;
  }

  // ROS wheel velocity [rad/s] -> Arduino int16 [rad/s * 100]
  MotorCommand command{};

  const double left_scaled =
    std::clamp(hw_commands_[0] * 100.0, -32768.0, 32767.0);
  const double right_scaled =
    std::clamp(hw_commands_[1] * 100.0, -32768.0, 32767.0);

  command.velocity_L = static_cast<int16_t>(std::lround(left_scaled));
  command.velocity_R = static_cast<int16_t>(std::lround(right_scaled));

  send_packet(
    PKT_CMD,
    reinterpret_cast<const uint8_t *>(&command),
    sizeof(command));

  return hardware_interface::return_type::OK;
}

}  // namespace mensabot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mensabot_hardware::MensabotHardware,
  hardware_interface::SystemInterface)

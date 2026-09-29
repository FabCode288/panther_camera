/**
 * @file mensabot_hardware.hpp
 * @brief Declaration of the MensaBot ros2_control hardware interface.
 */

#ifndef MENSABOT_HARDWARE__MENSABOT_HARDWARE_HPP_
#define MENSABOT_HARDWARE__MENSABOT_HARDWARE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include <sensor_msgs/msg/imu.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"

// ============================================================================
// COMMUNICATION PROTOCOL
// ============================================================================

constexpr uint8_t PACKET_HEADER_1 = 0xAA;
constexpr uint8_t PACKET_HEADER_2 = 0x55;
constexpr size_t PACKET_PAYLOAD_SIZE = 27;

#pragma pack(push, 1)

struct Packet
{
  uint8_t header1;
  uint8_t header2;
  uint8_t type;
  uint8_t payload[PACKET_PAYLOAD_SIZE];
  uint16_t checksum;
};

struct MotorCommand
{
  int16_t velocity_L;
  int16_t velocity_R;
};

struct MotorFeedback
{
  uint32_t timestamp_us;
  float position_L;
  float position_R;
  float omega_L;
  float omega_R;
};

struct ImuData
{
  float accel_x;
  float gyro_z;
};

struct DebugCmd
{
  int16_t left;
  int16_t right;
};

#pragma pack(pop)

static_assert(sizeof(Packet) == 32, "Packet size must be exactly 32 bytes");
static_assert(sizeof(MotorCommand) == 4, "MotorCommand size invalid");
static_assert(sizeof(MotorFeedback) == 20, "MotorFeedback size invalid");
static_assert(sizeof(DebugCmd) == 4, "DebugCmd size invalid");

union PacketBuffer
{
  Packet packet;
  uint8_t bytes[sizeof(Packet)];
};

enum PacketType : uint8_t
{
  PKT_PING           = 1,
  PKT_READY          = 2,
  PKT_HB             = 3,
  PKT_CMD            = 10,
  PKT_ESTOP          = 20,
  PKT_RESET          = 21,
  PKT_MOTOR_FEEDBACK = 30,
  PKT_IMU_DATA       = 31,
  PKT_DEBUG_CMD      = 40
};

namespace mensabot_hardware
{

class MensabotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MensabotHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  int serial_fd_ = -1;
  std::string port_ = "/dev/ttyACM0";

  PacketBuffer rx_buffer_{};
  size_t rx_index_ = 0;

  bool send_packet(
    uint8_t type,
    const uint8_t * payload = nullptr,
    uint8_t length = 0);

  bool read_packet(Packet & packet);

  uint16_t calculate_checksum(const Packet & packet);

  void process_packet(const Packet & packet);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  MotorFeedback latest_feedback_{};
  bool feedback_received_ = false;

  bool connected_ = false;
  bool ready_ = false;
  bool active_ = false;
  std::atomic<bool> estop_{false};
  bool estop_sent_ = false;

  // Timing is measured only relative to the previous read/write cycle.
  // No subtraction of rclcpp::Time objects is used.
  double time_since_last_message_ = 0.0;
  double time_since_last_send_ = 0.0;

  double heartbeat_timeout_ = 0.5;
  double send_period_ = 0.02;
};

}  // namespace mensabot_hardware

#endif  // MENSABOT_HARDWARE__MENSABOT_HARDWARE_HPP_
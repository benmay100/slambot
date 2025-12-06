#ifndef SLAMBOT_HARDWARE__SLAMBOT_SYSTEM_HPP_
#define SLAMBOT_HARDWARE__SLAMBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// ROS 2 Messages for Micro-ROS communication
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace slambot_hardware
{
class SlambotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SlambotSystemHardware)

  // Lifecycle Node Interface standard methods
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Hardware Interface standard methods
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Micro-ROS communication node
  std::shared_ptr<rclcpp::Node> node_;

  // Publishers and Subscribers
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Data storage for ROS 2 control to read from/write to
  // Order: FL, FR, RL, RR
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  // IMU Data storage
  // Orientation (x, y, z, w), Angular Vel (x, y, z), Linear Accel (x, y, z)
  std::vector<double> hw_imu_orientation_;     // 4 elements
  std::vector<double> hw_imu_angular_velocity_;// 3 elements
  std::vector<double> hw_imu_linear_acceleration_; // 3 elements

  // Callbacks for data received from Pico
  void wheel_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
};

}  // namespace slambot_hardware

#endif  // SLAMBOT_HARDWARE__SLAMBOT_SYSTEM_HPP_
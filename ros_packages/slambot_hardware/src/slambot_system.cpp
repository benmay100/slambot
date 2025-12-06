#include "slambot_hardware/slambot_system.hpp" 

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// This line is needed to register this class as a plugin
#include "pluginlib/class_list_macros.hpp"


namespace slambot_hardware
{
// Set up convenient shortcuts for Lifecycle node return values
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


// This is the main constructor. It's called when the class is loaded.
CallbackReturn SlambotSystemHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // 1. Call the base class on_init using the new params
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // 2. The base class has now populated the 'info_' member variable for us.
  // We can continue using 'info_' just as before.

  node_ = std::make_shared<rclcpp::Node>("slambot_hardware_interface");

  // Use the clear variable name you suggested
  hw_commands_velocity_.resize(info_.joints.size(), 0.0);
  hw_states_position_.resize(info_.joints.size(), 0.0);
  hw_states_velocity_.resize(info_.joints.size(), 0.0);
  
  hw_imu_orientation_.resize(4, 0.0);
  hw_imu_angular_velocity_.resize(3, 0.0);
  hw_imu_linear_acceleration_.resize(3, 0.0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    RCLCPP_INFO(node_->get_logger(), "Found joint: '%s'", joint.name.c_str());
  }
  for (const hardware_interface::ComponentInfo & sensor : info_.sensors)
  {
    RCLCPP_INFO(node_->get_logger(), "Found sensor: '%s'", sensor.name.c_str());
  }

  return CallbackReturn::SUCCESS;
}

// Added missing lifecycle functions
CallbackReturn SlambotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Configuring Slambot hardware interface...");
  // Nothing to configure for now
  RCLCPP_INFO(node_->get_logger(), "Hardware interface configured.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SlambotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up Slambot hardware interface...");
  // Reset data storage
  hw_commands_velocity_.assign(hw_commands_velocity_.size(), 0.0);
  hw_states_position_.assign(hw_states_position_.size(), 0.0);
  hw_states_velocity_.assign(hw_states_velocity_.size(), 0.0);
  hw_imu_orientation_.assign(hw_imu_orientation_.size(), 0.0);
  hw_imu_angular_velocity_.assign(hw_imu_angular_velocity_.size(), 0.0);
  hw_imu_linear_acceleration_.assign(hw_imu_linear_acceleration_.size(), 0.0);
  RCLCPP_INFO(node_->get_logger(), "Hardware interface cleaned up.");
  return CallbackReturn::SUCCESS;
}


// This function is called by ros2_control to get all available "state" interfaces.
std::vector<hardware_interface::StateInterface> SlambotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // --- Export Wheel Joint States ---
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }

  // --- Export IMU Sensor States ---
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.x", &hw_imu_orientation_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.y", &hw_imu_orientation_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.z", &hw_imu_orientation_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.w", &hw_imu_orientation_[3]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "angular_velocity.x", &hw_imu_angular_velocity_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "angular_velocity.y", &hw_imu_angular_velocity_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "angular_velocity.z", &hw_imu_angular_velocity_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "linear_acceleration.x", &hw_imu_linear_acceleration_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "linear_acceleration.y", &hw_imu_linear_acceleration_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "linear_acceleration.z", &hw_imu_linear_acceleration_[2]));

  return state_interfaces;
}


// This function is called by ros2_control to get all available "command" interfaces.
std::vector<hardware_interface::CommandInterface> SlambotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // We only accept "velocity" commands for our 4 wheel joints.
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
  }

  return command_interfaces;
}


// This function is called when the hardware interface is started.
CallbackReturn SlambotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Activating Slambot hardware interface...");

  // Set all commands to 0 on activation
  for (auto & command : hw_commands_velocity_)
  {
    command = 0.0;
  }

  // Create publishers and subscribers
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  
  wheel_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_commands", qos);
  wheel_state_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/wheel_states", 10, std::bind(&SlambotSystemHardware::wheel_state_callback, this, std::placeholders::_1));
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_raw", 10, std::bind(&SlambotSystemHardware::imu_callback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "Hardware interface activated. Ready to control.");
  return CallbackReturn::SUCCESS;
}


// This function is called when the hardware interface is stopped.
CallbackReturn SlambotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating Slambot hardware interface...");

  // Stop all motors
  auto stop_msg = std_msgs::msg::Float32MultiArray();
  stop_msg.data = {0.0, 0.0, 0.0, 0.0};
  wheel_cmd_pub_->publish(stop_msg);
  
  // Reset publishers/subscribers
  wheel_cmd_pub_.reset();
  wheel_state_sub_.reset();
  imu_sub_.reset();
  
  RCLCPP_INFO(node_->get_logger(), "Hardware interface deactivated.");
  return CallbackReturn::SUCCESS;
}


// This is called by ros2_control *before* its update loop.
hardware_interface::return_type SlambotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Spin our internal node to process any waiting messages from the Pico
  rclcpp::spin_some(node_);
  return hardware_interface::return_type::OK;
}


// This is called by ros2_control *after* its update loop.
hardware_interface::return_type SlambotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto cmd_msg = std_msgs::msg::Float32MultiArray();
  cmd_msg.data.resize(4);

  // NO CONVERSION. Pass rad/s commands directly to Pico.
  cmd_msg.data[0] = (float)hw_commands_velocity_[0]; // front_left_wheel_joint
  cmd_msg.data[1] = (float)hw_commands_velocity_[1]; // front_right_wheel_joint
  cmd_msg.data[2] = (float)hw_commands_velocity_[2]; // rear_left_wheel_joint
  cmd_msg.data[3] = (float)hw_commands_velocity_[3]; // rear_right_wheel_joint

  wheel_cmd_pub_->publish(cmd_msg);

  return hardware_interface::return_type::OK;
}


// --- CALLBACKS ---
// These functions are triggered whenever a message is received from the Pico.

// Callback for /wheel_states topic
void SlambotSystemHardware::wheel_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 8) {
    RCLCPP_WARN(node_->get_logger(), "Wheel state message has incorrect size: %zu", msg->data.size());
    return;
  }
  
  // NO CONVERSION. Data from Pico is already in rad/s and rad.
  // Data: [vel_fl, vel_fr, vel_rl, vel_rr, pos_fl, pos_fr, pos_rl, pos_rr]
  
  // Velocities (rad/s)
  hw_states_velocity_[0] = msg->data[0]; // fl
  hw_states_velocity_[1] = msg->data[1]; // fr
  hw_states_velocity_[2] = msg->data[2]; // rl
  hw_states_velocity_[3] = msg->data[3]; // rr

  // Positions (rad)
  hw_states_position_[0] = msg->data[4]; // fl
  hw_states_position_[1] = msg->data[5]; // fr
  hw_states_position_[2] = msg->data[6]; // rl
  hw_states_position_[3] = msg->data[7]; // rr
}

// Callback for /imu/data_raw topic
void SlambotSystemHardware::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Direct copy, as IMU data is already in standard SI units
  hw_imu_orientation_[0] = msg->orientation.x;
  hw_imu_orientation_[1] = msg->orientation.y;
  hw_imu_orientation_[2] = msg->orientation.z;
  hw_imu_orientation_[3] = msg->orientation.w;

  // Angular Velocity
  hw_imu_angular_velocity_[0] = msg->angular_velocity.x;
  hw_imu_angular_velocity_[1] = msg->angular_velocity.y;
  hw_imu_angular_velocity_[2] = msg->angular_velocity.z;

  // Linear Acceleration
  hw_imu_linear_acceleration_[0] = msg->linear_acceleration.x;
  hw_imu_linear_acceleration_[1] = msg->linear_acceleration.y;
  hw_imu_linear_acceleration_[2] = msg->linear_acceleration.z;
}

}  // namespace slambot_hardware


// This is the magic line that registers this class as a plugin.
// The name "slambot_hardware/SlambotHardwareInterface" MUST match what we put in the XML file.
PLUGINLIB_EXPORT_CLASS(
  slambot_hardware::SlambotSystemHardware,
  hardware_interface::SystemInterface)
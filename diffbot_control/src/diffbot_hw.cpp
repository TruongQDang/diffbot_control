#include "diffbot_control/diffbot_hw.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffbot_system_hw
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  wheel_left_.name = info_.hardware_parameters["left_wheel_name"];
  wheel_right_.name = info_.hardware_parameters["right_wheel_name"];
  encoder_param_.angle_resolution = std::stod(info_.hardware_parameters["angle_resolution"]);
  encoder_param_.encoder_counts_per_rev = std::stoi(info_.hardware_parameters["encoder_counts_per_rev"]);
  cmd_vel_threshold_ = std::stod(info_.hardware_parameters["cmd_vel_diff_threshold"]);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  node_ = rclcpp::Node::make_shared("control_comm_node");

  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  wheel_vel_pub_ = node_->create_publisher<diffbot_msgs::msg::WheelVel>("happi_robot/cmd_wheel_vel", qos);

  rclcpp::QoS qos_profile(10);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  auto wheel_vel_callback =
      [this](diffbot_msgs::msg::WheelVel::UniquePtr wheel_vel_msg) -> void
  {
    latest_wheel_vel_ = *wheel_vel_msg;
  };
  wheel_vel_sub_ =
      node_->create_subscription<diffbot_msgs::msg::WheelVel>(
          "happi_robot/wheel_vel",
          qos_profile,
          wheel_vel_callback);

  auto wheel_pos_callback =
      [this](diffbot_msgs::msg::WheelPos::UniquePtr wheel_pos_msg) -> void
  {
    latest_wheel_pos_ = *wheel_pos_msg;
  };
  wheel_pos_sub_ =
      node_->create_subscription<diffbot_msgs::msg::WheelPos>(
          "happi_robot/wheel_pos",
          qos_profile,
          wheel_pos_callback);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_left_.name, hardware_interface::HW_IF_POSITION, &wheel_left_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_left_.name, hardware_interface::HW_IF_VELOCITY, &wheel_left_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_right_.name, hardware_interface::HW_IF_POSITION, &wheel_right_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_right_.name, hardware_interface::HW_IF_VELOCITY, &wheel_right_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_left_.name, hardware_interface::HW_IF_VELOCITY, &wheel_left_.cmd_vel));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_right_.name, hardware_interface::HW_IF_VELOCITY, &wheel_right_.cmd_vel));

  return command_interfaces;
}

hardware_interface::return_type DiffBotSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (rclcpp::ok())
  {
    rclcpp::spin_some(node_);
  }

  // if (first_measurement_) {
  //   wheel_left_.encoder_step = latest_wheel_pos_.left_wheel_pos;
  //   wheel_right_.encoder_step = latest_wheel_pos_.right_wheel_pos;
  //   wheel_left_.pos = 0.0;
  //   wheel_right_.pos = 0.0;
  //   first_measurement_ = false;
  //   return hardware_interface::return_type::OK;
  // }

  
  // // convert from step number to wheel position in radian (also handle wrap-around)
  // auto left_encoder_diff = std::abs(latest_wheel_pos_.left_wheel_pos - wheel_left_.encoder_step) % encoder_param_.encoder_counts_per_rev;
  // auto right_encoder_diff = std::abs(latest_wheel_pos_.right_wheel_pos - wheel_right_.encoder_step) % encoder_param_.encoder_counts_per_rev;
  
  // wheel_left_.pos = latest_wheel_vel_.left_wheel_vel >-= 0 
  // wheel_left_.pos = latest_wheel_pos_.left_wheel_pos -  * encoder_param_.angle_resolution;
  // wheel_right_.pos = latest_wheel_pos_.right_wheel_pos * encoder_param_.angle_resolution;
  wheel_left_.pos = latest_wheel_pos_.left_wheel_pos * encoder_param_.angle_resolution;
  wheel_right_.pos = latest_wheel_pos_.right_wheel_pos * encoder_param_.angle_resolution;
  // convert from step/s to rad/s
  wheel_left_.vel = latest_wheel_vel_.left_wheel_vel * encoder_param_.angle_resolution;
  wheel_right_.vel = latest_wheel_vel_.right_wheel_vel * encoder_param_.angle_resolution;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto left_diff = std::abs(wheel_left_.cmd_vel - wheel_left_.vel);
  const auto right_diff = std::abs(wheel_right_.cmd_vel - wheel_right_.vel);
  if (left_diff <= cmd_vel_threshold_ && right_diff <= cmd_vel_threshold_) {
    return hardware_interface::return_type::OK;
  }

  diffbot_msgs::msg::WheelVel wheel_vel_cmd; // step/s
  wheel_vel_cmd.left_wheel_vel = wheel_left_.cmd_vel / encoder_param_.angle_resolution;
  wheel_vel_cmd.right_wheel_vel = wheel_right_.cmd_vel / encoder_param_.angle_resolution;

  if (rclcpp::ok())
  {
    wheel_vel_pub_->publish(wheel_vel_cmd);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace diffbot_system_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffbot_system_hw::DiffBotSystemHardware, hardware_interface::SystemInterface)
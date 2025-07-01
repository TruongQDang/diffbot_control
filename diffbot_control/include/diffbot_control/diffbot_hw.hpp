#ifndef DIFFBOT_SYSTEM_HARDWARE
#define DIFFBOT_SYSTEM_HARDWARE

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "diffbot_msgs/msg/wheel_pos.hpp"
#include "diffbot_msgs/msg/wheel_vel.hpp"

namespace diffbot_system_hw
{
class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<diffbot_msgs::msg::WheelVel>::SharedPtr wheel_vel_pub_;
  rclcpp::Subscription<diffbot_msgs::msg::WheelPos>::SharedPtr wheel_pos_sub_;
  rclcpp::Subscription<diffbot_msgs::msg::WheelVel>::SharedPtr wheel_vel_sub_;

  struct Wheel
  {
    std::string name = "";
    int encoder_step;
    double pos;
    double vel;
    double cmd_vel;
  };

  Wheel wheel_left_;
  Wheel wheel_right_;
  
  struct Encoder
  {
    double angle_resolution; // [rad/step]
    int encoder_counts_per_rev;
  };

  Encoder encoder_param_;

  diffbot_msgs::msg::WheelPos latest_wheel_pos_; // [step number]
  diffbot_msgs::msg::WheelVel latest_wheel_vel_; // [steps/s]

  double cmd_vel_threshold_; // rad/s

  bool first_measurement_ = true;
};

}  // namespace diffbot_system_hw

#endif  // DIFFBOT_SYSTEM_HARDWARE

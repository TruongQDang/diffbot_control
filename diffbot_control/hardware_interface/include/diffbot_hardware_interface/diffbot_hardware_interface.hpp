#ifndef DIFFBOT_HARDWARE_INTERFACE
#define DIFFBOT_HARDWARE_INTERFACE

#include <string>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "diffbot_interfaces/msg/motor_pos.hpp"
#include "diffbot_interfaces/msg/motor_vel.hpp"

namespace diffbot_hardware_interface
{
class DiffbotHardwareInterface : public hardware_interface::SystemInterface
{
      public:
	hardware_interface::CallbackReturn
	on_init(const hardware_interface::HardwareInfo &info) override;

	hardware_interface::return_type read(const rclcpp::Time &time,
	                                     const rclcpp::Duration &period) override;

	hardware_interface::return_type write(const rclcpp::Time &time,
	                                      const rclcpp::Duration &period) override;

	std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

	std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      private:
	rclcpp::Node::SharedPtr node_;
	rclcpp::Publisher<diffbot_interfaces::msg::MotorVel>::SharedPtr motor_vel_pub_;
	rclcpp::Subscription<diffbot_interfaces::msg::MotorPos>::SharedPtr motor_pos_sub_;
	rclcpp::Subscription<diffbot_interfaces::msg::MotorVel>::SharedPtr motor_vel_sub_;

	struct Motor {
		std::string name = "";
		double pos;
		double vel;
		double cmd_vel;
	};

	Motor motor_left_;
	Motor motor_right_;

	struct Encoder {
		double angle_resolution; // [rad/step]
		int encoder_counts_per_rev;
	};

	Encoder encoder_param_;

	diffbot_interfaces::msg::MotorPos latest_motor_pos_; // [step number]
	diffbot_interfaces::msg::MotorVel latest_motor_vel_; // [steps/s]

	double cmd_vel_threshold_; // rad/s

	bool first_measurement_ = true;
};

} // namespace diffbot_hardware_interface

#endif // DIFFBOT_HARDWARE_INTERFACE

#include "diffbot_hardware_interface/diffbot_hardware_interface.hpp"

#include <cmath>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace diffbot_hardware_interface
{
hardware_interface::CallbackReturn
DiffbotHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
	if (hardware_interface::SystemInterface::on_init(info)
	    != hardware_interface::CallbackReturn::SUCCESS) {
		return hardware_interface::CallbackReturn::ERROR;
	}

	motor_left_.name = info_.hardware_parameters["left_wheel_name"];
	motor_right_.name = info_.hardware_parameters["right_wheel_name"];
	encoder_param_.angle_resolution = std::stod(info_.hardware_parameters["angle_resolution"]);
	encoder_param_.encoder_counts_per_rev
	    = std::stoi(info_.hardware_parameters["encoder_counts_per_rev"]);
	cmd_vel_threshold_ = std::stod(info_.hardware_parameters["cmd_vel_diff_threshold"]);

	for (const hardware_interface::ComponentInfo &joint : info_.joints) {
		if (joint.command_interfaces.size() != 1) {
			RCLCPP_FATAL(get_logger(),
			             "Joint '%s' has %zu command "
			             "interfaces "
			             "found. 1 expected.",
			             joint.name.c_str(),
			             joint.command_interfaces.size());
			return hardware_interface::CallbackReturn::ERROR;
		}

		if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
			RCLCPP_FATAL(get_logger(),
			             "Joint '%s' have %s command "
			             "interfaces "
			             "found. '%s' expected.",
			             joint.name.c_str(),
			             joint.command_interfaces[0].name.c_str(),
			             hardware_interface::HW_IF_VELOCITY);
			return hardware_interface::CallbackReturn::ERROR;
		}

		if (joint.state_interfaces.size() != 2) {
			RCLCPP_FATAL(get_logger(),
			             "Joint '%s' has %zu state "
			             "interface. 2 expected.",
			             joint.name.c_str(),
			             joint.state_interfaces.size());
			return hardware_interface::CallbackReturn::ERROR;
		}

		if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
			RCLCPP_FATAL(get_logger(),
			             "Joint '%s' have '%s' as first "
			             "state "
			             "interface. '%s' expected.",
			             joint.name.c_str(),
			             joint.state_interfaces[0].name.c_str(),
			             hardware_interface::HW_IF_POSITION);
			return hardware_interface::CallbackReturn::ERROR;
		}

		if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
			RCLCPP_FATAL(get_logger(),
			             "Joint '%s' have '%s' as "
			             "second state "
			             "interface. '%s' expected.",
			             joint.name.c_str(),
			             joint.state_interfaces[1].name.c_str(),
			             hardware_interface::HW_IF_VELOCITY);
			return hardware_interface::CallbackReturn::ERROR;
		}
	}

	node_ = rclcpp::Node::make_shared("control_comm_node");

	auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
	qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	motor_vel_pub_
	    = node_->create_publisher<diffbot_interfaces::msg::MotorVel>("diffbot/cmd_motor_vel",
	                                                                 qos);

	rclcpp::QoS qos_profile(10);
	qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
	auto motor_vel_callback
	    = [this](diffbot_interfaces::msg::MotorVel::UniquePtr motor_vel_msg) -> void {
		latest_motor_vel_ = *motor_vel_msg;
	};
	motor_vel_sub_
	    = node_->create_subscription<diffbot_interfaces::msg::MotorVel>("diffbot/motor_vel",
	                                                                    qos_profile,
	                                                                    motor_vel_callback);

	auto motor_pos_callback
	    = [this](diffbot_interfaces::msg::MotorPos::UniquePtr motor_pos_msg) -> void {
		latest_motor_pos_ = *motor_pos_msg;
	};
	motor_pos_sub_
	    = node_->create_subscription<diffbot_interfaces::msg::MotorPos>("diffbot/motor_pos",
	                                                                    qos_profile,
	                                                                    motor_pos_callback);

	return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DiffbotHardwareInterface::export_state_interfaces()
{
	std::vector<hardware_interface::StateInterface> state_interfaces;

	state_interfaces.emplace_back(
	    hardware_interface::StateInterface(motor_left_.name,
	                                       hardware_interface::HW_IF_POSITION,
	                                       &motor_left_.pos));
	state_interfaces.emplace_back(
	    hardware_interface::StateInterface(motor_left_.name,
	                                       hardware_interface::HW_IF_VELOCITY,
	                                       &motor_left_.vel));

	state_interfaces.emplace_back(
	    hardware_interface::StateInterface(motor_right_.name,
	                                       hardware_interface::HW_IF_POSITION,
	                                       &motor_right_.pos));
	state_interfaces.emplace_back(
	    hardware_interface::StateInterface(motor_right_.name,
	                                       hardware_interface::HW_IF_VELOCITY,
	                                       &motor_right_.vel));

	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DiffbotHardwareInterface::export_command_interfaces()
{
	std::vector<hardware_interface::CommandInterface> command_interfaces;

	command_interfaces.emplace_back(
	    hardware_interface::CommandInterface(motor_left_.name,
	                                         hardware_interface::HW_IF_VELOCITY,
	                                         &motor_left_.cmd_vel));

	command_interfaces.emplace_back(
	    hardware_interface::CommandInterface(motor_right_.name,
	                                         hardware_interface::HW_IF_VELOCITY,
	                                         &motor_right_.cmd_vel));

	return command_interfaces;
}

hardware_interface::return_type
DiffbotHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
	if (rclcpp::ok()) {
		rclcpp::spin_some(node_);
	}

	motor_left_.pos = latest_motor_pos_.left_motor_pos * encoder_param_.angle_resolution;
	motor_right_.pos = latest_motor_pos_.right_motor_pos * encoder_param_.angle_resolution;
	// convert from step/s to rad/s
	motor_left_.vel = latest_motor_vel_.left_motor_vel * encoder_param_.angle_resolution;
	motor_right_.vel = latest_motor_vel_.right_motor_vel * encoder_param_.angle_resolution;

	return hardware_interface::return_type::OK;
}

hardware_interface::return_type
DiffbotHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
	const auto left_diff = std::abs(motor_left_.cmd_vel - motor_left_.vel);
	const auto right_diff = std::abs(motor_right_.cmd_vel - motor_right_.vel);
	if (left_diff <= cmd_vel_threshold_ && right_diff <= cmd_vel_threshold_) {
		return hardware_interface::return_type::OK;
	}

	diffbot_interfaces::msg::MotorVel motor_vel_cmd; // step/s
	motor_vel_cmd.left_motor_vel = motor_left_.cmd_vel / encoder_param_.angle_resolution;
	motor_vel_cmd.right_motor_vel = motor_right_.cmd_vel / encoder_param_.angle_resolution;

	if (rclcpp::ok()) {
		motor_vel_pub_->publish(motor_vel_cmd);
	}

	return hardware_interface::return_type::OK;
}

} // namespace diffbot_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffbot_hardware_interface::DiffbotHardwareInterface,
                       hardware_interface::SystemInterface)
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "diffbot_msgs/msg/wheel_pos.hpp"
#include "diffbot_msgs/msg/wheel_vel.hpp"

using namespace std::chrono_literals;

class CommNode : public rclcpp::Node
{
public:
        CommNode()
        : Node("diffbot_comm_node")
        {
                auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
                qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); 
                wheel_vel_pub_ = this->create_publisher<diffbot_msgs::msg::WheelVel>("happi_robot/cmd_wheel_vel", qos);
                auto timer_callback =
                        [this]() -> void
                {
                       auto wheel_vel_cmd = diffbot_msgs::msg::WheelVel();
                       wheel_vel_cmd.left_wheel_vel = 1000;
                       wheel_vel_cmd.right_wheel_vel = 3000;
                       this->wheel_vel_pub_->publish(wheel_vel_cmd);
                };
                timer_ = this->create_wall_timer(100ms, timer_callback);

                rclcpp::QoS qos_profile(10);
                qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
                auto wheel_vel_callback =
                        [this](diffbot_msgs::msg::WheelVel::UniquePtr wheel_vel_msg) -> void
                {
                        RCLCPP_INFO(this->get_logger(), "left vel: %d", wheel_vel_msg->left_wheel_vel);
                };
                wheel_vel_sub_ =
                        this->create_subscription<diffbot_msgs::msg::WheelVel>(
                                "happi_robot/wheel_vel", 
                                qos_profile, 
                                wheel_vel_callback);

                auto wheel_pos_callback =
                        [this](diffbot_msgs::msg::WheelPos::UniquePtr wheel_pos_msg) -> void
                {
                        RCLCPP_INFO(this->get_logger(), "left pos: %d", wheel_pos_msg->left_wheel_pos);
                };
                wheel_pos_sub_ =
                        this->create_subscription<diffbot_msgs::msg::WheelPos>(
                                "happi_robot/wheel_pos", 
                                qos_profile, 
                                wheel_pos_callback);
        }

private:
        rclcpp::Publisher<diffbot_msgs::msg::WheelVel>::SharedPtr wheel_vel_pub_;
        rclcpp::Subscription<diffbot_msgs::msg::WheelPos>::SharedPtr wheel_pos_sub_;
        rclcpp::Subscription<diffbot_msgs::msg::WheelVel>::SharedPtr wheel_vel_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
};


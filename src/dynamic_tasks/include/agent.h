#pragma once

#include "vec.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "dynamic_interfaces/srv/set_targets.hpp"

class Agent : public rclcpp::Node
{
public:
	Agent(const std::string &name);


private:
	void position_callback(const geometry_msgs::msg::Pose &pose);
	void target_callback(const geometry_msgs::msg::Pose &pose);
    void set_targets_callback(const std::shared_ptr<dynamic_interfaces::srv::SetTargets::Request>& request);
    void timer_callback();

    void update_target();

	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriptionAgent_;
	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriptionTarget_;
    rclcpp::Service<dynamic_interfaces::srv::SetTargets>::SharedPtr setTargetService_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_;
	rclcpp::TimerBase::SharedPtr timer_;

	std::optional<Vec> agentPos;
	std::optional<Vec> currentTargetPos;
    std::optional<std::string> currentTargetName;
    bool assignedTarget;
	std::mutex mutex;
    std::string name;

    std::queue<std::string> remainingTargets;
    std::vector<std::string> completedTargets;
};
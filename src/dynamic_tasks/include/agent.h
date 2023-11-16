#pragma once

#include "vec.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Agent : public rclcpp::Node
{
public:
	Agent(const std::string &name);

    void add_target(const std::string &name);

private:
	void position_callback(const geometry_msgs::msg::Pose &pose);
	void target_callback(const geometry_msgs::msg::Pose &pose);
    void timer_callback();

	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriptionAgent_;
	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriptionTarget_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_;
	rclcpp::TimerBase::SharedPtr timer_;

	std::optional<Vec> agentPos;
	std::optional<Vec> currentTargetPos;
    bool assignedPos;
	std::mutex mutex;
    std::string name;

    std::queue<std::string> remainingTargets;
};
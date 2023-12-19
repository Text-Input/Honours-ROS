#pragma once

#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "vec.h"

struct AgentSimInfo {
	Vec position;
	Vec speed;
};

class Simulation : public rclcpp::Node {
public:
	Simulation();

private:
	void agentTwistCallback(const geometry_msgs::msg::Twist &twist, const std::string &agent);

	void genWorld();
	void run();

	std::mutex mutex;

	std::map<std::string, Vec> targets;
	std::map<std::string, AgentSimInfo> agents;

	rclcpp::TimerBase::SharedPtr timer_;

	std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr> agentsPosePublisher_;
	std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr> targetsPosePublisher_;
	std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> agentsTwistSubscriber_;
};

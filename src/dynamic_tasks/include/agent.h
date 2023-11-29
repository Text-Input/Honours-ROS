#pragma once

#include "vec.h"
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "dynamic_interfaces/srv/set_targets.hpp"
#include "dynamic_interfaces/msg/agent_target_state.hpp"


class Agent : public rclcpp::Node
{
public:
	Agent(int agentNum);


private:
    void position_callback(const geometry_msgs::msg::Pose &pose);
	void target_callback(const geometry_msgs::msg::Pose &pose);
    void set_targets_callback(const std::shared_ptr<dynamic_interfaces::srv::SetTargets::Request>& request);
    void timer_callback();

    void agent_pose_callback(geometry_msgs::msg::Pose pose, int agentNum);
    void agent_cmd_vel_callback(geometry_msgs::msg::Twist, int agentNum);

    void update_target();
    void send_target_state();
    Vec collision_avoidance(Vec desiredVelocity);

	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriptionAgent_;
	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriptionTarget_;
    rclcpp::Service<dynamic_interfaces::srv::SetTargets>::SharedPtr setTargetService_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_;
    rclcpp::Publisher<dynamic_interfaces::msg::AgentTargetState>::SharedPtr targetState_;
	rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> otherAgentsPose_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> otherAgentsCmdVel_;

	std::optional<Vec> agentPos;
	std::optional<Vec> currentTargetPos;
    std::optional<std::string> currentTargetName;
    bool assignedTarget;
	std::mutex mutex;
    std::string name;
    int number;

    std::deque<std::string> remainingTargets;
    std::vector<std::string> completedTargets;

    struct AgentInfo
    {
        Vec position;
        Vec velocity;
    };

    std::map<int, AgentInfo> otherAgentsInfo_;
};
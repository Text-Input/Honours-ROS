#pragma once

#include "common.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <atomic>

#include "dynamic_interfaces/msg/world_info.hpp"
#include "dynamic_interfaces/srv/set_targets.hpp"

class TaskAllocator : public rclcpp::Node {
public:
    TaskAllocator();

private:
    void assignTargets();

    void targetCallback(const geometry_msgs::msg::Pose &poseMsg, const std::string &target);
    void agentCallback(const geometry_msgs::msg::Pose &poseMsg, const std::string &agent);
    void worldCallback(const dynamic_interfaces::msg::WorldInfo &worldInfo);

    bool dataInitialized;
    std::mutex mutex;
    std::map<std::string, TargetInfo> targets;
    std::map<std::string, AgentInfo> agents;
    std::set<std::string> assignedTargets;
    std::map<std::string, std::vector<std::string>> agentAssignment;

    std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> targetsSubscriptions_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> agentsSubscriptions_;
    rclcpp::Subscription<dynamic_interfaces::msg::WorldInfo>::SharedPtr worldSubscription_;
    std::map<std::string, rclcpp::Client<dynamic_interfaces::srv::SetTargets>::SharedPtr> agentsTargetSet_;

//    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_;
};
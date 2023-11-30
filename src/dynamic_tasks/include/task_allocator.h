#pragma once

#include "common.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <atomic>

#include "dynamic_interfaces/msg/world_info.hpp"
#include "dynamic_interfaces/msg/agent_target_state.hpp"
#include "dynamic_interfaces/srv/set_targets.hpp"

enum class DynamicAlgs {
    Simple,
    MinimizeTime,
    MinimizeTimeV2
};

using AgentAllocation = std::map<std::string, std::vector<std::string>>;

struct SystemState {
    AgentAllocation currentAllocation;
    std::map<std::string, TargetInfo> targets;
    std::set<std::string> assignedTargets;
    std::map<std::string, AgentInfo> agents;
};

struct AllocationResult {
   AgentAllocation newAllocation;
   std::set<std::string> newAssignedTargets;
};

class TaskAllocator : public rclcpp::Node {
public:
    TaskAllocator(DynamicAlgs dynamicAlgs);

private:

    void targetCallback(const geometry_msgs::msg::Pose &poseMsg, const std::string &target);
    void agentCallback(const geometry_msgs::msg::Pose &poseMsg, const std::string &agent);
    void agentStateCallback(const dynamic_interfaces::msg::AgentTargetState &agentTargetState, const std::string &agentName);
    void worldCallback(const dynamic_interfaces::msg::WorldInfo &worldInfo);


    // Generic allocation function. Will call figure out which algorithm to call
    void assignTargets();

    // Dynamic algorithms
    AllocationResult dynamicSimple(SystemState systemState);
    AllocationResult minimizeTime(SystemState systemState);
    AllocationResult minimizeTimeV2(SystemState systemState);

    std::vector<std::string> getCapableAgents(int type, std::map<std::string, AgentInfo> agents);
    template<class iterator_type>
    double getPathLength(Vec startPosition, iterator_type pathBegin, iterator_type pathEnd, std::map<std::string, TargetInfo> targetsInfo);

    DynamicAlgs dynamicAlgs;

    bool dataInitialized;
    std::mutex mutex;
    std::map<std::string, TargetInfo> targets;
    std::map<std::string, AgentInfo> agents;
    std::set<std::string> assignedTargets;
    std::set<std::string> completedTargets;
    AgentAllocation agentAssignment;

    std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> targetsSubscriptions_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> agentsSubscriptions_;
    std::vector<rclcpp::Subscription<dynamic_interfaces::msg::AgentTargetState>::SharedPtr> agentsTargetStates_;
    rclcpp::Subscription<dynamic_interfaces::msg::WorldInfo>::SharedPtr worldSubscription_;
    std::map<std::string, rclcpp::Client<dynamic_interfaces::srv::SetTargets>::SharedPtr> agentsTargetSet_;

//    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_;
};

template<class iterator_type>
double TaskAllocator::getPathLength(Vec currentPosition, iterator_type pathBegin, iterator_type pathEnd, std::map<std::string, TargetInfo> targetsInfo) {
    double length = 0;
    Vec previousPosition = currentPosition;

    for (auto it = pathBegin; it != pathEnd; it++) {
        std::string target = *it;

        auto targetPos = targetsInfo[target].position;

        if (!targetPos) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Trying to calculate path with a target missing a position: " << target);
            continue;
        }

        length += (*targetPos - previousPosition).magnitude();
        previousPosition = *targetPos;
    }

    return length;
}
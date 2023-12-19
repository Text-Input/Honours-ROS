#pragma once

#include "common.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <atomic>

#include "dynamic_interfaces/msg/world_info.hpp"
#include "dynamic_interfaces/msg/agent_target_state.hpp"
#include "dynamic_interfaces/srv/set_targets.hpp"
#include "dynamic_interfaces/msg/allocation_time_info.hpp"
#include "dynamic_interfaces/srv/world_info_provider_control.hpp"

enum class StaticAlgs {
	None,
	Greedy
};

enum class DynamicAlgs {
    Simple,
    MinimizeTime,
    MinimizeTimeV2,
	StaticGreedy
};

using AgentAllocation = std::map<std::string, std::vector<std::string>>;

struct SystemState {
	// What each agent needs to do
    AgentAllocation currentAllocation;

	// List of targets that exist (may not be "discovered")
    std::map<std::string, TargetInfo> targets;

	// Which targets we already assigned
    std::set<std::string> assignedTargets;

	// Which targets were completed
	std::set<std::string> completedTargets;

	// List of agents
    std::map<std::string, AgentInfo> agents;
};

struct AllocationResult {
   AgentAllocation newAllocation;
   std::set<std::string> newAssignedTargets;
};

class TaskAllocator : public rclcpp::Node {
public:
    TaskAllocator();

private:
	void parseParameters();
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

	// Static algorithms
	AllocationResult staticGreedy(SystemState systemState);

    std::vector<std::string> getCapableAgents(int type, std::map<std::string, AgentInfo> agents);
    template<class iterator_type>
    double getPathLength(Vec startPosition, iterator_type pathBegin, iterator_type pathEnd, std::map<std::string, TargetInfo> targetsInfo);

    DynamicAlgs dynamicAlgs;
	StaticAlgs staticAlgs;

    bool dataInitialized = false;
	bool firstAllocation = true; // Used to check if we need to run static allocation
	int targetPositionInitializedCount = 0;
    std::mutex mutex;
	std::mutex allocatorMutex;
    std::map<std::string, TargetInfo> targets;
    std::map<std::string, AgentInfo> agents;
    std::set<std::string> assignedTargets;
    std::set<std::string> completedTargets;
    AgentAllocation agentAssignment;

    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> targetsSubscriptions_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> agentsSubscriptions_;
    std::vector<rclcpp::Subscription<dynamic_interfaces::msg::AgentTargetState>::SharedPtr> agentsTargetStates_;
    rclcpp::Subscription<dynamic_interfaces::msg::WorldInfo>::SharedPtr worldSubscription_;
    std::map<std::string, rclcpp::Client<dynamic_interfaces::srv::SetTargets>::SharedPtr> agentsTargetSet_;
	rclcpp::Publisher<dynamic_interfaces::msg::AllocationTimeInfo>::SharedPtr allocationTimePublisher_;
	rclcpp::Client<dynamic_interfaces::srv::WorldInfoProviderControl>::SharedPtr worldInfoProviderControl_;

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

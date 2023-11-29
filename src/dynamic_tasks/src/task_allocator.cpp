#include "task_allocator.h"

#include "common.h"

#include <algorithm>

TaskAllocator::TaskAllocator(DynamicAlgs dynamicAlgs)
        : Node("task_allocator"), dynamicAlgs(dynamicAlgs), dataInitialized(false)
{
    // Subscribe to the position of all targets
    for(int i = 0; i < TARGET_COUNT; i++) {
        std::string target = "target" + std::to_string(i);
        std::function<void(const geometry_msgs::msg::Pose &poseMsg)> fnc = std::bind(&TaskAllocator::targetCallback, this, std::placeholders::_1, target);
        this->targetsSubscriptions_.push_back(this->create_subscription<geometry_msgs::msg::Pose>("/model/" + target + "/pose", 10, fnc));
    }

    // Subscribe to the position of all agents, and create client to set target
    for(int i = 0; i < AGENT_COUNT; i++) {
        std::string agent = "agent" + std::to_string(i);
        std::function<void(const geometry_msgs::msg::Pose &poseMsg)> fnc = std::bind(&TaskAllocator::agentCallback, this, std::placeholders::_1, agent);
        this->agentsSubscriptions_.push_back(this->create_subscription<geometry_msgs::msg::Pose>("/model/" + agent+ "/pose", 10, fnc));

        this->agentsTargetSet_[agent] = this->create_client<dynamic_interfaces::srv::SetTargets>("/" + agent + "/set_targets");
    }

    // Get information about the "world" (i.e. target type and robot type)
    this->worldSubscription_ = this->create_subscription<dynamic_interfaces::msg::WorldInfo>("/world_info", 10, std::bind(&TaskAllocator::worldCallback, this, std::placeholders::_1));
}

void TaskAllocator::targetCallback(const geometry_msgs::msg::Pose &poseMsg, const std::string &targetName) {
    std::lock_guard<std::mutex> lg(this->mutex);

    auto target = this->targets.find(targetName);

    if (target == this->targets.end()) {
        // Ignore target if we haven't "found" it yet
        return;
    }

    Vec position{poseMsg.position.x, poseMsg.position.y, poseMsg.position.z};

    target->second.position = position;
}

void TaskAllocator::agentCallback(const geometry_msgs::msg::Pose &poseMsg, const std::string &agentName) {
    std::lock_guard<std::mutex> lg(this->mutex);

    auto agent = this->agents.find(agentName);

    if (agent == this->agents.end()) {
        // Ignore target if we haven't "found" it yet
        return;
    }

    Vec position{poseMsg.position.x, poseMsg.position.y, poseMsg.position.z};

    agent->second.position = position;
}

void TaskAllocator::worldCallback(const dynamic_interfaces::msg::WorldInfo &worldInfo) {
    std::lock_guard<std::mutex> lg(this->mutex);

    if (this->dataInitialized && !worldInfo.is_update) {
        // This is just a periodic message. Nothing to do here.
        return;
    }

    RCLCPP_INFO(this->get_logger(), "World has been updated");

    for (auto &x: worldInfo.targets) {
        if (this->targets.find(x.name) == this->targets.end()) {
            this->targets[x.name] = TargetInfo{x.type, {}};
            RCLCPP_INFO_STREAM(this->get_logger(), "Added target " << x.name);
        }
    }

    for (auto &x: worldInfo.agents) {
        if (this->agents.find(x.name) == this->agents.end()) {
            this->agents[x.name] = AgentInfo{x.capable_types, {}};
            RCLCPP_INFO_STREAM(this->get_logger(), "Added agent " << x.name);
        }
    }

    this->dataInitialized = true;

    this->assignTargets();
}

void TaskAllocator::assignTargets() {
    switch (this->dynamicAlgs) {
        case DynamicAlgs::Simple:
            dynamicSimple();
            break;
    }
}

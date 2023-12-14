#include "task_allocator.h"

#include "common.h"

#include <algorithm>

TaskAllocator::TaskAllocator(DynamicAlgs dynamicAlgs, StaticAlgs staticAlgs)
        : Node("task_allocator"), dynamicAlgs(dynamicAlgs), staticAlgs(staticAlgs), dataInitialized(false)
{
    // Subscribe to the position of all targets
    for(int i = 0; i < TARGET_COUNT; i++) {
        std::string target = "target" + std::to_string(i);
        std::function<void(const geometry_msgs::msg::Pose &poseMsg)> fnc = std::bind(&TaskAllocator::targetCallback, this, std::placeholders::_1, target);
        this->targetsSubscriptions_.push_back(this->create_subscription<geometry_msgs::msg::Pose>("/model/" + target + "/pose", 10, fnc));
    }

    // Subscribe to the position and state of all agents, and create client to set target
    for(int i = 0; i < AGENT_COUNT; i++) {
        std::string agent = "agent" + std::to_string(i);

        std::function<void(const geometry_msgs::msg::Pose &poseMsg)> agentfnc = std::bind(&TaskAllocator::agentCallback, this, std::placeholders::_1, agent);
        this->agentsSubscriptions_.push_back(this->create_subscription<geometry_msgs::msg::Pose>("/model/" + agent+ "/pose", 10, agentfnc));

        std::function<void(const dynamic_interfaces::msg::AgentTargetState&)> agentStatefnc = std::bind(&TaskAllocator::agentStateCallback, this, std::placeholders::_1, agent);
        this->agentsTargetStates_.push_back(this->create_subscription<dynamic_interfaces::msg::AgentTargetState>("/" + agent+ "/target_state", 10, agentStatefnc));

        this->agentsTargetSet_[agent] = this->create_client<dynamic_interfaces::srv::SetTargets>("/" + agent + "/set_targets");
    }

    // Get information about the "world" (i.e. target type and robot type)
    this->worldSubscription_ = this->create_subscription<dynamic_interfaces::msg::WorldInfo>("/world_info", 10, std::bind(&TaskAllocator::worldCallback, this, std::placeholders::_1));
}

void TaskAllocator::targetCallback(const geometry_msgs::msg::Pose &poseMsg, const std::string &targetName) {
    std::lock_guard<std::mutex> lg(this->mutex);

    Vec position{poseMsg.position.x, poseMsg.position.y, poseMsg.position.z};

    this->targets[targetName].position = position;
}

void TaskAllocator::agentCallback(const geometry_msgs::msg::Pose &poseMsg, const std::string &agentName) {
    std::lock_guard<std::mutex> lg(this->mutex);

    Vec position{poseMsg.position.x, poseMsg.position.y, poseMsg.position.z};

    this->agents[agentName].position = position;
}

void TaskAllocator::worldCallback(const dynamic_interfaces::msg::WorldInfo &worldInfo) {
    {
        std::lock_guard<std::mutex> lg(this->mutex);

        if (this->dataInitialized && !worldInfo.is_update) {
            // This is just a periodic message. Nothing to do here.
            return;
        }

        RCLCPP_INFO(this->get_logger(), "World has been updated");

        for (auto &x: worldInfo.targets) {
            if (!this->targets[x.name].discovered) {
                this->targets[x.name].type = x.type;
                this->targets[x.name].discovered = true;
                RCLCPP_INFO_STREAM(this->get_logger(), "Added target " << x.name);
            }
        }

        for (auto &x: worldInfo.agents) {
            if (!this->agents[x.name].capable_types) {
                this->agents[x.name].capable_types = x.capable_types;
                RCLCPP_INFO_STREAM(this->get_logger(), "Added agent " << x.name);
            }
        }

        this->dataInitialized = true;
    }

    this->assignTargets();
}

void TaskAllocator::agentStateCallback(const dynamic_interfaces::msg::AgentTargetState &agentTargetState, const std::string &agentName) {
    std::lock_guard<std::mutex> lg(this->mutex);

    for (auto x : agentTargetState.completed_targets) {
        if (!this->completedTargets.contains(x)) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Agent " << agentName << " completed target " << x << ". Removing it from it's path.");
            this->completedTargets.insert(x);

            // TODO: This is expensive. Maybe use std::list?
            // TODO: Eventually might want to check presence of target in other agents.
            auto &assignment = this->agentAssignment[agentName];
            assignment.erase(std::remove(assignment.begin(), assignment.end(), x), assignment.end());
        }
    }
}

void TaskAllocator::assignTargets() {
    SystemState state;
    {
        std::lock_guard<std::mutex> lg(this->mutex);
        state = { this->agentAssignment, this->targets, this->assignedTargets, this->completedTargets, this->agents };
    }

    AllocationResult result;
	if (firstAllocation && staticAlgs != StaticAlgs::None) {
		if (this->targets.empty()) {
			// Only run once we get info about the world
			return;
		}

		RCLCPP_INFO(this->get_logger(), "Running static allocation");

		switch (this->staticAlgs) {
			case StaticAlgs::Greedy:
				result = staticGreedy(state);
				break;
			default:
				throw std::runtime_error("Unhandled static algorithm");
		}

		RCLCPP_INFO(this->get_logger(), "Done static allocation");
	} else {
		switch (this->dynamicAlgs) {
			case DynamicAlgs::Simple:
				result = dynamicSimple(state);
				break;
			case DynamicAlgs::MinimizeTime:
				result = minimizeTime(state);
				break;
			case DynamicAlgs::MinimizeTimeV2:
				result = minimizeTimeV2(state);
				break;
			case DynamicAlgs::StaticGreedy:
				result = staticGreedy(state);
				break;
			default:
				throw std::runtime_error("Unhandled dynamic algorithm");
		}
	}

	firstAllocation = false;

    {
        std::lock_guard<std::mutex> lg(this->mutex);

        this->assignedTargets = result.newAssignedTargets;

        for (auto x : result.newAllocation) {
            // Agent's allocations got updated
            if (this->agentAssignment[x.first] != x.second) {
                // Tell the agent the new assignment
                auto request = std::make_shared<dynamic_interfaces::srv::SetTargets::Request>();
                request->targets = x.second;

                // TODO: Check result
                this->agentsTargetSet_[x.first]->async_send_request(request);

                this->agentAssignment[x.first] = x.second;
            }
        }
    }
}

std::vector<std::string> TaskAllocator::getCapableAgents(int type, std::map<std::string, AgentInfo> agents) {
    // Find agents of the given type
    std::vector<std::string> capable_agents;
    for (auto &y : agents) {
        auto &capable_types = y.second.capable_types;
        if (!capable_types) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Agent " << y.first << " has not capable types yet.");
            continue;
        }
        if (std::find(capable_types->begin(), capable_types->end(), type) != capable_types->end()) {
            capable_agents.push_back(y.first);
        }
    }

    return capable_agents;
}


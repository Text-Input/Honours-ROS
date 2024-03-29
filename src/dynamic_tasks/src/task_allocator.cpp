#include "task_allocator.h"

#include "common.h"

#include <algorithm>

#include <boost/chrono/include.hpp>

TaskAllocator::TaskAllocator()
        : Node("task_allocator")
{
	parseParameters();
	this->declare_parameter<int64_t>("target_count");
	auto targetCount = this->get_parameter("target_count").as_int();

    // Subscribe to the position of all targets
    for(int i = 0; i < targetCount; i++) {
        std::string target = "target" + std::to_string(i);
        std::function<void(const geometry_msgs::msg::Pose &poseMsg)> fnc = std::bind(&TaskAllocator::targetCallback, this, std::placeholders::_1, target);
        this->targetsSubscriptions_[target] = this->create_subscription<geometry_msgs::msg::Pose>("/model/" + target + "/pose", 10, fnc);
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

	// For publishing info about the time each allocation took
	this->allocationTimePublisher_ = this->create_publisher<dynamic_interfaces::msg::AllocationTimeInfo>("/allocation_time_info", 10);

	// For controlling new targets coming (say during static allocation)
	this->worldInfoProviderControl_ = this->create_client<dynamic_interfaces::srv::WorldControl>("/world_info_provider_control");
	this->simulationControl_ = this->create_client<dynamic_interfaces::srv::WorldControl>("/simulation_control");
}

void TaskAllocator::parseParameters() {
	this->declare_parameter("dalg", "minimize_time_v2");
	this->declare_parameter("salg", "none");

	auto dalg = this->get_parameter("dalg").as_string();
	if (dalg == "simple")
		dynamicAlgs = DynamicAlgs::Simple;
	else if (dalg == "minimize_time")
		dynamicAlgs = DynamicAlgs::MinimizeTime;
	else if (dalg == "minimize_time_v2")
		dynamicAlgs = DynamicAlgs::MinimizeTimeV2;
	else if (dalg == "static_greedy")
		dynamicAlgs = DynamicAlgs::StaticGreedy;
	else {
		std::cout << "Invalid dynamic alg option: " << dalg << std::endl;
		exit(1);
	}

	auto salg = this->get_parameter("salg").as_string();
	if (salg == "none")
		staticAlgs = StaticAlgs::None;
	else if (salg == "greedy")
		staticAlgs = StaticAlgs::Greedy;
	else {
		std::cout << "Invalid static alg option: " << salg << std::endl;
		exit(1);
	}
}

void TaskAllocator::targetCallback(const geometry_msgs::msg::Pose &poseMsg, const std::string &targetName) {
	std::lock_guard<std::mutex> lg(this->mutex);

	Vec position{poseMsg.position.x, poseMsg.position.y, poseMsg.position.z};

	this->targets[targetName].position = position;

	// Assume target position will not change
	this->targetsSubscriptions_[targetName].reset();
	this->targetsSubscriptions_.erase(targetName);

	// Keep track of how many targets we received the position
	this->targetPositionInitializedCount++;
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

    for (auto &x : agentTargetState.completed_targets) {
        if (!this->completedTargets.contains(x)) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Agent " << agentName << " completed target " << x << ". Removing it from it's path.");
            this->completedTargets.insert(x);

            // TODO: This is expensive. Maybe use std::list?
			for (auto &assignment : this->agentAssignment) {
				assignment.second.erase(std::remove(assignment.second.begin(), assignment.second.end(), x), assignment.second.end());
			}
        }
    }
}

void TaskAllocator::assignTargets() {
	// Make sure we only run one allocator at once to prevent weird behaviour
	std::lock_guard<std::mutex> lg(this->allocatorMutex);

    SystemState state;
    {
        std::lock_guard<std::mutex> lg2(this->mutex);
        state = { this->agentAssignment, this->targets, this->assignedTargets, this->completedTargets, this->agents };
    }

	auto timestamp_thread_start = boost::chrono::thread_clock::now();
    AllocationResult result;
	if (firstAllocation && staticAlgs != StaticAlgs::None) {
		if (!this->dataInitialized || this->targetPositionInitializedCount != this->get_parameter("target_count").as_int()) {
			// Only run once we get all the info about the world
			return;
		}

		RCLCPP_INFO(this->get_logger(), "Running static allocation");

		// Make sure no new targets are adding during this step
		this->pauseWorld(true);

		switch (this->staticAlgs) {
			case StaticAlgs::Greedy:
				result = staticGreedy(state);
				break;
			default:
				throw std::runtime_error("Unhandled static algorithm");
		}

		RCLCPP_INFO(this->get_logger(), "Done static allocation");

		// Now allow new targets to come
		this->pauseWorld(false);
	} else {
		// While it could be the first, we don't want it to be identified as static
		firstAllocation = false;

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
				// This algorithm is so slow, we need to pause the world to allow it to keep up.
				this->pauseWorld(true);
				result = staticGreedy(state);
				this->pauseWorld(false);
				break;
			default:
				throw std::runtime_error("Unhandled dynamic algorithm");
		}
	}

	auto elapsed_cpu_time = boost::chrono::duration_cast<boost::chrono::microseconds>(boost::chrono::thread_clock::now() - timestamp_thread_start).count();

	int remaining_targets_count = 0;
	for (auto &x : result.newAllocation) {
		// Should be accurate since we have a callback removing any completed targets from the agent's allocation
		remaining_targets_count += x.second.size();
	}

	dynamic_interfaces::msg::AllocationTimeInfo allocationTimeInfo;
	allocationTimeInfo.is_first_static = firstAllocation;
	allocationTimeInfo.elapsed_time_us = elapsed_cpu_time;
	allocationTimeInfo.remaining_targets = remaining_targets_count;
	allocationTimeInfo.targets_processed = result.targetsProcessed;
	this->allocationTimePublisher_->publish(allocationTimeInfo);

	firstAllocation = false;

    {
        std::lock_guard<std::mutex> lg2(this->mutex);

        this->assignedTargets = result.newAssignedTargets;

        for (const auto& x : result.newAllocation) {
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


void TaskAllocator::pauseWorld(bool shouldPause) {
	auto request = std::make_shared<dynamic_interfaces::srv::WorldControl::Request>();
	request->pause = shouldPause;
	this->worldInfoProviderControl_->async_send_request(request);
	this->simulationControl_->async_send_request(request);
}

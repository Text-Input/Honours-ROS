#include "world_info_provider.h"

#include "common.h"

using namespace std::chrono_literals;

// Wait time between target "discovery"
constexpr std::chrono::milliseconds TARGET_PERIOD = 500ms;

WorldInfoProvider::WorldInfoProvider()
        : Node("target_info_provider"), rng(10), dist6(0, 5)
{
	this->declare_parameter("known_target_percentage", 0.5);
	this->declare_parameter<int64_t>("target_count");
	this->declare_parameter("specialized", false);
	this->declare_parameter("target_discovered_chunk_size", 1);

    info_pub = this->create_publisher<dynamic_interfaces::msg::WorldInfo>("/world_info", 10);
    timer_ = this->create_wall_timer(TARGET_PERIOD, std::bind(&WorldInfoProvider::timer_callback, this));
	control = this->create_service<dynamic_interfaces::srv::WorldControl>("/world_info_provider_control", std::bind(
			&WorldInfoProvider::control_callback, this, std::placeholders::_1));

    generate_capabilities();
}

void WorldInfoProvider::generate_capabilities() {
	double known_percentage = this->get_parameter("known_target_percentage").as_double();
	int64_t target_count = this->get_parameter("target_count").as_int();
	bool specialized = this->get_parameter("specialized").as_bool();

    for (int i = 0; i < target_count; i++) {
        std::string name = "target" + std::to_string(i);

	    this->target_capabilities[name] = TargetWorldInfo{static_cast<uint8_t>(this->dist6(this->rng))};

		// Add a certain amount of targets that are known from the start
		if (i < target_count * known_percentage) {
			this->known_targets.push_back(name);
		}
    }

	if (!specialized) {
		this->agent_capabilities["agent0"] = {0, 1, 2, 3, 4, 5};
		this->agent_capabilities["agent1"] = {0, 1, 2, 3, 4, 5};
		this->agent_capabilities["agent2"] = {0, 1, 2, 3, 4, 5};
		this->agent_capabilities["agent3"] = {0, 1, 2, 3, 4, 5};
		this->agent_capabilities["agent4"] = {0, 1, 2, 3, 4, 5};
		this->agent_capabilities["agent5"] = {0, 1, 2, 3, 4, 5};
	} else {
		this->agent_capabilities["agent0"] = {0, 3, 5};
		this->agent_capabilities["agent1"] = {0, 4, 5};
		this->agent_capabilities["agent2"] = {1, 3, 5};
		this->agent_capabilities["agent3"] = {1, 4, 5};
		this->agent_capabilities["agent4"] = {2, 3, 5};
		this->agent_capabilities["agent5"] = {2, 4, 5};
	}
}

void WorldInfoProvider::timer_callback() {
	if (this->is_paused) {
		return;
	}

	int chunk_size = this->get_parameter("target_discovered_chunk_size").as_int();
    dynamic_interfaces::msg::WorldInfo worldInfo;

    for (auto &x : this->known_targets) {
		dynamic_interfaces::msg::Target target;
		target.name = x;
		target.type = this->target_capabilities[x].type;

		worldInfo.targets.push_back(target);
    }

    for (auto &x : this->agent_capabilities) {
        dynamic_interfaces::msg::Agent target;
        target.name = x.first;
        target.capable_types = x.second;

        worldInfo.agents.push_back(target);
    }

    if (worldInfo.targets != this->previousWorldInfo.targets) {
        worldInfo.is_update = true;
    }

	for (int i = 0; i < chunk_size; i++) {
		if (this->known_targets.size() != this->target_capabilities.size()) {
			// Next target is just the current size of the list
			auto targetName = "target" + std::to_string(this->known_targets.size());
			this->known_targets.push_back(targetName);
		}
	}

    this->info_pub->publish(worldInfo);
    this->previousWorldInfo = worldInfo;
}

void WorldInfoProvider::control_callback(const std::shared_ptr<dynamic_interfaces::srv::WorldControl::Request>& request) {
	this->is_paused = request->pause;

	if (this->is_paused) {
		RCLCPP_INFO(this->get_logger(), "Pausing...");
	} else {
		RCLCPP_INFO(this->get_logger(), "Resuming");
	}
}

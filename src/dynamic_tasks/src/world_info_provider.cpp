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

    info_pub = this->create_publisher<dynamic_interfaces::msg::WorldInfo>("/world_info", 10);
    timer_ = this->create_wall_timer(TARGET_PERIOD, std::bind(&WorldInfoProvider::timer_callback, this));
	control = this->create_service<dynamic_interfaces::srv::WorldInfoProviderControl>("/world_info_provider_control", std::bind(&WorldInfoProvider::control_callack, this, std::placeholders::_1));

    generate_capabilities();
}

void WorldInfoProvider::generate_capabilities() {
	double known_percentage = this->get_parameter("known_target_percentage").as_double();
	int64_t target_count = this->get_parameter("target_count").as_int();

    for (int i = 0; i < target_count; i++) {
        std::string name = "target" + std::to_string(i);

	    this->target_capabilities[name] = TargetWorldInfo{static_cast<uint8_t>(this->dist6(this->rng))};

		// Add a certain amount of targets that are known from the start
		if (i < target_count * known_percentage) {
			this->known_targets.push_back(name);
		}
    }

    this->agent_capabilities["agent0"] = {0, 1, 2, 3, 4, 5};
    this->agent_capabilities["agent1"] = {0, 1, 2, 3, 4, 5};
    this->agent_capabilities["agent2"] = {0, 1, 2, 3, 4, 5};
    this->agent_capabilities["agent3"] = {0, 1, 2, 3, 4, 5};
    this->agent_capabilities["agent4"] = {0, 1, 2, 3, 4, 5};
    this->agent_capabilities["agent5"] = {0, 1, 2, 3, 4, 5};
}

void WorldInfoProvider::timer_callback() {
	if (this->is_paused) {
		return;
	}

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

	if (this->known_targets.size() != this->target_capabilities.size()) {
		// Next target is just the current size of the list
		auto targetName = "target" + std::to_string(this->known_targets.size());
		this->known_targets.push_back(targetName);
	}

    this->info_pub->publish(worldInfo);
    this->previousWorldInfo = worldInfo;
}

void WorldInfoProvider::control_callack(const std::shared_ptr<dynamic_interfaces::srv::WorldInfoProviderControl::Request>& request) {
	this->is_paused = request->pause;

	if (this->is_paused) {
		RCLCPP_INFO(this->get_logger(), "Pausing...");
	} else {
		RCLCPP_INFO(this->get_logger(), "Resuming");
	}
}

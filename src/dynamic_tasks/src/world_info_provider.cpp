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
    timer_ = this->create_wall_timer(1s, std::bind(&WorldInfoProvider::timer_callback, this));

    generate_capabilities();
}

void WorldInfoProvider::generate_capabilities() {
	double known_percentage = this->get_parameter("known_target_percentage").as_double();
	int64_t target_count = this->get_parameter("target_count").as_int();

    auto enable_time{std::chrono::steady_clock::now()};
    enable_time += TARGET_PERIOD;

    for (int i = 0; i < target_count; i++) {
        std::string name = "target" + std::to_string(i);

		// Only start increasing the time once we get past the required percentage
		if (i > target_count * known_percentage) {
			this->target_capabilities[name] = TargetWorldInfo{static_cast<uint8_t>(this->dist6(this->rng)), enable_time};
			enable_time += TARGET_PERIOD;
		} else {
			// Put the enable time in the past.
			this->target_capabilities[name] = TargetWorldInfo{static_cast<uint8_t>(this->dist6(this->rng)), enable_time - std::chrono::seconds(10)};
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
    dynamic_interfaces::msg::WorldInfo worldInfo;

    for (auto &x : this->target_capabilities) {
        if (std::chrono::steady_clock::now() >= x.second.enable_time) {
            dynamic_interfaces::msg::Target target;
            target.name = x.first;
            target.type = x.second.type;

            worldInfo.targets.push_back(target);
        }
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

    this->info_pub->publish(worldInfo);
    this->previousWorldInfo = worldInfo;
}

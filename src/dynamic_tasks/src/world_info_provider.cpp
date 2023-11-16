#include "world_info_provider.h"

#include "common.h"

using namespace std::chrono_literals;

WorldInfoProvider::WorldInfoProvider()
        : Node("target_info_provider"), rng(10), dist6(1, 6)
{
    info_pub = this->create_publisher<dynamic_interfaces::msg::WorldInfo>("/world_info", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&WorldInfoProvider::timer_callback, this));

    generate_capabilities();
}

void WorldInfoProvider::generate_capabilities() {
    for (int i = 0; i < TARGET_COUNT; i++) {
        std::string name = "target" + std::to_string(i);

        this->target_capabilities[name] = this->dist6(this->rng);
    }

    this->agent_capabilities["agent0"] = {0};
    this->agent_capabilities["agent1"] = {1, 2};
    this->agent_capabilities["agent2"] = {2, 5};
    this->agent_capabilities["agent3"] = {3};
    this->agent_capabilities["agent4"] = {4};
    this->agent_capabilities["agent5"] = {5};
}

void WorldInfoProvider::timer_callback() {
    dynamic_interfaces::msg::WorldInfo worldInfo;

    for (auto x : this->target_capabilities) {
        dynamic_interfaces::msg::Target target;
        target.name = x.first;
        target.type = x.second;

        worldInfo.targets.push_back(target);
    }

    for (auto x : this->agent_capabilities) {
        dynamic_interfaces::msg::Agent target;
        target.name = x.first;
        target.capable_types = x.second;

        worldInfo.agents.push_back(target);
    }

    this->info_pub->publish(worldInfo);
}

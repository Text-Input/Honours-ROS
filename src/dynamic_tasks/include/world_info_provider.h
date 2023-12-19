#pragma once

#include <map>
#include <random>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "dynamic_interfaces/msg/world_info.hpp"
#include "dynamic_interfaces/srv/world_control.hpp"

struct TargetWorldInfo {
   uint8_t type;
};

class WorldInfoProvider : public rclcpp::Node
{
public:
    WorldInfoProvider();

private:
    void generate_capabilities();
    void timer_callback();
	void control_callback(const std::shared_ptr<dynamic_interfaces::srv::WorldControl::Request>& request);

    rclcpp::Publisher<dynamic_interfaces::msg::WorldInfo>::SharedPtr info_pub;
	rclcpp::Service<dynamic_interfaces::srv::WorldControl>::SharedPtr control;

    rclcpp::TimerBase::SharedPtr timer_;
    std::mt19937 rng;
    std::uniform_int_distribution<std::mt19937::result_type> dist6;

    std::map<std::string, TargetWorldInfo> target_capabilities;
	std::vector<std::string> known_targets;
    std::map<std::string, std::vector<uint8_t>> agent_capabilities;

    dynamic_interfaces::msg::WorldInfo previousWorldInfo;

	std::atomic<bool> is_paused = false;
};
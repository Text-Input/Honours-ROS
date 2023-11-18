#pragma once

#include <map>
#include <random>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "dynamic_interfaces/msg/world_info.hpp"

struct TargetWorldInfo {
   uint8_t type;
   std::chrono::time_point<std::chrono::steady_clock> enable_time;
};

class WorldInfoProvider : public rclcpp::Node
{
public:
    WorldInfoProvider();

private:
    void generate_capabilities();
    void timer_callback();

    rclcpp::Publisher<dynamic_interfaces::msg::WorldInfo>::SharedPtr info_pub;

    rclcpp::TimerBase::SharedPtr timer_;
    std::mt19937 rng;
    std::uniform_int_distribution<std::mt19937::result_type> dist6;

    std::map<std::string, TargetWorldInfo> target_capabilities;
    std::map<std::string, std::vector<uint8_t>> agent_capabilities;

    dynamic_interfaces::msg::WorldInfo previousWorldInfo;
};
#pragma once

#include <optional>
#include <cstdint>
#include <vector>

#include "vec.h"

constexpr int TARGET_COUNT = 50;
constexpr int AGENT_COUNT = 6;

struct TargetInfo {
    bool discovered = false;
    uint8_t type = {};
    std::optional<Vec> position = {};
};

struct AgentInfo {
    std::optional<std::vector<uint8_t>> capable_types = {};
    std::optional<Vec> position = {};
};

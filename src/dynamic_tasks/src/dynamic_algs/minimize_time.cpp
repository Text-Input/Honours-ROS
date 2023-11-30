#include "task_allocator.h"

AllocationResult TaskAllocator::minimizeTime(SystemState systemState) {
    auto newAllocation = systemState.currentAllocation;
    auto newAssignedTargets = systemState.assignedTargets;

    for (auto &x: systemState.targets) {
        auto targetToAssign = x.first;
        if (x.second.discovered && systemState.assignedTargets.find(targetToAssign) == systemState.assignedTargets.end()) {
            // Target has not been assigned yet.

            // Find agents of the given type
            std::vector<std::string> capable_agents = getCapableAgents(x.second.type, systemState.agents);

            if (capable_agents.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No agents found for type %d", x.second.type);
                continue;
            }

            // For each agent, add target to the end of their path, and calculate length
            std::map<std::string, std::pair<std::vector<std::string>, int>> paths;
            for (const auto& y: capable_agents) {
                auto newPath = systemState.currentAllocation[y];
                newPath.push_back(targetToAssign);

                auto agentPosition = systemState.agents[y].position;
                if (!agentPosition) {
                    RCLCPP_WARN(this->get_logger(), "Trying to add target to agent with no position");
                }

                auto distance = this->getPathLength(*agentPosition, newPath.begin(), newPath.end(), systemState.targets);

                paths[y] = {newPath, distance};
            }

            // Find the agent with the minimum path
            std::string minPathAgent;
            double minLength = -1;
            for(const auto& y : paths) {
                if (minLength == -1 || y.second.second < minLength) {
                    minLength = y.second.second;
                    minPathAgent = y.first;
                }
            }

            newAllocation[minPathAgent] = paths[minPathAgent].first;
            newAssignedTargets.insert(targetToAssign);
        }
    }

    return { newAllocation, newAssignedTargets };
}

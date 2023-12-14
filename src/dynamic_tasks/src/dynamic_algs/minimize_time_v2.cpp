#include "task_allocator.h"

AllocationResult TaskAllocator::minimizeTimeV2(SystemState systemState) {
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

            // Find the agent with the minimum path
            std::string minPathAgent;
            std::vector<std::string> path;
            double minLength = -1;
            for(const auto& agent : capable_agents) {
                auto agentPosition = systemState.agents[agent].position;
                if (!agentPosition) {
                    RCLCPP_WARN(this->get_logger(), "Trying to add target to agent with no position");
	                continue;
                }

                std::list<std::string> currentPath = { newAllocation[agent].begin(), newAllocation[agent].end() };

                auto it = currentPath.begin();
                bool reachedEnd = false;
                while (!reachedEnd) {
                    // Do this so we run one last time when it reaches the end
                    if (it == currentPath.end()) {
                        reachedEnd = true;
                    }

                    it = currentPath.insert(it, targetToAssign);

                    double length = this->getPathLength(*agentPosition, currentPath.begin(), currentPath.end(), systemState.targets);

                    if (minLength == -1 || length < minLength) {
                        minLength = length;
                        minPathAgent = agent;
                        path = { currentPath.begin(), currentPath.end() };
                    }

                    // Reset list for next iteration
                    it = currentPath.erase(it);

                    // Not sure if increasing beyond end is defined. Better not risk it.
                    if (!reachedEnd) {
                        it++;
                    }
                }
            }

            newAllocation[minPathAgent] = path;
            newAssignedTargets.insert(targetToAssign);
        }
    }

    return { newAllocation, newAssignedTargets };
}

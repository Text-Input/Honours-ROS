#include "task_allocator.h"

AllocationResult TaskAllocator::dynamicSimple(SystemState systemState) {
    auto newAllocation = systemState.currentAllocation;
    auto newAssignedTargets = systemState.assignedTargets;

    for(auto &x : systemState.targets) {
        if (x.second.discovered && systemState.assignedTargets.find(x.first) == systemState.assignedTargets.end()) {
            // Target has not been assigned yet.

            // Find agents of the given type
            std::vector<std::string> capable_agents = getCapableAgents(x.second.type, systemState.agents);

            if (capable_agents.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No agents found for type %d", x.second.type);
                continue;
            }

            // For now just select the first one
            auto chosen_agent = capable_agents[0];

            // Keep track of the assignment locally
            newAllocation[chosen_agent].push_back(x.first);

            newAssignedTargets.insert(x.first);
        }
    }

    return { newAllocation, newAssignedTargets };
}
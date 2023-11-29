#include "task_allocator.h"

void TaskAllocator::dynamicSimple() {
    for(auto &x : this->targets) {
        if (this->assignedTargets.find(x.first) == this->assignedTargets.end()) {
            // Target has not been assigned yet.

            // Find agents of the given type
            std::vector<std::string> capable_agents;
            for (auto &y : this->agents) {
                auto &capable_types = y.second.capable_types;
                if (std::find(capable_types.begin(), capable_types.end(), x.second.type) != capable_types.end()) {
                    capable_agents.push_back(y.first);
                }
            }

            if (capable_agents.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No agents found for type %d", x.second.type);
                continue;
            }

            // For now just select the first one
            auto chosen_agent = capable_agents[0];

            // Keep track of the assignment locally
            this->agentAssignment[chosen_agent].push_back(x.first);

            // Tell the agent the new assignment
            auto request = std::make_shared<dynamic_interfaces::srv::SetTargets::Request>();
            request->targets = this->agentAssignment[chosen_agent];

            // For now we just ignore the result
            auto result = this->agentsTargetSet_[chosen_agent]->async_send_request(request);

            this->assignedTargets.insert(x.first);
        }
    }
}
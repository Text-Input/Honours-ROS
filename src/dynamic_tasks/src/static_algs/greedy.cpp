#include "task_allocator.h"

AllocationResult TaskAllocator::staticGreedy(SystemState systemState) {
	AgentAllocation newAllocation;
	std::set<std::string> newAssignedTargets;

	// Get a list of targets that we need to allocate. For this algorithm, we don't care about currently assigned targets.
	std::map<std::basic_string<char>, TargetInfo> remainingTargets;
	for (auto &x: systemState.targets) {
		auto targetToAssign = x.first;
		if (x.second.discovered &&
		    systemState.completedTargets.find(targetToAssign) == systemState.completedTargets.end()) {
			// Target is discovered and not completed yet
			remainingTargets[x.first] = x.second;
		}
	}

	int targetsProcessed = remainingTargets.size();

	while (!remainingTargets.empty()) {
		// Find the target and agent that would increase the path length minimally
		std::string minPathAgent;
		auto minPathTarget = remainingTargets.begin();
		double minimumLength = std::numeric_limits<double>::infinity();
		for (auto target = remainingTargets.begin(); target != remainingTargets.end(); target++) {
			std::vector<std::string> capable_agents = getCapableAgents(target->second.type, systemState.agents);

			for (auto &agent: capable_agents) {
				auto agentPosition = systemState.agents[agent].position;

				auto currentPath = newAllocation[agent];
				currentPath.push_back(target->first);

				double length = this->getPathLength(*agentPosition, currentPath.begin(), currentPath.end(),
				                                    systemState.targets);

				if (length < minimumLength) {
					minimumLength = length;
					minPathAgent = agent;
					minPathTarget = target;
				}
			}
		}

		newAllocation[minPathAgent].push_back(minPathTarget->first);
		newAssignedTargets.insert(minPathTarget->first);
		remainingTargets.erase(minPathTarget);
	}

	return {newAllocation, newAssignedTargets, targetsProcessed};
}

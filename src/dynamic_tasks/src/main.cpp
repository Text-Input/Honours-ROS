#include "agent.h"
#include "world_info_provider.h"

#include <memory>
#include <map>
#include <chrono>
#include <thread>


#include <rclcpp/rclcpp.hpp>

std::string agentNames[] = {"agent1", "agent2", "agent3", "agent4", "agent5", "agent6"};
std::tuple<std::string, std::string> assignedTargets[] = {
        {"target1", "agent1"},
        {"target2", "agent1"},
        {"target3", "agent1"},
        {"target4", "agent1"},
        {"target5", "agent1"},
        {"target6", "agent1"},
        {"target7", "agent1"},
        };

std::map<std::string, std::shared_ptr<Agent>> agents;

void assign_tasks() {
    for (auto task : assignedTargets) {
        agents[std::get<1>(task)]->add_target(std::get<0>(task));

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto target_info = std::make_shared<WorldInfoProvider>();
    executor.add_node(target_info);

    for (const auto& agentName : agentNames) {
        auto agent = std::make_shared<Agent>(agentName);
        agents[agentName] = agent;
        executor.add_node(agent);
    }

    std::thread t(assign_tasks);

    executor.spin();

	rclcpp::shutdown();

	return 0;
}

#include "agent.h"

#include <memory>
#include <map>
#include <chrono>
#include <thread>


#include <rclcpp/rclcpp.hpp>

std::string agentNames[] = {"agent1", "agent2", "agent3", "agent4", "agent5", "agent6"};
std::tuple<std::string, std::string> assignedTargets[] = {
        {"target1", "agent1"},
        {"target2", "agent2"},
        {"target3", "agent3"},
        {"target4", "agent4"},
        {"target5", "agent5"},
        {"target6", "agent6"},
        {"target7", "agent6"},
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
    for (const auto& agentName : agentNames) {
        auto agent = std::make_shared<Agent>(agentName);
        agents[agentName] = agent;
        executor.add_node(agent);
    }

//
//    auto agent6 = std::make_shared<Agent>("agent6");
//    agent6->add_target("target6");
//
//    auto agent1 = std::make_shared<Agent>("agent1");
//    agent1->add_target("target1");
//
//    executor.add_node(agent1);
//    executor.add_node(agent6);

    std::thread t(assign_tasks);

    executor.spin();

	rclcpp::shutdown();

	return 0;
}

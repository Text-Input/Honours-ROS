#include "agent.h"
#include "world_info_provider.h"
#include "task_allocator.h"
#include "common.h"
#include "simulation.h"

#include <memory>
#include <map>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <cxxopts.hpp>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

	// Add little delay to allow Python program to start
	std::this_thread::sleep_for(std::chrono::seconds(2));

    rclcpp::executors::MultiThreadedExecutor executor;

	auto simulation = std::make_shared<Simulation>();
	executor.add_node(simulation);

    std::vector<std::shared_ptr<Agent>> agents;
    for (int i = 0; i < AGENT_COUNT; i++) {
        auto agent = std::make_shared<Agent>(i);
        agents.push_back(agent);

        executor.add_node(agent);
    }

    auto target_info = std::make_shared<WorldInfoProvider>();
    executor.add_node(target_info);

    auto task_alloc = std::make_shared<TaskAllocator>();
    executor.add_node(task_alloc);

    executor.spin();

	rclcpp::shutdown();

	return 0;
}

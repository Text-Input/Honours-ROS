#include "agent.h"
#include "world_info_provider.h"
#include "task_allocator.h"
#include "common.h"

#include <memory>
#include <map>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <cxxopts.hpp>

void task() {
	std::this_thread::sleep_for (std::chrono::seconds(1));
	exit(0);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

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

	std::thread t1(task);

    executor.spin();

	rclcpp::shutdown();

	return 0;
}

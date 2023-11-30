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

struct Args {
    DynamicAlgs dynamicAlgs;
};

Args parse_args(int argc, char * argv[]);

int main(int argc, char * argv[]) {
    auto args = parse_args(argc, argv);

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

    auto task_alloc = std::make_shared<TaskAllocator>(args.dynamicAlgs);
    executor.add_node(task_alloc);

    executor.spin();

	rclcpp::shutdown();

	return 0;
}

Args parse_args(int argc, char * argv[]) {
    cxxopts::Options options("Dynamic Tasks", "");

    options.add_options()
        ("d,dalg", "Select dynamic algorithm", cxxopts::value<std::string>())
        ;

    auto result = options.parse(argc, argv);

    DynamicAlgs dynamicAlgs;
    auto alg = result["dalg"].as<std::string>();
    if (alg == "simple")
        dynamicAlgs = DynamicAlgs::Simple;
    else if (alg == "minimize_time")
        dynamicAlgs = DynamicAlgs::MinimizeTime;
    else {
        std::cout << "Invalid dynamic alg option: " << result["dalg"].as<std::string>() << std::endl;
        exit(1);
    }

    return { dynamicAlgs };
}

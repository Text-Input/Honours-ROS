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
	StaticAlgs staticAlgs;
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

    auto task_alloc = std::make_shared<TaskAllocator>(args.dynamicAlgs, args.staticAlgs);
    executor.add_node(task_alloc);

    executor.spin();

	rclcpp::shutdown();

	return 0;
}

Args parse_args(int argc, char * argv[]) {
    cxxopts::Options options("Dynamic Tasks", "");

    options.add_options()
        ("d,dalg", "Select dynamic algorithm", cxxopts::value<std::string>())
		("s,salg", "Select static algorithm", cxxopts::value<std::string>()->default_value("none"))
        ;

    auto result = options.parse(argc, argv);

    DynamicAlgs dynamicAlgs;
    auto dalg = result["dalg"].as<std::string>();
    if (dalg == "simple")
        dynamicAlgs = DynamicAlgs::Simple;
    else if (dalg == "minimize_time")
        dynamicAlgs = DynamicAlgs::MinimizeTime;
    else if (dalg == "minimize_time_v2")
        dynamicAlgs = DynamicAlgs::MinimizeTimeV2;
    else if (dalg == "static_greedy")
	    dynamicAlgs = DynamicAlgs::StaticGreedy;
    else {
        std::cout << "Invalid dynamic alg option: " << result["dalg"].as<std::string>() << std::endl;
        exit(1);
    }

	StaticAlgs staticAlgs;
	auto salg = result["salg"].as<std::string>();
	if (salg == "none")
		staticAlgs = StaticAlgs::None;
	else if (salg == "greedy")
		staticAlgs = StaticAlgs::Greedy;
	else {
		std::cout << "Invalid static alg option: " << result["salg"].as<std::string>() << std::endl;
		exit(1);
	}

    return { dynamicAlgs, staticAlgs };
}

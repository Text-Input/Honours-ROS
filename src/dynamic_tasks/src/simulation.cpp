#include "simulation.h"

#include <random>

#include "common.h"

std::chrono::milliseconds simulationPeriod(10);

Simulation::Simulation()
		: Node("simulation")
{
	this->declare_parameter("use_gazebo", false);
	this->declare_parameter<int64_t >("target_count", 50);

	this->genWorld();

	auto targetCount = this->get_parameter("target_count").as_int();

	if (this->get_parameter("use_gazebo").as_bool()) {
		// Don't use the custom sim at the same time as gazebo
		return;
	}

	// Publish the position of all targets
	for(int i = 0; i < targetCount; i++) {
		std::string target = "target" + std::to_string(i);
		this->targetsPosePublisher_[target] = this->create_publisher<geometry_msgs::msg::Pose>("/model/" + target + "/pose", 10);
	}

	// Publish the position of all agents
	for(int i = 0; i < AGENT_COUNT; i++) {
		std::string agent = "agent" + std::to_string(i);
		this->agentsPosePublisher_[agent] = this->create_publisher<geometry_msgs::msg::Pose>("/model/" + agent + "/pose", 10);
	}

	// Subscribe to the twist messages for agents
	for(int i = 0; i < AGENT_COUNT; i++) {
		std::string agent = "agent" + std::to_string(i);
		std::function<void(const geometry_msgs::msg::Twist &)> fnc = std::bind(&Simulation::agentTwistCallback, this, std::placeholders::_1, agent);
		this->agentsTwistSubscriber_.push_back(this->create_subscription<geometry_msgs::msg::Twist>("/model/" + agent + "/cmd_vel", 10, fnc));
	}

	// Main simulation loop
	timer_ = this->create_wall_timer(simulationPeriod, std::bind(&Simulation::run, this));
}

void Simulation::genWorld() {
	auto targetCount = this->get_parameter("target_count").as_int();

	std::mt19937 rng(128371293798);
	std::uniform_real_distribution<double> distr(-50.0, 50.0);

	for (int i = 0; i < targetCount; i++) {
		auto targetName = "target" + std::to_string(i);

		double x = distr(rng);
		double y = distr(rng);

		this->targets[targetName] = { x, y, 0};
	}

	this->agents["agent0"] = { {-2, 0, 0}, { 0, 0, 0} };
	this->agents["agent1"] = { {-1, 0, 0}, { 0, 0, 0} };
	this->agents["agent2"] = { {0, 0, 0}, { 0, 0, 0} };
	this->agents["agent3"] = { {1, 0, 0}, { 0, 0, 0} };
	this->agents["agent4"] = { {2, 0, 0}, { 0, 0, 0} };
	this->agents["agent5"] = { {3, 0, 0}, { 0, 0, 0} };
}

void Simulation::run() {
	std::lock_guard<std::mutex> lg(this->mutex);

	for (auto &agent : this->agents) {
		auto &position = agent.second.position;
		auto &speed = agent.second.speed;

		auto timeElapsedSec = std::chrono::duration<double>(simulationPeriod).count();

		position = position + (speed * timeElapsedSec);
	}

	for (auto &target : this->targets) {
		geometry_msgs::msg::Pose pose;
		pose.position.x = target.second.x;
		pose.position.y = target.second.y;
		pose.position.z = target.second.z;

		this->targetsPosePublisher_[target.first]->publish(pose);
	}

	for (auto &agent : this->agents) {
		geometry_msgs::msg::Pose pose;
		pose.position.x = agent.second.position.x;
		pose.position.y = agent.second.position.y;
		pose.position.z = agent.second.position.z;

		this->agentsPosePublisher_[agent.first]->publish(pose);
	}
}

void Simulation::agentTwistCallback(const geometry_msgs::msg::Twist &twist, const std::string &agent) {
	std::lock_guard<std::mutex> lg(this->mutex);

	this->agents[agent].speed.x = twist.linear.x;
	this->agents[agent].speed.y = twist.linear.y;
	this->agents[agent].speed.z = twist.linear.z;
}


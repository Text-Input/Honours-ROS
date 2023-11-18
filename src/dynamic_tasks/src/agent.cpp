#include "agent.h"

#include "vec.h"

#include <string>
#include <memory>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

double speed = 1;

Agent::Agent(const std::string &name)
        : Node(name), name(name)
{
    subscriptionAgent_ = this->create_subscription<geometry_msgs::msg::Pose>("/model/" + name + "/pose", 10, std::bind(&Agent::position_callback, this, std::placeholders::_1));
    setTargetService_ = this->create_service<dynamic_interfaces::srv::SetTargets>("/" + name + "/set_targets", std::bind(&Agent::set_targets_callback, this, std::placeholders::_1));
    control_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/" + name + "/cmd_vel", 10);
    targetState_ = this->create_publisher<dynamic_interfaces::msg::AgentTargetState>("/" + name + "/target_state", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&Agent::timer_callback, this));
}

void Agent::position_callback(const geometry_msgs::msg::Pose &pose)
{
    std::lock_guard<std::mutex> lock(this->mutex);

    this->agentPos = Vec{pose.position.x, pose.position.y, pose.position.z};
}

void Agent::target_callback(const geometry_msgs::msg::Pose &pose) {
    std::lock_guard<std::mutex> lock(this->mutex);

    this->currentTargetPos = Vec{pose.position.x, pose.position.y, pose.position.z};
}

void Agent::timer_callback()
{
    std::lock_guard<std::mutex> lock(this->mutex);

    Vec delta;
    if (!this->currentTargetPos || !this->agentPos || !assignedTarget) {
        delta = {0, 0, 0};
    } else {
        Vec delta = *this->currentTargetPos - *this->agentPos;
        double magnitude = delta.magnitude();

        if (magnitude < 1.5) {
            std::cout << "Agent " << this->name << " arrived at assigned target" << std::endl;
            RCLCPP_INFO_STREAM(this->get_logger(), "Arrived at target " << *this->currentTargetName);

            this->remainingTargets.pop_front();
            this->update_target();

            delta = {0, 0, 0};
        } else {
            if (magnitude != 0) {
                delta /= magnitude;
            }
        }
    }

    delta *= speed;

    geometry_msgs::msg::Twist desired_speed;
    desired_speed.linear.x = delta.x;
    desired_speed.linear.y = delta.y;
    desired_speed.linear.z = delta.z;

    this->control_->publish(desired_speed);

    this->send_target_state();
}

void Agent::set_targets_callback(const std::shared_ptr<dynamic_interfaces::srv::SetTargets::Request>& request) {
    std::lock_guard<std::mutex> lock(this->mutex);
    RCLCPP_INFO(this->get_logger(), "Received new targets");

    std::vector<std::string> assigned_targets(request->targets);

    // Removed any targets that we already completed
    assigned_targets.erase(std::remove_if(
            assigned_targets.begin(),
            assigned_targets.end(),
            [this](const std::string &target) {
                return std::find(this->completedTargets.begin(), this->completedTargets.end(), target) !=
                       this->completedTargets.end();
            }), assigned_targets.end());

    // Clear the queue
    this->remainingTargets = {};

    // Add the requested targets to the queue
    for (auto &x: assigned_targets) {
        this->remainingTargets.push_back(x);
    }

    std::stringstream targets;
    for (auto &x : assigned_targets) {
        targets << x << ", ";
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Current assigned targets: " << targets.str());

    this->update_target();
}

// Important! Make sure a mutex is held before entering this function.
void Agent::update_target() {
    if (!this->remainingTargets.empty() && this->remainingTargets.front() != this->currentTargetName) {
        auto newTarget = this->remainingTargets.front();

        RCLCPP_INFO_STREAM(this->get_logger(), "Going to new target " << newTarget);

        this->currentTargetPos = {};

        // If we are currently following a target, make sure we reset the subscriber
        if (assignedTarget) {
            subscriptionTarget_.reset();
        }

        subscriptionTarget_ = this->create_subscription<geometry_msgs::msg::Pose>("/model/" + newTarget + "/pose",
                                                                                      10,
                                                                                      std::bind(&Agent::target_callback,
                                                                                                this,
                                                                                                std::placeholders::_1));
        this->currentTargetName = newTarget;

        assignedTarget = true;

    } else if (this->remainingTargets.empty()) {
        // We don't have any targets to follow.
        if (assignedTarget) {
            subscriptionTarget_.reset();
        }

        this->currentTargetPos = {};
        assignedTarget = false;
    }
}

void Agent::send_target_state() {
    dynamic_interfaces::msg::AgentTargetState targetState;
    targetState.completed_targets = this->completedTargets;
    targetState.remaining_targets = { this->remainingTargets.begin(), this->remainingTargets.end() };

    this->targetState_->publish(targetState);
}

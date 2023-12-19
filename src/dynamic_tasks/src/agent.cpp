#include "agent.h"

#include "vec.h"
#include "common.h"

#include <string>
#include <memory>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

double lookup_forward_time_seconds = 1;
double lookup_forward_time_steps = 0.05;
double collision_radius = 1.5;

Agent::Agent(int agentNum)
        : Node("agent" + std::to_string(agentNum)), name("agent" + std::to_string(agentNum)), number(agentNum)
{
	this->declare_parameter("speed", 5.0);

    subscriptionAgent_ = this->create_subscription<geometry_msgs::msg::Pose>("/model/" + name + "/pose", 10, std::bind(&Agent::position_callback, this, std::placeholders::_1));
    setTargetService_ = this->create_service<dynamic_interfaces::srv::SetTargets>("/" + name + "/set_targets", std::bind(&Agent::set_targets_callback, this, std::placeholders::_1));
    control_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/" + name + "/cmd_vel", 10);
    targetState_ = this->create_publisher<dynamic_interfaces::msg::AgentTargetState>("/" + name + "/target_state", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&Agent::timer_callback, this));

    // Subscribe to the other agent's position and velocity for collision avoidance
    for(int i = 0; i < AGENT_COUNT; i++) {
        // The lower numbered agent is in charge of avoiding collisions, so we don't even need to listen to them
        if (i <= agentNum) {
            continue;
        }

        std::function<void(geometry_msgs::msg::Pose)> poseCb = std::bind(&Agent::agent_pose_callback, this, std::placeholders::_1, i);
        std::function<void(geometry_msgs::msg::Twist)> cmdVelCb = std::bind(&Agent::agent_cmd_vel_callback, this, std::placeholders::_1, i);
        otherAgentsPose_.push_back(this->create_subscription<geometry_msgs::msg::Pose>("/model/agent" + std::to_string(i) + "/pose", 10, poseCb));
        otherAgentsCmdVel_.push_back(this->create_subscription<geometry_msgs::msg::Twist>("/model/agent" + std::to_string(i) + "/cmd_vel", 10, cmdVelCb));
    }
}

void Agent::position_callback(const geometry_msgs::msg::Pose &pose)
{
    std::lock_guard<std::mutex> lock(this->mutex);

    this->agentPos = Vec{pose.position.x, pose.position.y, pose.position.z};
}

void Agent::target_callback(const geometry_msgs::msg::Pose &pose)
{
    std::lock_guard<std::mutex> lock(this->mutex);

    this->currentTargetPos = Vec{pose.position.x, pose.position.y, pose.position.z};
}

void Agent::timer_callback()
{
    std::lock_guard<std::mutex> lock(this->mutex);

    Vec delta {};
    if (!this->currentTargetPos || !this->agentPos || !assignedTarget) {
        delta = {0, 0, 0};
    } else {
        delta = *this->currentTargetPos - *this->agentPos;
        double magnitude = delta.magnitude();

        if (magnitude < 1.5) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Arrived at target " << *this->currentTargetName);

            this->completedTargets.push_back(*this->currentTargetName);
            this->remainingTargets.pop_front();
            this->update_target();

            delta = {0, 0, 0};
        } else {
            if (magnitude != 0) {
                delta /= magnitude;
            }
        }
    }

    delta *= this->get_parameter("speed").as_double();

    delta = this->collision_avoidance(delta);

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

void Agent::agent_pose_callback(geometry_msgs::msg::Pose pose, int agentNum) {
    this->otherAgentsInfo_[agentNum].position = Vec(pose.position);
}

void Agent::agent_cmd_vel_callback(geometry_msgs::msg::Twist twist, int agentNum) {
    this->otherAgentsInfo_[agentNum].position = Vec(twist.linear);
}

Vec Agent::collision_avoidance(Vec desiredVelocity) {
    if (!this->agentPos) {
        return desiredVelocity;
    }

    for (double t = 0; t < lookup_forward_time_seconds; t += lookup_forward_time_steps) {
        Vec ourPosition = *this->agentPos + (desiredVelocity * t);
        for (auto x: this->otherAgentsInfo_) {
            // Try to avoid deadlocks
            if (x.second.velocity == Vec(0, 0, 0)) {
                continue;
            }

            Vec otherPosition = x.second.position + (x.second.velocity * t);
            double mag = (ourPosition - otherPosition).magnitude();

            if (mag <= collision_radius) {
                RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Detected collision with agent" << x.first << ". Pausing...");

                return {0, 0, 0};
            }
        }
    }

    return desiredVelocity;
}

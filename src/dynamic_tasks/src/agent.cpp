#include "agent.h"

#include "vec.h"

#include <string>
#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

double speed = 1;

Agent::Agent(const std::string &name)
        : Node(name), name(name)
{
    subscriptionAgent_ = this->create_subscription<geometry_msgs::msg::Pose>("/model/" + name + "/pose", 10, std::bind(&Agent::position_callback, this, std::placeholders::_1));
    control_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/" + name + "/cmd_vel", 10);
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

    if (!this->currentTargetPos || !this->agentPos || !assignedPos) {
        return;
    }

    Vec delta = *this->currentTargetPos - *this->agentPos;

//    std::cout << this->currentTargetPos->x << " " << this->currentTargetPos->y << " " << this->currentTargetPos->z << std::endl;
//    std::cout << this->agentPos->x << " " << this->agentPos->y << " " << this->agentPos->z << std::endl;
//    std::cout << delta.x << " " << delta.y << " " << delta.z << std::endl << std::endl;

    double magnitude = delta.magnitude();

    if (magnitude < 1.5) {
        std::cout << "Agent " << this->name << " arrived at assigned target" << std::endl;

        if (this->remainingTargets.empty()) {
            subscriptionTarget_.reset();
            this->currentTargetPos = {};
            this->assignedPos = false;
        } else {
            auto targetName = this->remainingTargets.front();
            subscriptionTarget_ = this->create_subscription<geometry_msgs::msg::Pose>("/model/" + targetName + "/pose", 10, std::bind(&Agent::target_callback, this, std::placeholders::_1));
            assignedPos = true;
            this->remainingTargets.pop();
        }

        delta.x = 0;
        delta.y = 0;
        delta.z = 0;
    } else {
        if (magnitude != 0) {
            delta /= magnitude;
        }
    }

    delta *= speed;

    geometry_msgs::msg::Twist desired_speed;
    desired_speed.linear.x = delta.x;
    desired_speed.linear.y = delta.y;
    desired_speed.linear.z = delta.z;

    this->control_->publish(desired_speed);
}

void Agent::add_target(const std::string &targetName) {
    std::lock_guard<std::mutex> lock(this->mutex);

    std::cout << "Adding target " << targetName << " to agent " << this->name << std::endl;

    if (!assignedPos) {
        subscriptionTarget_ = this->create_subscription<geometry_msgs::msg::Pose>("/model/" + targetName + "/pose", 10, std::bind(&Agent::target_callback, this, std::placeholders::_1));
        assignedPos = true;
    } else {
        this->remainingTargets.push(targetName);
    }

}

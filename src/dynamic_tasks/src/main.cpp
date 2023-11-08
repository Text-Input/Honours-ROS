#include <string>
#include <memory>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

double speed = 1;

class Agent : public rclcpp::Node
{
public:
	Agent()
			: Node("agent1")
	{
		subscriptionAgent_ = this->create_subscription<geometry_msgs::msg::Pose>("/model/agent6/pose", 10, std::bind(&Agent::position_callback, this, std::placeholders::_1));
		subscriptionTarget_ = this->create_subscription<geometry_msgs::msg::Pose>("/model/target1/pose", 10, std::bind(&Agent::target_callback, this, std::placeholders::_1));
		control_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/agent6/cmd_vel", 10);
		timer_ = this->create_wall_timer(500ms, std::bind(&Agent::timer_callback, this));
	}

private:
	void position_callback(const geometry_msgs::msg::Pose &pose)
	{
		std::lock_guard<std::mutex> lock(this->mutex);

		this->positionX = pose.position.x;
		this->positionY = pose.position.y;
		this->positionZ = pose.position.z;
	}

	void target_callback(const geometry_msgs::msg::Pose &pose) {
		std::lock_guard<std::mutex> lock(this->mutex);

		this->targetX = pose.position.x;
		this->targetY = pose.position.y;
		this->targetZ = pose.position.z;
	}


		void timer_callback()
	{
		std::lock_guard<std::mutex> lock(this->mutex);

		double deltaX = this->targetX - this->positionX;
		double deltaY = this->targetY - this->positionY;
		double deltaZ = this->targetZ - this->positionZ;

		double magnitude = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

		if (magnitude != 0) {
			deltaX /= magnitude;
			deltaY /= magnitude;
			deltaZ /= magnitude;
		}

		deltaX *= speed;
		deltaY *= speed;
		deltaZ *= speed;

		geometry_msgs::msg::Twist desired_speed;
		desired_speed.linear.x = deltaX;
		desired_speed.linear.y = deltaY;
		desired_speed.linear.z = deltaZ;

		this->control_->publish(desired_speed);
	}

	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriptionAgent_;
	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriptionTarget_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_;
	rclcpp::TimerBase::SharedPtr timer_;


	double positionX;
	double positionY;
	double positionZ;
	double targetX;
	double targetY;
	double targetZ;
	std::mutex mutex;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Agent>());
	rclcpp::shutdown();

	return 0;
}

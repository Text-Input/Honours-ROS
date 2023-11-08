#include <mutex>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/msgs/MessageTypes.hh>
#include <ignition/transport/Node.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;

namespace vector_robot {
	class VectorRobot :
			public ignition::gazebo::System,
			public ignition::gazebo::ISystemConfigure,
			public ignition::gazebo::ISystemPreUpdate,
			public ignition::gazebo::ISystemPostUpdate {
	private:
		Model model{kNullEntity};

		// Requested target velocity
		msgs::Twist targetVel;

		// Ignition communication node
		transport::Node node;

		// Mutex to protect the target velocity command
		std::mutex mutex;

	public:
		VectorRobot() {
			ignlog << "Vector Robot loaded!" << std::endl;
		}

		~VectorRobot() override {

		}

		void Configure(const ignition::gazebo::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
		               ignition::gazebo::EntityComponentManager &_ecm,
		               ignition::gazebo::EventManager &_eventMgr) override {
			ignerr << "configuring" << std::endl;

			this->model = Model(_entity);

			// Subscribe to the velocity topic
			auto topic = "/model/" + this->model.Name(_ecm) + "/cmd_vel";
			ignerr << "announcing on " << topic << std::endl;
			this->node.Subscribe(topic, &VectorRobot::OnCmdVel, this);

		}

		void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override {
			this->mode
		}

		void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
		                const ignition::gazebo::EntityComponentManager &_ecm) override {
			if (_info.paused) {
				return;
			}

			this->model;
		}

		void OnCmdVel(const msgs::Twist &_msg) {
			ignerr << "received vel" << std::endl;

			std::lock_guard<std::mutex> local(this->mutex);

			this->targetVel = _msg;
		}

	};
}

IGNITION_ADD_PLUGIN(
		vector_robot::VectorRobot,
		ignition::gazebo::System,
		vector_robot::VectorRobot::ISystemConfigure,
		vector_robot::VectorRobot::ISystemPreUpdate,
		vector_robot::VectorRobot::ISystemPostUpdate)

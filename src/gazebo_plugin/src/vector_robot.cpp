#include <ignition/gazebo/System.hh>

#include <ignition/plugin/Register.hh>

namespace vector_robot {
	class VectorRobot :
			public ignition::gazebo::System,
			public ignition::gazebo::ISystemPostUpdate {
	public:
		VectorRobot() {
			ignlog << "Vector Robot loaded!" << std::endl;
		}

	public:
		~VectorRobot() override {

		}

	public:
		void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
		                const ignition::gazebo::EntityComponentManager &_ecm) override {
		}
	};
}

IGNITION_ADD_PLUGIN(
		vector_robot::VectorRobot,
		ignition::gazebo::System,
		vector_robot::VectorRobot::ISystemPostUpdate)

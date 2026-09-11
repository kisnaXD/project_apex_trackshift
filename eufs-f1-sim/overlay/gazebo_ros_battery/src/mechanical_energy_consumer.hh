#pragma once

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros_battery/battery_consumer_base.hh>

namespace gazebo
{

class GAZEBO_VISIBLE MechanicalEnergyConsumerPlugin : public BatteryConsumerBase
{
public:
  MechanicalEnergyConsumerPlugin();
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  void OnUpdate(const common::UpdateInfo & info);

  event::ConnectionPtr before_physics_update_connection_;
  double efficiency_{1.0};
  double friction_{0.0};
  double consumer_idle_power_{0.0};
  double ignore_first_duration_{0.0};
  bool initialized_{false};
  common::Time first_update_time_;
  common::Time last_update_time_;
  common::Time last_publish_time_;
  double last_energy_{0.0};
};

}  // namespace gazebo

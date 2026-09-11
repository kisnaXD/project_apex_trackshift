#include "mechanical_energy_consumer.hh"

#include <algorithm>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MechanicalEnergyConsumerPlugin);

MechanicalEnergyConsumerPlugin::MechanicalEnergyConsumerPlugin() = default;

void MechanicalEnergyConsumerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  BatteryConsumerBase::Load(_model, _sdf);

  this->efficiency_ = _sdf->Get<double>("efficiency", this->efficiency_).first;
  this->friction_ = _sdf->Get<double>("friction", this->friction_).first;
  this->consumer_idle_power_ = _sdf->Get<double>("consumer_idle_power", this->consumer_idle_power_).first;
  this->ignore_first_duration_ = _sdf->Get<double>("ignore_first_duration", this->ignore_first_duration_).first;

  if (this->enabled_) {
    this->battery->SetPowerLoad(this->consumer_id_, this->consumer_idle_power_);
  }

  this->before_physics_update_connection_ = event::Events::ConnectBeforePhysicsUpdate(
    std::bind(&MechanicalEnergyConsumerPlugin::OnUpdate, this, std::placeholders::_1));
}

void MechanicalEnergyConsumerPlugin::OnUpdate(const common::UpdateInfo & info)
{
  if (this->first_update_time_ == common::Time::Zero) {
    this->first_update_time_ = info.simTime;
  }
  if (info.simTime < this->first_update_time_ + this->ignore_first_duration_) {
    return;
  }
  if (!this->initialized_) {
    this->last_update_time_ = this->last_publish_time_ = info.simTime;
    this->last_energy_ = this->model->GetWorldEnergyPotential();
    this->initialized_ = true;
    return;
  }

  const auto dt = (info.simTime - this->last_update_time_).Double();
  if (dt <= 0.0) {
    return;
  }

  const auto current_energy = this->model->GetWorldEnergyPotential();
  const auto delta_energy = current_energy - this->last_energy_;
  this->last_energy_ = current_energy;
  this->last_update_time_ = info.simTime;

  auto power = (delta_energy / dt) / this->efficiency_;
  power += this->friction_ * std::abs(power);
  power = std::max(power, this->consumer_idle_power_);

  if (this->enabled_) {
    this->battery->SetPowerLoad(this->consumer_id_, power);
  }

  this->Publish(power, info.simTime);
}

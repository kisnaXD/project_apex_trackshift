#include "battery_discharge.hh"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/any.pb.h>
#include <gazebo/physics/physics.hh>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin);

BatteryPlugin::BatteryPlugin() = default;

BatteryPlugin::~BatteryPlugin() = default;

void BatteryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->ros_node_ = gazebo_ros::Node::Get(_sdf);
  this->model_ = _model;
  this->world_ = _model->GetWorld();
  this->sdf_ = _sdf;

  this->gz_node_.reset(new transport::Node);
  this->gz_node_->Init(_model->GetWorld()->Name());

  this->update_period_ = 1.0 / _sdf->Get<double>("update_rate", 1.0 / this->update_period_).first;

  const auto gz_ns = "~/" + _model->GetName() + "/" + _sdf->GetAttribute("name")->GetAsString() + "/";
  this->gz_charge_power_pub_ = this->gz_node_->Advertise<msgs::Any>(gz_ns + "charge_power", 1, 1 / this->update_period_);
  this->gz_discharge_power_pub_ = this->gz_node_->Advertise<msgs::Any>(gz_ns + "discharge_power", 1, 1 / this->update_period_);

  const auto & qos = this->ros_node_->get_qos();
  this->battery_state_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::BatteryState>(
    "battery_state", qos.get_publisher_qos("battery_state", rclcpp::SensorDataQoS()));
  this->charge_state_wh_pub_ = this->ros_node_->create_publisher<std_msgs::msg::Float64>(
    "charge_level_wh", qos.get_publisher_qos("charge_level_wh", rclcpp::QoS(10)));

  this->ambient_temperature_ = _sdf->Get<double>("ambient_temperature", this->ambient_temperature_).first;
  const auto ambient_temperature_gz_topic = _sdf->Get<std::string>("ambient_temperature_gz_topic", "").first;
  if (!ambient_temperature_gz_topic.empty()) {
    this->gz_ambient_temperature_sub_ = this->gz_node_->Subscribe(
      ambient_temperature_gz_topic, &BatteryPlugin::OnGzAmbientTempMsg, this, true);
  }

  const auto link_name = _sdf->Get<std::string>("link_name");
  this->link_ = this->model_->GetLink(link_name);

  this->allow_charging_ = _sdf->Get<bool>("allow_charging", this->allow_charging_).first;
  const auto allow_charging_gz_topic = _sdf->Get<std::string>(
    "allow_charging_gz_topic", "~/" + _model->GetName() + "/allow_charging").first;
  if (!allow_charging_gz_topic.empty()) {
    this->gz_allow_charging_sub_ = this->gz_node_->Subscribe(
      allow_charging_gz_topic, &BatteryPlugin::OnGzAllowChargingMsg, this, true);
  }

  this->compute_resistance_ = _sdf->Get<bool>("compute_resistance", false).first;
  this->compute_temperature_ = _sdf->Get<bool>("compute_temperature", false).first;

  this->e0_ = _sdf->Get<double>("constant_coef");
  this->e1_ = _sdf->Get<double>("linear_coef");
  this->q0_ = _sdf->Get<double>("initial_charge");
  this->c_ = _sdf->Get<double>("capacity");
  const auto design_capacity = _sdf->Get<double>("design_capacity", this->c_).first;
  this->tau_ = _sdf->Get<double>("smooth_current_tau");
  this->t_ = this->base_temperature_ = _sdf->Get<double>("temperature", this->t_).first;

  this->forgez_mode_ = _sdf->Get<std::string>("forgez_mode", "Nominal").first;
  this->forgez_e_lap_wh_ = _sdf->Get<double>("forgez_E_lap_wh", this->forgez_e_lap_wh_).first;
  this->forgez_t_core_ = _sdf->Get<double>("forgez_T_core", this->forgez_t_core_).first;
  this->forgez_r_ot_ = _sdf->Get<double>("forgez_R_OT", this->forgez_r_ot_).first;
  this->r_ = this->forgez_r_ot_;

  if (this->compute_resistance_) {
    const auto coefs = _sdf->Get<std::string>("resistance_temperature_coeffs", "").first;
    std::vector<std::string> coeff_strs;
    boost::split(coeff_strs, coefs, boost::is_any_of(",;"));
    for (const auto & coeff_str : coeff_strs) {
      try {
        this->resistance_temperature_coeffs_.emplace_back(boost::lexical_cast<double>(coeff_str));
      } catch (const boost::bad_lexical_cast &) {
        gzerr << "Could not read coefficient " << coeff_str << " as number.\n";
      }
    }
  } else {
    this->r_ = _sdf->Get<double>("resistance", this->r_).first;
  }

  this->heat_dissipation_rate_ = _sdf->Get<double>("heat_dissipation_rate", this->heat_dissipation_rate_).first;
  this->heat_capacity_ = _sdf->Get<double>("heat_capacity", this->heat_capacity_).first;

  const auto battery_name = _sdf->Get<std::string>("battery_name");
  if (this->link_->BatteryCount() > 0) {
    this->battery_ = this->link_->Battery(battery_name);
  } else {
    gzerr << "There is no battery specification in the link!\n";
    return;
  }

  const auto frame_id = _sdf->Get<std::string>("frame_id", battery_name).first;
  const auto technology = _sdf->Get<std::string>("technology", "").first;
  auto power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  if (technology == "LIPO") {
    power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
  }

  this->battery_msg_.header.frame_id = frame_id;
  this->battery_msg_.capacity = static_cast<float>(this->c_);
  this->battery_msg_.design_capacity = static_cast<float>(design_capacity);
  this->battery_msg_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  this->battery_msg_.power_supply_technology = power_supply_technology;
  this->battery_msg_.present = true;
  this->battery_msg_.temperature = static_cast<float>(this->t_);

  this->battery_->SetUpdateFunc([this](const common::BatteryPtr & b) {
      return this->OnUpdateVoltage(b);
    });

  RCLCPP_INFO(
    this->ros_node_->get_logger(),
    "Forgez battery mode=%s T_core=%.1f E_lap=%.1f Wh R_OT=%.4f Ohm",
    this->forgez_mode_.c_str(), this->forgez_t_core_, this->forgez_e_lap_wh_, this->forgez_r_ot_);
}

void BatteryPlugin::Init()
{
  this->q_ = this->q0_;
  this->heat_energy_ = 0.0;
  if (this->compute_temperature_) {
    this->t_ = this->base_temperature_;
  }
  this->UpdateResistance();
}

void BatteryPlugin::Reset()
{
  this->ismooth_ = 0.0;
  this->last_update_time_ = common::Time::Zero;
  this->Init();
}

void BatteryPlugin::UpdateResistance()
{
  if (this->compute_resistance_ && !std::isnan(this->t_) && !this->resistance_temperature_coeffs_.empty()) {
    double new_r = 0.0;
    for (size_t i = 0; i < this->resistance_temperature_coeffs_.size(); ++i) {
      new_r += this->resistance_temperature_coeffs_[i] * std::pow(this->t_, static_cast<double>(i));
    }
    this->r_ = new_r;
  }
}

double BatteryPlugin::OnUpdateVoltage(const common::BatteryPtr & _battery)
{
  const double dt = this->world_->Physics()->GetMaxStepSize();
  double total_power = 0.0;
  double total_charge_power = 0.0;
  double total_discharge_power = 0.0;
  const double k = dt / this->tau_;

  for (const auto & power_load : _battery->PowerLoads()) {
    if (power_load.second >= 0 || this->allow_charging_) {
      total_power += power_load.second;
    }
    if (power_load.second >= 0) {
      total_discharge_power += power_load.second;
    }
    if (this->allow_charging_ && power_load.second < 0) {
      total_charge_power += -power_load.second;
    }
  }

  const auto voltage = (std::max)(_battery->Voltage(), this->e0_ + this->e1_);
  const auto iraw = total_power / voltage;
  this->ismooth_ = this->ismooth_ + k * (iraw - this->ismooth_);
  this->q_ = (std::max)(0.0, this->q_ - GZ_SEC_TO_HOUR(dt * this->ismooth_));

  this->UpdateResistance();

  const auto voltage_loss = this->r_ * this->ismooth_;
  double et = this->e0_ + this->e1_ * (1 - this->q_ / this->c_) - voltage_loss;
  et = (std::min)(et, this->e0_);

  auto power_loss = voltage_loss * this->ismooth_;

  if (this->q_ <= 0) {
    et = 0;
    power_loss = 0;
    total_discharge_power = std::min(total_discharge_power, total_charge_power);
  } else if (this->q_ >= this->c_) {
    this->q_ = this->c_;
    this->ismooth_ = (std::max)(0.0, this->ismooth_);
    power_loss = voltage_loss * this->ismooth_;
    total_charge_power = 0;
  }

  if (this->compute_temperature_) {
    const auto generated_heat_joules = power_loss * dt;
    this->heat_energy_ += generated_heat_joules;
    const auto dissipated_joules = (this->t_ - this->ambient_temperature_) * this->heat_dissipation_rate_;
    this->heat_energy_ -= dissipated_joules;
    this->t_ = this->base_temperature_ + this->heat_energy_ / this->heat_capacity_;
  }

  if (this->last_update_time_ + this->update_period_ < this->world_->SimTime()) {
    this->last_update_time_ = this->world_->SimTime();

    this->battery_msg_.header.stamp = this->ros_node_->get_clock()->now();
    this->battery_msg_.voltage = static_cast<float>(et);
    this->battery_msg_.current = static_cast<float>(-this->ismooth_);
    this->battery_msg_.charge = static_cast<float>(this->q_);
    this->battery_msg_.percentage = static_cast<float>(this->q_ / this->c_);
    if (this->q_ == this->c_) {
      this->battery_msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
    } else if (this->ismooth_ >= 0) {
      this->battery_msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    } else {
      this->battery_msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    }
    this->battery_msg_.temperature = static_cast<float>(this->t_);
    this->battery_state_pub_->publish(this->battery_msg_);

    std_msgs::msg::Float64 charge_msg_wh;
    charge_msg_wh.data = this->q_ * et;
    this->charge_state_wh_pub_->publish(charge_msg_wh);

    this->gz_charge_power_pub_->Publish(msgs::ConvertAny(total_charge_power));
    this->gz_discharge_power_pub_->Publish(msgs::ConvertAny(total_discharge_power));
  }

  return et;
}

void BatteryPlugin::OnGzAmbientTempMsg(const ConstAnyPtr & _msg)
{
  if (_msg->has_type() && _msg->type() == msgs::Any_ValueType_DOUBLE && _msg->has_double_value()) {
    this->ambient_temperature_ = _msg->double_value();
  }
}

void BatteryPlugin::OnGzAllowChargingMsg(const ConstAnyPtr & _msg)
{
  if (_msg->has_type() && _msg->type() == msgs::Any_ValueType_BOOLEAN && _msg->has_bool_value()) {
    this->allow_charging_ = _msg->bool_value();
  }
}

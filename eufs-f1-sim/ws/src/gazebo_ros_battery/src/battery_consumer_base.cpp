#include <gazebo_ros_battery/battery_consumer_base.hh>

#include <gazebo/msgs/int.pb.h>
#include <gazebo/physics/physics.hh>

using namespace gazebo;

BatteryConsumerBase::~BatteryConsumerBase()
{
  if (this->battery && this->consumer_id_ != std::numeric_limits<uint32_t>::max()) {
    this->battery->RemoveConsumer(this->consumer_id_);
  }
}

void BatteryConsumerBase::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->world = _model->GetWorld();
  this->sdf = _sdf;
  this->ros_node_ = gazebo_ros::Node::Get(_sdf);

  const auto link_name = _sdf->Get<std::string>("link_name");
  this->link = _model->GetLink(link_name);
  if (!this->link) {
    RCLCPP_ERROR(this->ros_node_->get_logger(), "Cannot find link '%s'", link_name.c_str());
    return;
  }

  const auto battery_name = _sdf->Get<std::string>("battery_name");
  this->battery = this->link->Battery(battery_name);
  if (!this->battery) {
    RCLCPP_ERROR(
      this->ros_node_->get_logger(),
      "Cannot find battery '%s' in link '%s'", battery_name.c_str(), link_name.c_str());
    return;
  }

  const auto default_consumer_name = _sdf->GetAttribute("name")->GetAsString();
  this->consumer_name_ = _sdf->Get<std::string>("consumer_name", default_consumer_name).first;
  this->consumer_id_ = this->battery->AddConsumer();

  this->gz_node_.reset(new transport::Node);
  this->gz_node_->Init(_model->GetWorld()->Name() + "/" + _model->GetName() + "/" + this->consumer_name_);

  this->gz_consumer_id_pub_ = this->gz_node_->Advertise<gazebo::msgs::Int>("~/consumer_id", 1);
  this->gz_power_load_pub_ = this->gz_node_->Advertise<gazebo::msgs::Any>("~/power_load", 1);
  this->enabled_ = _sdf->Get("enabled", true).first;
  this->gz_enable_sub_ = this->gz_node_->Subscribe(
    "~/enable", &BatteryConsumerBase::OnGzEnabledMsg, this, true);

  gazebo::msgs::Int consumer_id_msg;
  consumer_id_msg.set_data(static_cast<int32_t>(this->consumer_id_));
  this->gz_consumer_id_pub_->Publish(consumer_id_msg);
}

void BatteryConsumerBase::Reset()
{
  // no-op
}

void BatteryConsumerBase::Publish(const double power_load, const gazebo::common::Time & time)
{
  this->gz_power_load_pub_->Publish(gazebo::msgs::ConvertAny(power_load));
}

void BatteryConsumerBase::SetEnabled(const bool enabled)
{
  this->enabled_ = enabled;
}

void BatteryConsumerBase::OnGzEnabledMsg(const ConstAnyPtr & msg)
{
  if (msg->has_type() && msg->type() == msgs::Any_ValueType_BOOLEAN && msg->has_bool_value()) {
    this->SetEnabled(msg->bool_value());
  }
}

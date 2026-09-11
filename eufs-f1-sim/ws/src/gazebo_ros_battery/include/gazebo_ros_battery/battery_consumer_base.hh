#pragma once

#include <limits>
#include <memory>
#include <string>

#include <gazebo/common/Battery.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/any.pb.h>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo_ros/node.hpp>
#include <sdf/Element.hh>

namespace gazebo
{

class GAZEBO_VISIBLE BatteryConsumerBase : public gazebo::ModelPlugin
{
public:
  ~BatteryConsumerBase() override;
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void Reset() override;

protected:
  void Publish(double power_load, const gazebo::common::Time & time);
  void SetEnabled(bool enabled);
  void OnGzEnabledMsg(const ConstAnyPtr & msg);

  physics::WorldPtr world;
  physics::ModelPtr model;
  physics::LinkPtr link;
  sdf::ElementPtr sdf;
  common::BatteryPtr battery;

  gazebo_ros::Node::SharedPtr ros_node_;
  transport::NodePtr gz_node_;
  transport::PublisherPtr gz_consumer_id_pub_;
  transport::PublisherPtr gz_power_load_pub_;
  transport::SubscriberPtr gz_enable_sub_;

  uint32_t consumer_id_{std::numeric_limits<uint32_t>::max()};
  std::string consumer_name_;
  bool enabled_{true};
};

}  // namespace gazebo

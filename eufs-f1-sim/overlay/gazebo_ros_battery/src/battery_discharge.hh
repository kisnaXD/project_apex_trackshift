#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/msgs/any.pb.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sdf/sdf.hh>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{

class GAZEBO_VISIBLE BatteryPlugin : public ModelPlugin
{
public:
  BatteryPlugin();
  ~BatteryPlugin() override;
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void Init() override;
  void Reset() override;

private:
  double OnUpdateVoltage(const common::BatteryPtr & _battery);
  void OnGzAmbientTempMsg(const ConstAnyPtr & _msg);
  void OnGzAllowChargingMsg(const ConstAnyPtr & _msg);
  void UpdateResistance();

  event::ConnectionPtr update_connection_;
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  common::BatteryPtr battery_;
  sdf::ElementPtr sdf_;
  common::Time last_update_time_;
  double update_period_{1.0};
  bool allow_charging_{true};
  bool compute_resistance_{false};
  bool compute_temperature_{false};
  double base_temperature_{std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> resistance_temperature_coeffs_;
  double heat_dissipation_rate_{0.0};
  double heat_capacity_{1.0};
  double ambient_temperature_{25.0};
  double e0_{0.0};
  double e1_{0.0};
  double q0_{0.0};
  double c_{0.0};
  double r_{0.0};
  double tau_{0.0};
  double ismooth_{0.0};
  double q_{0.0};
  double t_{std::numeric_limits<double>::quiet_NaN()};
  double heat_energy_{0.0};
  std::string forgez_mode_;
  double forgez_e_lap_wh_{220.0};
  double forgez_t_core_{40.0};
  double forgez_r_ot_{0.04};

  gazebo_ros::Node::SharedPtr ros_node_;
  transport::NodePtr gz_node_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr charge_state_wh_pub_;
  transport::SubscriberPtr gz_ambient_temperature_sub_;
  transport::SubscriberPtr gz_allow_charging_sub_;
  transport::PublisherPtr gz_charge_power_pub_;
  transport::PublisherPtr gz_discharge_power_pub_;
  sensor_msgs::msg::BatteryState battery_msg_;
};

}  // namespace gazebo

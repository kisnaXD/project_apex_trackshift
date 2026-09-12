#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>

namespace gazebo
{

class EnergyAwareAckermannGatePlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model_ = model;
    this->world_ = model->GetWorld();
    this->ros_node_ = gazebo_ros::Node::Get(sdf);

    const auto link_name = sdf->Get<std::string>("link_name", "base_link").first;
    this->link_ = model->GetLink(link_name);
    if (!this->link_) {
      RCLCPP_ERROR(this->ros_node_->get_logger(), "Cannot find link '%s'", link_name.c_str());
      return;
    }

    this->update_period_ = 1.0 / sdf->Get<double>("update_rate", 50.0).first;
    this->vehicle_mass_kg_ = sdf->Get<double>("vehicle_mass_kg", this->vehicle_mass_kg_).first;
    this->max_deploy_power_w_ = sdf->Get<double>("max_deploy_power_w", this->max_deploy_power_w_).first;
    this->reserve_energy_wh_ = sdf->Get<double>("reserve_energy_wh", this->reserve_energy_wh_).first;
    this->reserve_buffer_wh_ = sdf->Get<double>("reserve_buffer_wh", this->reserve_buffer_wh_).first;
    this->thermal_derate_temperature_ = sdf->Get<double>(
      "thermal_derate_temperature", this->thermal_derate_temperature_).first;
    this->shutdown_temperature_ = sdf->Get<double>("shutdown_temperature", this->shutdown_temperature_).first;
    this->max_launch_accel_mps2_ = sdf->Get<double>("max_launch_accel_mps2", this->max_launch_accel_mps2_).first;
    this->max_brake_accel_mps2_ = sdf->Get<double>("max_brake_accel_mps2", this->max_brake_accel_mps2_).first;
    this->drag_coefficient_area_ = sdf->Get<double>("drag_coefficient_area", this->drag_coefficient_area_).first;
    this->air_density_ = sdf->Get<double>("air_density", this->air_density_).first;
    this->rolling_resistance_coefficient_ = sdf->Get<double>(
      "rolling_resistance_coefficient", this->rolling_resistance_coefficient_).first;
    this->max_speed_mps_ = sdf->Get<double>("max_speed_mps", this->max_speed_mps_).first;

    const auto qos = this->ros_node_->get_qos();
    this->cmd_sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "input_cmd_vel", qos.get_subscription_qos("input_cmd_vel", rclcpp::QoS(10)),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        this->target_cmd_ = *msg;
        this->has_cmd_ = true;
      });
    this->battery_sub_ = this->ros_node_->create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_state", qos.get_subscription_qos("battery_state", rclcpp::SensorDataQoS()),
      [this](sensor_msgs::msg::BatteryState::SharedPtr msg) {
        this->battery_energy_wh_ = std::max(0.0, static_cast<double>(msg->charge) * static_cast<double>(msg->voltage));
        this->battery_temperature_ = msg->temperature;
        this->has_battery_ = true;
      });
    this->cmd_pub_ = this->ros_node_->create_publisher<geometry_msgs::msg::Twist>(
      "output_cmd_vel", qos.get_publisher_qos("output_cmd_vel", rclcpp::QoS(10)));
    this->derate_pub_ = this->ros_node_->create_publisher<std_msgs::msg::String>(
      "gate_derate_reason", qos.get_publisher_qos("gate_derate_reason", rclcpp::QoS(10)));

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&EnergyAwareAckermannGatePlugin::OnUpdate, this, std::placeholders::_1));
  }

private:
  void OnUpdate(const common::UpdateInfo & info)
  {
    if (!this->link_) {
      return;
    }
    if (this->last_update_time_ == common::Time::Zero) {
      this->last_update_time_ = info.simTime;
      return;
    }
    const double dt = (info.simTime - this->last_update_time_).Double();
    if (dt <= 0.0 || info.simTime < this->last_publish_time_ + this->update_period_) {
      return;
    }
    this->last_update_time_ = info.simTime;
    this->last_publish_time_ = info.simTime;

    geometry_msgs::msg::Twist limited_cmd;
    if (this->has_cmd_) {
      limited_cmd = this->target_cmd_;
    }

    const auto pose = this->link_->WorldPose();
    const auto forward = pose.Rot().RotateVector(ignition::math::Vector3d::UnitX);
    const double speed_mps = this->link_->WorldLinearVel().Dot(forward);
    const double speed_abs_mps = std::abs(speed_mps);
    const double target_speed = std::clamp(limited_cmd.linear.x, -this->max_speed_mps_, this->max_speed_mps_);
    const double available_power = this->AvailableDeployPower();

    double next_speed = speed_mps;
    if (target_speed > speed_mps) {
      const double aero_force = 0.5 * this->air_density_ * this->drag_coefficient_area_ * speed_abs_mps * speed_abs_mps;
      const double rolling_force = this->rolling_resistance_coefficient_ * this->vehicle_mass_kg_ * 9.80665;
      const double power_limited_force = speed_abs_mps < 2.0 ?
        this->vehicle_mass_kg_ * this->max_launch_accel_mps2_ :
        available_power / speed_abs_mps;
      const double accel = std::clamp(
        (power_limited_force - aero_force - rolling_force) / this->vehicle_mass_kg_,
        0.0,
        this->max_launch_accel_mps2_);
      next_speed = std::min(target_speed, speed_mps + accel * dt);
    } else if (target_speed < speed_mps) {
      next_speed = std::max(target_speed, speed_mps - this->max_brake_accel_mps2_ * dt);
    }

    limited_cmd.linear.x = next_speed;
    this->cmd_pub_->publish(limited_cmd);

    std_msgs::msg::String msg;
    msg.data = this->derate_reason_;
    this->derate_pub_->publish(msg);
  }

  double AvailableDeployPower()
  {
    this->derate_reason_ = "nominal";
    double available_power = this->max_deploy_power_w_;

    if (!this->has_battery_) {
      this->derate_reason_ = "battery_state_unavailable";
      return available_power;
    }

    if (this->battery_energy_wh_ <= this->reserve_energy_wh_) {
      this->derate_reason_ = "reserve_energy";
      return 0.0;
    }
    if (this->battery_energy_wh_ < this->reserve_energy_wh_ + this->reserve_buffer_wh_) {
      const double scale = (this->battery_energy_wh_ - this->reserve_energy_wh_) / std::max(1.0, this->reserve_buffer_wh_);
      available_power *= std::clamp(scale, 0.0, 1.0);
      this->derate_reason_ = "low_energy";
    }

    if (!std::isnan(this->battery_temperature_) && this->battery_temperature_ >= this->shutdown_temperature_) {
      this->derate_reason_ = "thermal_shutdown";
      return 0.0;
    }
    if (!std::isnan(this->battery_temperature_) && this->battery_temperature_ >= this->thermal_derate_temperature_) {
      const double span = std::max(1.0, this->shutdown_temperature_ - this->thermal_derate_temperature_);
      const double scale = (this->shutdown_temperature_ - this->battery_temperature_) / span;
      available_power *= std::clamp(scale, 0.0, 1.0);
      this->derate_reason_ = "thermal_derate";
    }

    return std::max(0.0, available_power);
  }

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  gazebo_ros::Node::SharedPtr ros_node_;
  event::ConnectionPtr update_connection_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr derate_pub_;

  common::Time last_update_time_;
  common::Time last_publish_time_;
  geometry_msgs::msg::Twist target_cmd_;
  bool has_cmd_{false};
  bool has_battery_{false};
  double update_period_{0.02};
  double vehicle_mass_kg_{788.0};
  double max_deploy_power_w_{12000.0};
  double reserve_energy_wh_{20.0};
  double reserve_buffer_wh_{60.0};
  double thermal_derate_temperature_{90.0};
  double shutdown_temperature_{105.0};
  double max_launch_accel_mps2_{8.0};
  double max_brake_accel_mps2_{12.0};
  double drag_coefficient_area_{1.35};
  double air_density_{1.225};
  double rolling_resistance_coefficient_{0.015};
  double max_speed_mps_{80.0};
  double battery_energy_wh_{0.0};
  double battery_temperature_{std::numeric_limits<double>::quiet_NaN()};
  std::string derate_reason_{"battery_state_unavailable"};
};

GZ_REGISTER_MODEL_PLUGIN(EnergyAwareAckermannGatePlugin)

}  // namespace gazebo

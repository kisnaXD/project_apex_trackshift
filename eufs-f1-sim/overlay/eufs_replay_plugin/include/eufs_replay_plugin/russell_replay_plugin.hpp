#ifndef EUFS_REPLAY_PLUGIN__RUSSELL_REPLAY_PLUGIN_HPP_
#define EUFS_REPLAY_PLUGIN__RUSSELL_REPLAY_PLUGIN_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_ros/node.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace eufs_replay_plugin {

struct ReplaySample {
  double time_s{0.0};
  double progress_m{0.0};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double speed_mps{0.0};
  double curvature_1pm{0.0};
  double acceleration_mps2{0.0};
};

class RussellReplayPlugin final : public gazebo::ModelPlugin {
 public:
  RussellReplayPlugin() = default;
  ~RussellReplayPlugin() override = default;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  void Reset() override;

 private:
  void OnUpdate();
  bool LoadReference(const std::string &path);
  bool SampleAt(double time_s, ReplaySample &sample) const;
  bool Finite(const ReplaySample &sample) const;
  void PublishTruth(const ReplaySample &sample, double sim_time_s, bool completed);

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_publisher_;
  gazebo::event::ConnectionPtr update_connection_;
  std::vector<ReplaySample> samples_;
  std::string reference_file_;
  std::string frame_id_{"map"};
  std::string child_frame_id_{"eufs2/base_link"};
  double phase_s_{0.0};
  double initial_phase_s_{0.0};
  double z_m_{0.0};
  double offset_x_m_{0.0};
  double offset_y_m_{0.0};
  bool loop_{false};
  bool initialized_{false};
  bool invalid_clock_{false};
  bool completed_{false};
  double last_sim_time_s_{0.0};
  unsigned int lap_count_{0};
};

}  // namespace eufs_replay_plugin

#endif  // EUFS_REPLAY_PLUGIN__RUSSELL_REPLAY_PLUGIN_HPP_

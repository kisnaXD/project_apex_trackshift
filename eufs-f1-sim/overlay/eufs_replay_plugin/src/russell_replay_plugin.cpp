#include "eufs_replay_plugin/russell_replay_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include <gazebo/gazebo.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <yaml-cpp/yaml.h>

namespace eufs_replay_plugin {
namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kPi = 3.14159265358979323846;

double AngleDelta(double start, double end) {
  return std::remainder(end - start, 2.0 * kPi);
}

double Required(const YAML::Node &row, const char *name) {
  if (!row[name]) {
    throw std::runtime_error(std::string("reference sample is missing ") + name);
  }
  const double value = row[name].as<double>();
  if (!std::isfinite(value)) {
    throw std::runtime_error(std::string("reference field is not finite: ") + name);
  }
  return value;
}

}  // namespace

GZ_REGISTER_MODEL_PLUGIN(RussellReplayPlugin)

void RussellReplayPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  model_ = model;
  world_ = model_->GetWorld();
  ros_node_ = gazebo_ros::Node::Get(sdf);
  if (!model_->IsStatic()) {
    RCLCPP_FATAL(ros_node_->get_logger(), "eufs_replay_plugin requires a static model; refusing dynamic playback");
    return;
  }
  if (!sdf->HasElement("reference_file")) {
    RCLCPP_FATAL(ros_node_->get_logger(), "eufs_replay_plugin requires <reference_file>");
    return;
  }
  reference_file_ = sdf->Get<std::string>("reference_file");
  if (sdf->HasElement("reference_phase_s")) phase_s_ = sdf->Get<double>("reference_phase_s");
  if (sdf->HasElement("frame_id")) frame_id_ = sdf->Get<std::string>("frame_id");
  if (sdf->HasElement("child_frame_id")) child_frame_id_ = sdf->Get<std::string>("child_frame_id");
  if (sdf->HasElement("z_m")) z_m_ = sdf->Get<double>("z_m");
  if (sdf->HasElement("offset_x_m")) offset_x_m_ = sdf->Get<double>("offset_x_m");
  if (sdf->HasElement("offset_y_m")) offset_y_m_ = sdf->Get<double>("offset_y_m");
  if (sdf->HasElement("loop")) loop_ = sdf->Get<bool>("loop");
  if (!LoadReference(reference_file_)) return;
  if (!std::isfinite(phase_s_) || !std::isfinite(z_m_) || !std::isfinite(offset_x_m_) ||
      !std::isfinite(offset_y_m_) || phase_s_ < 0.0 || phase_s_ > samples_.back().time_s) {
    RCLCPP_FATAL(ros_node_->get_logger(), "reference_phase_s is outside the frozen replay interval");
    samples_.clear();
    return;
  }
  initial_phase_s_ = phase_s_;
  const std::string topic = sdf->HasElement("truth_topic")
      ? sdf->Get<std::string>("truth_topic") : "/eufs2/replay_truth";
  truth_publisher_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(topic, rclcpp::QoS(10));
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&RussellReplayPlugin::OnUpdate, this));
  RCLCPP_INFO(ros_node_->get_logger(), "Loaded exact kinematic replay (%zu knots) for %s",
              samples_.size(), model_->GetName().c_str());
}

bool RussellReplayPlugin::LoadReference(const std::string &path) {
  try {
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node rows = root["samples"];
    if (!rows || !rows.IsSequence() || rows.size() < 2) {
      throw std::runtime_error("reference samples must contain at least two knots");
    }
    samples_.reserve(rows.size());
    double previous_time = -std::numeric_limits<double>::infinity();
    double previous_progress = -std::numeric_limits<double>::infinity();
    for (const YAML::Node &row : rows) {
      ReplaySample sample;
      sample.time_s = Required(row, "time_s");
      sample.progress_m = Required(row, "progress_m");
      sample.x_m = Required(row, "x_m");
      sample.y_m = Required(row, "y_m");
      sample.yaw_rad = Required(row, "yaw_rad");
      sample.speed_mps = Required(row, "speed_mps");
      sample.curvature_1pm = Required(row, "curvature_1pm");
      sample.acceleration_mps2 = row["acceleration_mps2"] ? row["acceleration_mps2"].as<double>() : 0.0;
      if (!Finite(sample) || sample.speed_mps < 0.0 || sample.time_s <= previous_time ||
          sample.progress_m < previous_progress) {
        throw std::runtime_error("reference timestamps/progress/fields are invalid");
      }
      previous_time = sample.time_s;
      previous_progress = sample.progress_m;
      samples_.push_back(sample);
    }
    if (std::abs(samples_.front().time_s) > kEpsilon || samples_.back().time_s <= kEpsilon) {
      throw std::runtime_error("reference time must start at zero and have positive duration");
    }
    return true;
  } catch (const std::exception &error) {
    RCLCPP_FATAL(ros_node_->get_logger(), "Unable to load frozen replay %s: %s", path.c_str(), error.what());
    samples_.clear();
    return false;
  }
}

bool RussellReplayPlugin::Finite(const ReplaySample &sample) const {
  return std::isfinite(sample.time_s) && std::isfinite(sample.progress_m) &&
         std::isfinite(sample.x_m) && std::isfinite(sample.y_m) && std::isfinite(sample.yaw_rad) &&
         std::isfinite(sample.speed_mps) && std::isfinite(sample.curvature_1pm) &&
         std::isfinite(sample.acceleration_mps2);
}

bool RussellReplayPlugin::SampleAt(double time_s, ReplaySample &sample) const {
  if (samples_.empty() || !std::isfinite(time_s) || time_s < samples_.front().time_s - kEpsilon ||
      time_s > samples_.back().time_s + kEpsilon) return false;
  auto right = std::lower_bound(samples_.begin(), samples_.end(), time_s,
      [](const ReplaySample &item, double value) { return item.time_s < value; });
  if (right == samples_.begin() || right == samples_.end() || std::abs(right->time_s - time_s) <= kEpsilon) {
    sample = (right == samples_.end()) ? samples_.back() : *right;
    return true;
  }
  const ReplaySample &left_sample = *(right - 1);
  const ReplaySample &right_sample = *right;
  const double dt = right_sample.time_s - left_sample.time_s;
  const double tau = time_s - left_sample.time_s;
  const double acceleration = (right_sample.speed_mps - left_sample.speed_mps) / dt;
  const double segment_progress = std::max(0.0, right_sample.progress_m - left_sample.progress_m);
  const double traveled = std::max(0.0, left_sample.speed_mps * tau + 0.5 * acceleration * tau * tau);
  const double alpha = segment_progress > kEpsilon ? std::min(1.0, traveled / segment_progress) : tau / dt;
  sample = left_sample;
  sample.time_s = time_s;
  sample.progress_m = left_sample.progress_m + alpha * (right_sample.progress_m - left_sample.progress_m);
  sample.x_m = left_sample.x_m + alpha * (right_sample.x_m - left_sample.x_m);
  sample.y_m = left_sample.y_m + alpha * (right_sample.y_m - left_sample.y_m);
  sample.yaw_rad = left_sample.yaw_rad + alpha * AngleDelta(left_sample.yaw_rad, right_sample.yaw_rad);
  sample.speed_mps = left_sample.speed_mps + acceleration * tau;
  sample.curvature_1pm = left_sample.curvature_1pm + alpha * (right_sample.curvature_1pm - left_sample.curvature_1pm);
  // Acceleration is authored on the knot for its outgoing interval.
  sample.acceleration_mps2 = left_sample.acceleration_mps2;
  return true;
}

void RussellReplayPlugin::OnUpdate() {
  if (samples_.empty() || invalid_clock_) return;
  const double sim_time = world_->SimTime().Double();
  if (!std::isfinite(sim_time)) return;
  if (initialized_ && sim_time + kEpsilon < last_sim_time_s_) {
    invalid_clock_ = true;
    RCLCPP_ERROR(ros_node_->get_logger(), "simulation time moved backwards; reset replay explicitly");
    return;
  }
  if (!initialized_) {
    initialized_ = true;
    last_sim_time_s_ = sim_time;
  }
  const double elapsed = sim_time - last_sim_time_s_;
  last_sim_time_s_ = sim_time;
  if (!completed_ || loop_) {
    phase_s_ += std::max(0.0, elapsed);
    if (loop_) {
      while (phase_s_ > samples_.back().time_s) {
        phase_s_ -= samples_.back().time_s;
        ++lap_count_;
      }
      completed_ = lap_count_ > 0;
    } else if (phase_s_ >= samples_.back().time_s) {
      phase_s_ = samples_.back().time_s;
      completed_ = true;
    }
  }
  ReplaySample sample;
  if (!SampleAt(phase_s_, sample)) return;
  const double yaw = sample.yaw_rad;
  model_->SetWorldPose(ignition::math::Pose3d(sample.x_m + offset_x_m_, sample.y_m + offset_y_m_, z_m_, 0.0, 0.0, yaw));
  PublishTruth(sample, sim_time, completed_ && !loop_);
}

void RussellReplayPlugin::PublishTruth(const ReplaySample &sample, double sim_time_s, bool completed) {
  if (!truth_publisher_) return;
  const double speed = completed ? 0.0 : sample.speed_mps;
  nav_msgs::msg::Odometry message;
  message.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(sim_time_s * 1e9)));
  message.header.frame_id = frame_id_;
  message.child_frame_id = child_frame_id_;
  message.pose.pose.position.x = sample.x_m + offset_x_m_;
  message.pose.pose.position.y = sample.y_m + offset_y_m_;
  message.pose.pose.position.z = z_m_;
  message.pose.pose.orientation.z = std::sin(sample.yaw_rad * 0.5);
  message.pose.pose.orientation.w = std::cos(sample.yaw_rad * 0.5);
  // Twist is expressed in child_frame_id (the body frame), unlike the
  // world-frame pose coordinates above.
  message.twist.twist.linear.x = speed;
  message.twist.twist.linear.y = 0.0;
  message.twist.twist.angular.z = completed ? 0.0 : speed * sample.curvature_1pm;
  truth_publisher_->publish(message);
}

void RussellReplayPlugin::Reset() {
  phase_s_ = initial_phase_s_;
  initialized_ = false;
  invalid_clock_ = false;
  completed_ = false;
  last_sim_time_s_ = 0.0;
  lap_count_ = 0;
}

}  // namespace eufs_replay_plugin

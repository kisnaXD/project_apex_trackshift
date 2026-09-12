#include "eufs_hybrid_model/model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace eufs_hybrid_model {
namespace {
double clamp(double value, double low, double high) { return std::max(low, std::min(high, value)); }
double finite_or(double value, double fallback) { return std::isfinite(value) ? value : fallback; }
double force_limit(double force, double power, double speed, double minimum_speed) {
  return std::min(std::abs(force), power / std::max(std::abs(speed), minimum_speed));
}
double thermal_step(double temperature, double ambient, double heat_w, double cooling_w_per_k, double capacity_j_per_k, double dt_s) {
  if (dt_s == 0.0) return temperature;
  if (cooling_w_per_k <= 0.0) return temperature + heat_w * dt_s / std::max(1.0, capacity_j_per_k);
  const double decay = std::exp(-cooling_w_per_k * dt_s / std::max(1.0, capacity_j_per_k));
  return ambient + (temperature - ambient) * decay + heat_w / cooling_w_per_k * (1.0 - decay);
}
void validate_profile(const Profile &p) {
  if (!std::isfinite(p.mass_kg) || p.mass_kg <= 0.0 || !std::isfinite(p.wheel_radius_m) || p.wheel_radius_m <= 0.0 || !std::isfinite(p.gravity_mps2) || p.gravity_mps2 <= 0.0 || !std::isfinite(p.ambient_temperature_k) || p.ambient_temperature_k < 0.0 || !std::isfinite(p.battery_capacity_j) || p.battery_capacity_j <= 0.0 || !std::isfinite(p.reserve_energy_j) || p.reserve_energy_j < 0.0 || p.reserve_energy_j > p.battery_capacity_j || !std::isfinite(p.mguk_efficiency) || p.mguk_efficiency <= 0.0 || p.mguk_efficiency > 1.0 || !std::isfinite(p.regen_efficiency) || p.regen_efficiency <= 0.0 || p.regen_efficiency > 1.0 || !std::isfinite(p.thermal_derate_start_k) || !std::isfinite(p.max_temperature_k) || p.max_temperature_k <= 0.0 || p.thermal_derate_start_k >= p.max_temperature_k || !std::isfinite(p.min_force_speed_mps) || p.min_force_speed_mps <= 0.0) throw std::invalid_argument("invalid hybrid profile");
  const double nonnegative[] = {p.drag_coefficient_n_per_mps2, p.rolling_resistance_n, p.ice_max_force_n, p.ice_max_power_w, p.mguk_max_force_n, p.mguk_max_power_w, p.mguk_regen_force_n, p.mguk_regen_power_w, p.auxiliary_power_w, p.thermal_capacity_j_per_k, p.cooling_w_per_k, p.max_deploy_per_lap_j, p.max_recovery_per_lap_j, p.tyre_normal_force_n, p.tyre_mu, p.tyre_temp_band_k, p.tyre_heat_capacity_j_per_k, p.tyre_cooling_w_per_k, p.wear_rate_per_j, p.downforce_coefficient_n_per_mps2};
  for (double value : nonnegative) if (!std::isfinite(value) || value < 0.0) throw std::invalid_argument("invalid hybrid profile limit");
  if (!std::isfinite(p.tyre_temp_optimum_k)) throw std::invalid_argument("invalid tyre temperature profile");
}
void validate_state(const Profile &p, const State &s) {
  if (!std::isfinite(s.stored_energy_j) || s.stored_energy_j < 0.0 || s.stored_energy_j > p.battery_capacity_j || !std::isfinite(s.temperature_k) || !std::isfinite(s.lap_deployed_j) || !std::isfinite(s.lap_recovered_j) || s.lap_deployed_j < 0.0 || s.lap_recovered_j < 0.0) throw std::invalid_argument("invalid hybrid state");
  for (double wear : s.tyre_wear) if (!std::isfinite(wear) || wear < 0.0 || wear > 1.0) throw std::invalid_argument("invalid tyre wear");
  for (double temperature : s.tyre_temperature_k) if (!std::isfinite(temperature)) throw std::invalid_argument("invalid tyre temperature");
}
}

double grip_scale(const Profile &p, const State &s) {
  double worst_grip = 1.0;
  for (std::size_t i = 0; i < s.tyre_wear.size(); ++i) {
    const double tyre_temp = clamp(
        1.0 - std::abs(s.tyre_temperature_k[i] - p.tyre_temp_optimum_k) /
            std::max(1.0, p.tyre_temp_band_k),
        0.0, 1.0);
    worst_grip = std::min(worst_grip, std::max(0.0, (1.0 - s.tyre_wear[i]) * tyre_temp));
  }
  return clamp(worst_grip, 0.0, 1.0);
}

double lateral_force_limit(const Profile &p, const State & /*state*/, const double speed_mps,
                           const double grip) {
  const double normal = std::max(
      0.0, p.tyre_normal_force_n + p.downforce_coefficient_n_per_mps2 * speed_mps * speed_mps);
  return std::max(0.0, p.tyre_mu * clamp(grip, 0.0, 1.0) * normal);
}

StepResult step(const Profile &p, const State &initial, const Request &request,
                double speed_mps, double lateral_force_n, double dt_s) {
  StepResult out;
  out.state = initial;
  validate_profile(p);
  validate_state(p, initial);
  if (!std::isfinite(dt_s) || dt_s < 0.0) throw std::invalid_argument("dt_s must be finite and non-negative");
  if (!std::isfinite(speed_mps) || !std::isfinite(lateral_force_n) || !std::isfinite(request.acceleration_mps2) || !std::isfinite(request.requested_mguk_force_n)) throw std::invalid_argument("hybrid inputs must be finite");
  if (speed_mps < -1e-9) throw std::invalid_argument("reverse speed is unsupported by this forward benchmark model");
  const double requested = clamp(request.acceleration_mps2, -12.0, 8.0) * p.mass_kg;
  out.requested_tyre_force_n = requested;
  out.grip_scale = grip_scale(p, out.state);
  const double normal_limit = lateral_force_limit(p, out.state, speed_mps, out.grip_scale);
  const double circle = std::sqrt(std::max(0.0, std::pow(normal_limit, 2) - lateral_force_n * lateral_force_n));
  out.longitudinal_force_limit_n = circle;
  const double demand = clamp(requested, -circle, circle);
  const double usable = std::max(0.0, initial.stored_energy_j - p.reserve_energy_j);
  const double thermal_scale = initial.temperature_k <= p.thermal_derate_start_k ? 1.0 : clamp((p.max_temperature_k - initial.temperature_k) / std::max(1.0, p.max_temperature_k - p.thermal_derate_start_k), 0.0, 1.0);
  out.thermal_limited = thermal_scale < 1.0;
  const double drive_limit = force_limit(p.mguk_max_force_n, p.mguk_max_power_w * thermal_scale, speed_mps, p.min_force_speed_mps);
  const double regen_limit = force_limit(p.mguk_regen_force_n, p.mguk_regen_power_w * thermal_scale, speed_mps, p.min_force_speed_mps);
  out.available_deploy_power_w = (usable > 0.0 && initial.lap_deployed_j < p.max_deploy_per_lap_j) ? drive_limit * std::abs(speed_mps) : 0.0;
  out.available_recovery_power_w = (initial.stored_energy_j < p.battery_capacity_j && initial.lap_recovered_j < p.max_recovery_per_lap_j) ? regen_limit * std::abs(speed_mps) : 0.0;
  double mguk = clamp(finite_or(request.requested_mguk_force_n, 0.0), -regen_limit, drive_limit);
  if (demand > 0.0) mguk = clamp(mguk, 0.0, demand);
  else if (demand < 0.0) mguk = clamp(mguk, demand, 0.0);
  else mguk = 0.0;
  if (mguk > 0.0 && (usable <= 0.0 || initial.lap_deployed_j >= p.max_deploy_per_lap_j || thermal_scale <= 0.0)) mguk = 0.0;
  if (mguk < 0.0 && (initial.lap_recovered_j >= p.max_recovery_per_lap_j || initial.stored_energy_j >= p.battery_capacity_j || thermal_scale <= 0.0)) mguk = 0.0;
  const double mechanical_mguk = mguk * speed_mps;
  if (mechanical_mguk > 0.0) {
    if (usable <= 0.0) mguk = 0.0;
    else {
      const double deploy_energy = std::max(0.0, usable - p.auxiliary_power_w * dt_s);
      const double energy_force_limit = deploy_energy * p.mguk_efficiency / std::max(std::abs(speed_mps), p.min_force_speed_mps) / std::max(dt_s, 1e-9);
      mguk = std::min(mguk, std::max(0.0, energy_force_limit));
      const double lap_force_limit = std::max(0.0, p.max_deploy_per_lap_j - initial.lap_deployed_j) * p.mguk_efficiency / std::max(std::abs(speed_mps), p.min_force_speed_mps) / std::max(dt_s, 1e-9);
      mguk = std::min(mguk, lap_force_limit);
    }
  } else if (mechanical_mguk < 0.0) {
    const double lap_force_limit = std::max(0.0, p.max_recovery_per_lap_j - initial.lap_recovered_j) / std::max(p.regen_efficiency, 1e-6) / std::max(std::abs(speed_mps), p.min_force_speed_mps) / std::max(dt_s, 1e-9);
    mguk = std::max(mguk, -lap_force_limit);
    const double headroom = std::max(0.0, p.battery_capacity_j - initial.stored_energy_j);
    if (headroom <= 0.0) mguk = 0.0;
    else mguk = std::max(mguk, -headroom / std::max(p.regen_efficiency, 1e-6) / std::max(std::abs(speed_mps), p.min_force_speed_mps) / std::max(dt_s, 1e-9));
  }
  const double remainder = demand - mguk;
  const double ice_limit = force_limit(p.ice_max_force_n, p.ice_max_power_w, speed_mps, p.min_force_speed_mps);
  out.delivered_ice_force_n = clamp(std::max(0.0, remainder), 0.0, ice_limit);
  out.friction_brake_force_n = std::min(0.0, remainder - out.delivered_ice_force_n);
  out.delivered_mguk_force_n = mguk;
  out.delivered_tyre_force_n = out.delivered_ice_force_n + out.delivered_mguk_force_n + out.friction_brake_force_n;
  const double drag = p.drag_coefficient_n_per_mps2 * speed_mps * std::abs(speed_mps) + p.rolling_resistance_n * (speed_mps == 0.0 ? 0.0 : (speed_mps > 0.0 ? 1.0 : -1.0));
  out.net_acceleration_mps2 = (out.delivered_tyre_force_n - drag) / std::max(1e-9, p.mass_kg);
  const double mech = out.delivered_mguk_force_n * speed_mps;
  const double battery_rate_without_aux = mech >= 0.0 ? -mech / std::max(1e-6, p.mguk_efficiency) : -mech * p.regen_efficiency;
  const double available_aux = dt_s > 0.0 ? std::max(0.0, initial.stored_energy_j - p.reserve_energy_j) / dt_s + battery_rate_without_aux : 0.0;
  const double auxiliary_power = std::min(p.auxiliary_power_w, std::max(0.0, available_aux));
  out.delivered_auxiliary_power_w = auxiliary_power;
  out.electrical_power_w = mech >= 0.0 ? mech / std::max(1e-6, p.mguk_efficiency) + auxiliary_power : mech * p.regen_efficiency + auxiliary_power;
  const double requested_energy_delta = -out.electrical_power_w * dt_s;
  out.state.stored_energy_j = clamp(initial.stored_energy_j + requested_energy_delta, 0.0, p.battery_capacity_j);
  out.signed_energy_delta_j = out.state.stored_energy_j - initial.stored_energy_j;
  out.electrical_power_w = dt_s > 0.0 ? -out.signed_energy_delta_j / dt_s : 0.0;
  out.usable_energy_j = std::max(0.0, out.state.stored_energy_j - p.reserve_energy_j);
  if (mech > 0.0) out.state.lap_deployed_j += mech * dt_s / std::max(1e-6, p.mguk_efficiency);
  if (mech < 0.0) out.state.lap_recovered_j += -mech * dt_s * p.regen_efficiency;
  const double losses = mech >= 0.0 ? mech * (1.0 / std::max(1e-6, p.mguk_efficiency) - 1.0) : (-mech) * (1.0 - p.regen_efficiency);
  out.state.temperature_k = thermal_step(initial.temperature_k, p.ambient_temperature_k, losses + auxiliary_power, p.cooling_w_per_k, p.thermal_capacity_j_per_k, dt_s);
  const double tyre_heat = std::abs(out.delivered_tyre_force_n) * std::abs(speed_mps) * 0.02 + std::abs(lateral_force_n) * std::abs(speed_mps) * 0.02;
  for (std::size_t i = 0; i < out.state.tyre_temperature_k.size(); ++i) {
    out.state.tyre_temperature_k[i] = thermal_step(initial.tyre_temperature_k[i], p.ambient_temperature_k, tyre_heat / 4.0, p.tyre_cooling_w_per_k, p.tyre_heat_capacity_j_per_k, dt_s);
    out.state.tyre_wear[i] = clamp(initial.tyre_wear[i] + p.wear_rate_per_j * (std::abs(out.delivered_tyre_force_n) + std::abs(lateral_force_n)) * std::abs(speed_mps) * dt_s, 0.0, 1.0);
  }
  out.store_empty = out.state.stored_energy_j <= 1e-9;
  out.at_reserve = out.state.stored_energy_j <= p.reserve_energy_j + 1e-9;
  out.deployment_unavailable = out.at_reserve || (out.thermal_limited && thermal_scale <= 0.0) || initial.lap_deployed_j >= p.max_deploy_per_lap_j;
  out.capacity_empty = out.store_empty;
  out.derated = out.at_reserve || out.thermal_limited || out.grip_scale < 1.0 || std::abs(out.delivered_tyre_force_n - requested) > 1e-6;
  out.derate_reason = out.at_reserve ? "reserve" : (out.thermal_limited ? "thermal" : (out.grip_scale < 1.0 ? "grip" : (out.derated ? "force_limit" : "")));
  return out;
}

State advance_lap(const State &state, unsigned int new_lap_index) {
  if (new_lap_index <= state.lap_index) throw std::invalid_argument("lap index must strictly advance");
  State next = state;
  next.lap_index = new_lap_index;
  next.lap_deployed_j = 0.0;
  next.lap_recovered_j = 0.0;
  return next;
}
}  // namespace eufs_hybrid_model

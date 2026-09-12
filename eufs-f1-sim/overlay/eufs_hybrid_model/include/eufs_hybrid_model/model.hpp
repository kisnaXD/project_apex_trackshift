#pragma once

#include <array>

namespace eufs_hybrid_model {

struct Profile {
  double mass_kg{788.0};
  double drag_coefficient_n_per_mps2{0.8269};
  double rolling_resistance_n{120.0};
  double gravity_mps2{9.81};
  double wheel_radius_m{0.346};
  double ice_max_force_n{9000.0};
  double ice_max_power_w{120000.0};
  double mguk_max_force_n{5000.0};
  double mguk_max_power_w{120000.0};
  double mguk_regen_force_n{3500.0};
  double mguk_regen_power_w{80000.0};
  double mguk_efficiency{0.95};
  double regen_efficiency{0.70};
  double auxiliary_power_w{500.0};
  // Explicit synthetic benchmark store; no FIA or legacy-plugin calibration.
  double battery_capacity_j{4000000.0};
  double reserve_energy_j{400000.0};
  double ambient_temperature_k{298.15};
  double max_temperature_k{373.15};
  double thermal_capacity_j_per_k{150000.0};
  double cooling_w_per_k{100.0};
  double thermal_derate_start_k{360.0};
  double max_deploy_per_lap_j{4000000.0};
  double max_recovery_per_lap_j{2000000.0};
  double tyre_normal_force_n{788.0 * 9.81};
  double tyre_mu{1.60};
  double tyre_temp_optimum_k{363.15};
  double tyre_temp_band_k{80.0};
  double tyre_heat_capacity_j_per_k{30000.0};
  double tyre_cooling_w_per_k{25.0};
  double wear_rate_per_j{1e-10};
  double min_force_speed_mps{1.0};
  double downforce_coefficient_n_per_mps2{1.50};
};

struct State {
  double stored_energy_j{0.0};
  double temperature_k{298.15};
  std::array<double, 4> tyre_temperature_k{{363.15, 363.15, 363.15, 363.15}};
  std::array<double, 4> tyre_wear{{0.0, 0.0, 0.0, 0.0}};
  double lap_deployed_j{0.0};
  double lap_recovered_j{0.0};
  unsigned int lap_index{0};
};

struct Request {
  double acceleration_mps2{0.0};  // tyre force / mass, before drag
  double requested_mguk_force_n{0.0};
};

struct StepResult {
  State state{};
  double requested_tyre_force_n{0.0};
  double delivered_ice_force_n{0.0};
  double delivered_mguk_force_n{0.0};
  double friction_brake_force_n{0.0};
  double delivered_tyre_force_n{0.0};
  double net_acceleration_mps2{0.0};
  double electrical_power_w{0.0};  // positive discharges the store
  double delivered_auxiliary_power_w{0.0};
  double available_deploy_power_w{0.0};
  double available_recovery_power_w{0.0};
  double signed_energy_delta_j{0.0};
  double usable_energy_j{0.0};
  double grip_scale{1.0};
  double longitudinal_force_limit_n{0.0};
  bool derated{false};
  bool capacity_empty{false};
  bool store_empty{false};
  bool at_reserve{false};
  bool deployment_unavailable{false};
  bool thermal_limited{false};
  const char *derate_reason{""};
};

StepResult step(const Profile &profile, const State &state, const Request &request,
                double speed_mps, double lateral_force_n, double dt_s);

// Shared tyre-state helpers.  The native Gazebo adapter uses these before
// allocation so that the force-circle and the vehicle dynamics see exactly
// the same grip and normal-force values as the pure model.
double grip_scale(const Profile &profile, const State &state);
double lateral_force_limit(const Profile &profile, const State &state, double speed_mps,
                           double grip);

State advance_lap(const State &state, unsigned int new_lap_index);

}  // namespace eufs_hybrid_model

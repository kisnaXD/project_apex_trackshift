#include "eufs_hybrid_model/model.hpp"
#include <iomanip>
#include <iostream>

int main() {
  using namespace eufs_hybrid_model;
  Profile profile;
  State state;
  state.stored_energy_j = profile.battery_capacity_j;
  state.temperature_k = profile.ambient_temperature_k;
  const Request requests[] = {{2.0, 1500.0}, {-2.0, -1500.0}, {0.0, 0.0}};
  const double speeds[] = {20.0, 20.0, 0.0};
  std::cout << std::setprecision(17);
  for (std::size_t i = 0; i < 3; ++i) {
    const auto result = step(profile, state, requests[i], speeds[i], 0.0, 0.01);
    std::cout << i << " " << result.delivered_ice_force_n << " "
              << result.delivered_mguk_force_n << " " << result.friction_brake_force_n
              << " " << result.signed_energy_delta_j << " "
              << result.state.stored_energy_j << " " << result.state.temperature_k
              << " " << result.state.tyre_temperature_k[0] << "\n";
    state = result.state;
  }
  State empty = state;
  empty.stored_energy_j = profile.reserve_energy_j;
  const auto empty_result = step(profile, empty, Request{1.0, 5000.0}, 20.0, 0.0, 0.1);
  std::cout << "empty " << empty_result.delivered_mguk_force_n << " " << empty_result.signed_energy_delta_j << "\n";
  State hot = state;
  hot.temperature_k = profile.max_temperature_k + 1.0;
  const auto hot_result = step(profile, hot, Request{2.0, 5000.0}, 20.0, 0.0, 0.1);
  std::cout << "hot " << hot_result.delivered_mguk_force_n << " " << hot_result.grip_scale << " " << hot_result.state.temperature_k << "\n";
  State quota = state;
  quota.lap_deployed_j = profile.max_deploy_per_lap_j;
  const auto quota_result = step(profile, quota, Request{1.0, 5000.0}, 0.0, 0.0, 0.1);
  std::cout << "quota " << quota_result.delivered_mguk_force_n << " " << quota_result.deployment_unavailable << "\n";
  State worn = state;
  worn.tyre_wear = {{1.0, 1.0, 1.0, 1.0}};
  const auto worn_result = step(profile, worn, Request{2.0, 5000.0}, 20.0, 0.0, 0.1);
  std::cout << "worn " << worn_result.grip_scale << " " << worn_result.longitudinal_force_limit_n << "\n";
  return 0;
}

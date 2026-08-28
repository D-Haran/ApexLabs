#pragma once

#include <filesystem>
#include <vector>

#include "apexlab/lap_simulation.hpp"

namespace apexlab {

struct OptimizationTrajectoryNode {
    double s_m{0.0};
    double time_s{0.0};
    double speed_mps{0.0};
    double steering_angle_rad{0.0};
    double sideslip_rad{0.0};
    double lateral_offset_m{0.0};
    double path_heading_rad{0.0};
    double path_curvature_1_m{0.0};
    double throttle{0.0};
    double brake{0.0};
};

class OptimizationTrajectory {
  public:
    OptimizationTrajectory(double track_length_m, std::vector<OptimizationTrajectoryNode> nodes);
    [[nodiscard]] OptimizationTrajectoryNode at_s(double s_m) const;
    [[nodiscard]] const std::vector<OptimizationTrajectoryNode>& nodes() const { return nodes_; }

  private:
    double length_m_{0.0};
    std::vector<OptimizationTrajectoryNode> nodes_;
};

[[nodiscard]] OptimizationTrajectory
load_optimization_trajectory(const std::filesystem::path& path, double track_length_m);

struct OptimizationReplayMetrics {
    double maximum_position_error_m{0.0};
    double maximum_speed_error_mps{0.0};
    double maximum_yaw_error_rad{0.0};
    double maximum_tire_utilization{0.0};
    double maximum_track_violation_m{0.0};
    double maximum_steering_correction_rad{0.0};
    double maximum_longitudinal_command_correction{0.0};
    double rms_position_error_m{0.0};
    double rms_speed_error_mps{0.0};
    double rms_yaw_error_rad{0.0};
};

struct OptimizationReplayResult {
    LapSimulationResult lap;
    OptimizationReplayMetrics metrics;
};

[[nodiscard]] OptimizationReplayResult
replay_optimization_trajectory(const NonlinearPlanarVehicleModel& model,
                               const PeriodicTrack& track,
                               const OptimizationTrajectory& trajectory,
                               const LapSimulationOptions& options = {});

} // namespace apexlab

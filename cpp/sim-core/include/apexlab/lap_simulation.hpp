#pragma once

#include <filesystem>
#include <span>
#include <vector>

#include "apexlab/nonlinear_planar_model.hpp"
#include "apexlab/road_vehicle_model.hpp"
#include "apexlab/simulation.hpp"
#include "apexlab/track.hpp"

namespace apexlab {

inline constexpr int lap_telemetry_schema_version = 4;

class ReferenceLine {
  public:
    explicit ReferenceLine(const PeriodicTrack& track, double constant_offset_m = 0.0);
    [[nodiscard]] TrackFrame frame_at_s(double s_m) const;
    [[nodiscard]] double offset_m() const { return offset_m_; }

  private:
    const PeriodicTrack* track_;
    double offset_m_;
};

struct SpeedProfileOptions {
    double lateral_acceleration_limit_mps2{9.0};
    double acceleration_limit_mps2{3.5};
    double braking_limit_mps2{7.0};
    double maximum_speed_mps{55.0};
    double safety_factor{0.82};
    double sample_spacing_m{1.0};
};

class SpeedProfile {
  public:
    SpeedProfile(const PeriodicTrack& track, const ReferenceLine& line,
                 SpeedProfileOptions options = {});
    [[nodiscard]] double target_speed_mps(double s_m) const;
    [[nodiscard]] double spacing_m() const { return spacing_m_; }
    [[nodiscard]] std::span<const double> values() const { return values_; }

  private:
    double length_m_{0.0};
    double spacing_m_{0.0};
    std::vector<double> values_;
};

struct PathControllerGains {
    double lateral_gain{0.045};
    double heading_gain{0.85};
    double speed_gain{0.20};
    double maximum_steering_rad{0.32};
};

struct LapSimulationOptions {
    Time timestep{si::seconds(0.005)};
    Time controller_timestep{si::seconds(0.02)};
    double initial_speed_mps{8.0};
    int warmup_laps{1};
    int timed_laps{1};
    double maximum_duration_s{300.0};
    IntegratorKind integrator{IntegratorKind::rk4};
    PathControllerGains controller;
};

class DirectedLapProgress {
  public:
    DirectedLapProgress(double track_length_m, double initial_s_m = 0.0);
    void update(double wrapped_s_m);
    [[nodiscard]] double unwrapped_s_m() const { return unwrapped_s_m_; }
    [[nodiscard]] int completed_laps() const { return completed_laps_; }

  private:
    double length_m_;
    double last_s_m_;
    double unwrapped_s_m_{0.0};
    int completed_laps_{0};
};

struct LapTelemetrySample {
    int schema_version{lap_telemetry_schema_version};
    Time time{si::seconds(0.0)};
    std::uint64_t step{0};
    PlanarState state;
    PlanarControl input;
    NonlinearPlanarForces forces;
    double speed_mps{0.0};
    double longitudinal_acceleration_mps2{0.0};
    double lateral_acceleration_mps2{0.0};
    double track_s_m{0.0};
    double unwrapped_track_s_m{0.0};
    double track_progress_fraction{0.0};
    double lateral_error_m{0.0};
    double heading_error_rad{0.0};
    double reference_curvature_1_m{0.0};
    double target_speed_mps{0.0};
    double speed_error_mps{0.0};
    int lap_number{0};
    double lap_elapsed_time_s{0.0};
    int sector_index{0};
    bool on_track{true};
    std::optional<SprungState> chassis;
    std::array<double, 4> suspension_compression_m{};
    Vec3 gravity_body;
};

struct LapSimulationResult {
    std::vector<LapTelemetrySample> samples;
    std::vector<double> completed_lap_times_s;
    std::vector<double> completed_sector_times_s;
    bool completed{false};
};

[[nodiscard]] LapSimulationResult simulate_laps(const NonlinearPlanarVehicleModel& model,
                                                const PeriodicTrack& track,
                                                const ReferenceLine& line,
                                                const SpeedProfile& speed_profile,
                                                const LapSimulationOptions& options = {});

void write_lap_telemetry_csv(const std::filesystem::path& path,
                             std::span<const LapTelemetrySample> samples);

struct SpatialTelemetrySample {
    double s_m{0.0};
    double speed_mps{0.0};
    double throttle{0.0};
    double brake{0.0};
    double steering_rad{0.0};
    double lateral_acceleration_mps2{0.0};
    double maximum_tire_utilization{0.0};
    double lateral_error_m{0.0};
};

[[nodiscard]] std::vector<SpatialTelemetrySample>
resample_lap_spatially(std::span<const LapTelemetrySample> samples, int lap_number,
                       double track_length_m, double spacing_m);

} // namespace apexlab

#include "apexlab/optimization_replay.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace apexlab {
namespace {

[[nodiscard]] double wrap_angle(double value) {
    return std::remainder(value, 2.0 * std::numbers::pi);
}

[[nodiscard]] double maximum_utilization(const NonlinearPlanarForces& forces) {
    double result = 0.0;
    for (const auto& wheel : forces.wheel) result = std::max(result, wheel.friction_utilization);
    return result;
}

[[nodiscard]] std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> values;
    std::stringstream stream{line};
    std::string value;
    while (std::getline(stream, value, ',')) values.push_back(value);
    return values;
}

} // namespace

OptimizationTrajectory::OptimizationTrajectory(double track_length_m,
                                               std::vector<OptimizationTrajectoryNode> nodes)
    : length_m_(track_length_m), nodes_(std::move(nodes)) {
    if (!(length_m_ > 0.0) || nodes_.size() < 20U) {
        throw std::invalid_argument("optimization trajectory requires a positive track and at least 20 nodes");
    }
    std::sort(nodes_.begin(), nodes_.end(),
              [](const auto& left, const auto& right) { return left.s_m < right.s_m; });
    for (std::size_t index = 0; index < nodes_.size(); ++index) {
        const auto& node = nodes_[index];
        if (!std::isfinite(node.s_m) || !std::isfinite(node.speed_mps) ||
            !std::isfinite(node.steering_angle_rad) || !std::isfinite(node.sideslip_rad) ||
            !std::isfinite(node.lateral_offset_m) ||
            !std::isfinite(node.throttle) || !std::isfinite(node.brake) || node.s_m < 0.0 ||
            node.s_m >= length_m_ || node.speed_mps <= 0.0 || node.throttle < 0.0 ||
            node.throttle > 1.0 || node.brake < 0.0 || node.brake > 1.0 ||
            (index > 0U && node.s_m <= nodes_[index - 1U].s_m)) {
            throw std::invalid_argument("optimization trajectory contains invalid or unordered values");
        }
    }
}

OptimizationTrajectoryNode OptimizationTrajectory::at_s(double s_m) const {
    double wrapped = std::fmod(s_m, length_m_);
    if (wrapped < 0.0) wrapped += length_m_;
    const auto upper = std::upper_bound(
        nodes_.begin(), nodes_.end(), wrapped,
        [](double value, const OptimizationTrajectoryNode& node) { return value < node.s_m; });
    const OptimizationTrajectoryNode* left = nullptr;
    const OptimizationTrajectoryNode* right = nullptr;
    double left_s = 0.0;
    double right_s = 0.0;
    if (upper == nodes_.begin() || upper == nodes_.end()) {
        left = &nodes_.back();
        right = &nodes_.front();
        left_s = left->s_m;
        right_s = right->s_m + length_m_;
        if (wrapped < nodes_.front().s_m) wrapped += length_m_;
    } else {
        left = &*std::prev(upper);
        right = &*upper;
        left_s = left->s_m;
        right_s = right->s_m;
    }
    const double fraction = (wrapped - left_s) / (right_s - left_s);
    const auto mix = [fraction](double a, double b) { return a + fraction * (b - a); };
    const auto mix_angle = [fraction](double a, double b) {
        return a + fraction * std::remainder(b - a, 2.0 * std::numbers::pi);
    };
    const double path_heading = std::isfinite(left->path_heading_rad) &&
                                        std::isfinite(right->path_heading_rad)
                                    ? mix_angle(left->path_heading_rad, right->path_heading_rad)
                                    : std::numeric_limits<double>::quiet_NaN();
    return {std::fmod(mix(left_s, right_s), length_m_), mix(left->time_s, right->time_s),
            mix(left->speed_mps, right->speed_mps),
            mix(left->steering_angle_rad, right->steering_angle_rad),
            mix(left->sideslip_rad, right->sideslip_rad),
            mix(left->lateral_offset_m, right->lateral_offset_m), path_heading,
            mix(left->path_curvature_1_m, right->path_curvature_1_m),
            mix(left->throttle, right->throttle), mix(left->brake, right->brake)};
}

OptimizationTrajectory load_optimization_trajectory(const std::filesystem::path& path,
                                                      double track_length_m) {
    std::ifstream stream{path};
    if (!stream) throw std::runtime_error("cannot read optimization trajectory: " + path.string());
    std::string line;
    if (!std::getline(stream, line)) throw std::runtime_error("optimization trajectory is empty");
    const auto header = split(line);
    std::unordered_map<std::string, std::size_t> columns;
    for (std::size_t index = 0; index < header.size(); ++index) columns.emplace(header[index], index);
    for (const char* required : {"s_m", "time_s", "speed_m_s", "steering_angle_rad",
                                 "sideslip_rad", "throttle", "brake"}) {
        if (!columns.contains(required))
            throw std::runtime_error("optimization trajectory missing column: " + std::string{required});
    }
    std::vector<OptimizationTrajectoryNode> nodes;
    while (std::getline(stream, line)) {
        if (line.empty()) continue;
        const auto values = split(line);
        if (values.size() != header.size()) throw std::runtime_error("malformed optimization trajectory row");
        const auto number = [&](const char* name) { return std::stod(values.at(columns.at(name))); };
        const auto optional_number = [&](const char* name, double fallback) {
            return columns.contains(name) ? number(name) : fallback;
        };
        nodes.push_back({number("s_m"), number("time_s"), number("speed_m_s"),
                         number("steering_angle_rad"), number("sideslip_rad"),
                         optional_number("lateral_offset_m", 0.0),
                         optional_number("path_heading_rad",
                                         std::numeric_limits<double>::quiet_NaN()),
                         optional_number("path_curvature_1_m", 0.0),
                         number("throttle"), number("brake")});
    }
    return OptimizationTrajectory{track_length_m, std::move(nodes)};
}

OptimizationReplayResult
replay_optimization_trajectory(const NonlinearPlanarVehicleModel& model,
                               const PeriodicTrack& track,
                               const OptimizationTrajectory& trajectory,
                               const LapSimulationOptions& options) {
    if (options.timestep.value() <= 0.0 || options.maximum_duration_s <= 0.0) {
        throw std::invalid_argument("invalid optimization replay options");
    }
    const OptimizationTrajectoryNode initial = trajectory.at_s(0.0);
    const TrackFrame initial_frame = track.frame_at_s(0.0);
    const Vec2 initial_position = track.position_at_s(0.0, initial.lateral_offset_m);
    const double initial_heading = std::isfinite(initial.path_heading_rad)
                                       ? initial.path_heading_rad
                                       : initial_frame.heading_rad;
    PlanarState state{
        si::meters(initial_position.x), si::meters(initial_position.y),
        si::radians(initial_heading - std::atan(initial.sideslip_rad)),
        si::meters_per_second(initial.speed_mps),
        si::meters_per_second(initial.sideslip_rad * initial.speed_mps),
        si::radians_per_second(initial.speed_mps *
                               (std::isfinite(initial.path_curvature_1_m)
                                    ? initial.path_curvature_1_m
                                    : initial_frame.curvature_1_m)),
    };
    OptimizationReplayResult result;
    result.lap.samples.reserve(
        static_cast<std::size_t>(options.maximum_duration_s / options.timestep.value()) + 1U);
    DirectedLapProgress progress{track.length_m()};
    std::optional<double> projection_hint = 0.0;
    double time_s = 0.0;
    double sector_start_time_s = 0.0;
    int sector_index = 0;
    double squared_position_error = 0.0;
    double squared_speed_error = 0.0;
    double squared_yaw_error = 0.0;
    std::size_t error_samples = 0U;
    const auto& sectors = track.definition().sector_boundaries_fraction;

    const auto tracking_control = [&](const PlanarState& control_state,
                                      const TrackCoordinate& coordinate) {
        const TrackFrame frame = track.frame_at_s(coordinate.s_m);
        const OptimizationTrajectoryNode target = trajectory.at_s(coordinate.s_m);
        const double path_heading = std::isfinite(target.path_heading_rad)
                                        ? target.path_heading_rad
                                        : frame.heading_rad;
        const double target_yaw = path_heading - std::atan(target.sideslip_rad);
        const double yaw_error = wrap_angle(control_state.yaw.value() - target_yaw);
        const double lateral_error = coordinate.lateral_offset_m - target.lateral_offset_m;
        const double steering =
            target.steering_angle_rad - options.controller.lateral_gain * lateral_error -
            options.controller.heading_gain * yaw_error;
        const double speed = std::hypot(control_state.velocity_x.value(),
                                        control_state.velocity_y.value());
        const double feed_forward = target.throttle - target.brake;
        const double signed_command = std::clamp(
            feed_forward + options.controller.speed_gain * (target.speed_mps - speed), -1.0, 1.0);
        return PlanarControl{
            si::radians(std::clamp(steering, -options.controller.maximum_steering_rad,
                                   options.controller.maximum_steering_rad)),
            std::max(0.0, signed_command), std::max(0.0, -signed_command)};
    };

    for (std::uint64_t step = 0; time_s <= options.maximum_duration_s; ++step) {
        const TrackCoordinate coordinate = track.project(
            {state.position_x.value(), state.position_y.value()}, projection_hint, 100.0);
        projection_hint = coordinate.s_m;
        if (step > 0U) progress.update(coordinate.s_m);
        const TrackFrame frame = track.frame_at_s(coordinate.s_m);
        const OptimizationTrajectoryNode target = trajectory.at_s(coordinate.s_m);
        const PlanarControl control = tracking_control(state, coordinate);
        const NonlinearPlanarForces forces = model.forces(state, control);
        const double speed = std::hypot(state.velocity_x.value(), state.velocity_y.value());
        const double path_heading = std::isfinite(target.path_heading_rad)
                                        ? target.path_heading_rad
                                        : frame.heading_rad;
        const double target_yaw = path_heading - std::atan(target.sideslip_rad);
        const double lateral_error = coordinate.lateral_offset_m - target.lateral_offset_m;
        const double position_error = std::abs(lateral_error);
        const double speed_error = std::abs(speed - target.speed_mps);
        const double yaw_error = std::abs(wrap_angle(state.yaw.value() - target_yaw));
        const double heading_error = wrap_angle(state.yaw.value() - frame.heading_rad);
        const bool on_track = track.contains(coordinate.s_m, coordinate.lateral_offset_m);
        result.lap.samples.push_back(
            {lap_telemetry_schema_version, si::seconds(time_s), step, state, control, forces, speed,
             forces.body_longitudinal_force.value() / model.parameters().mass.value(),
             forces.body_lateral_force.value() / model.parameters().mass.value(), coordinate.s_m,
             progress.unwrapped_s_m(), coordinate.s_m / track.length_m(), lateral_error,
             heading_error,
             std::isfinite(target.path_curvature_1_m) ? target.path_curvature_1_m
                                                      : frame.curvature_1_m,
             target.speed_mps, target.speed_mps - speed, 0,
             time_s, sector_index, on_track, std::nullopt, {}, {}});
        result.metrics.maximum_position_error_m =
            std::max(result.metrics.maximum_position_error_m, position_error);
        result.metrics.maximum_speed_error_mps =
            std::max(result.metrics.maximum_speed_error_mps, speed_error);
        result.metrics.maximum_yaw_error_rad =
            std::max(result.metrics.maximum_yaw_error_rad, yaw_error);
        squared_position_error += position_error * position_error;
        squared_speed_error += speed_error * speed_error;
        squared_yaw_error += yaw_error * yaw_error;
        ++error_samples;
        result.metrics.maximum_tire_utilization =
            std::max(result.metrics.maximum_tire_utilization, maximum_utilization(forces));
        result.metrics.maximum_steering_correction_rad =
            std::max(result.metrics.maximum_steering_correction_rad,
                     std::abs(control.steering_angle.value() - target.steering_angle_rad));
        result.metrics.maximum_longitudinal_command_correction =
            std::max(result.metrics.maximum_longitudinal_command_correction,
                     std::abs((control.throttle - control.brake) -
                              (target.throttle - target.brake)));
        if (!on_track) {
            const double violation = coordinate.lateral_offset_m > frame.left_width_m
                                         ? coordinate.lateral_offset_m - frame.left_width_m
                                         : -frame.right_width_m - coordinate.lateral_offset_m;
            result.metrics.maximum_track_violation_m =
                std::max(result.metrics.maximum_track_violation_m, violation);
        }
        if (progress.completed_laps() >= 1) {
            result.lap.completed = true;
            result.lap.completed_lap_times_s.push_back(time_s);
            if (!sectors.empty()) result.lap.completed_sector_times_s.push_back(time_s - sector_start_time_s);
            break;
        }
        if (!sectors.empty() && sector_index < static_cast<int>(sectors.size()) - 1 &&
            coordinate.s_m / track.length_m() >= sectors[static_cast<std::size_t>(sector_index)]) {
            result.lap.completed_sector_times_s.push_back(time_s - sector_start_time_s);
            sector_start_time_s = time_s;
            ++sector_index;
        }
        const auto derivative = [&](const PlanarState& stage, Time) {
            const TrackCoordinate stage_coordinate = track.project(
                {stage.position_x.value(), stage.position_y.value()}, coordinate.s_m, 100.0);
            return model.derivative(stage, tracking_control(stage, stage_coordinate));
        };
        state = integrate(options.integrator, state, si::seconds(time_s), options.timestep, derivative);
        time_s += options.timestep.value();
    }
    if (error_samples > 0U) {
        const double divisor = static_cast<double>(error_samples);
        result.metrics.rms_position_error_m = std::sqrt(squared_position_error / divisor);
        result.metrics.rms_speed_error_mps = std::sqrt(squared_speed_error / divisor);
        result.metrics.rms_yaw_error_rad = std::sqrt(squared_yaw_error / divisor);
    }
    return result;
}

} // namespace apexlab

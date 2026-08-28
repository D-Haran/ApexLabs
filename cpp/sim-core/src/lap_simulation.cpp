#include "apexlab/lap_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace apexlab {
namespace {

[[nodiscard]] double wrap_angle(double value) {
    return std::remainder(value, 2.0 * std::numbers::pi);
}

[[nodiscard]] double max_utilization(const NonlinearPlanarForces& forces) {
    double result = 0.0;
    for (const auto& wheel : forces.wheel) result = std::max(result, wheel.friction_utilization);
    return result;
}

} // namespace

ReferenceLine::ReferenceLine(const PeriodicTrack& track, double constant_offset_m)
    : track_(&track), offset_m_(constant_offset_m) {
    constexpr std::size_t checks = 1000;
    for (std::size_t index = 0; index < checks; ++index) {
        const double s = track.length_m() * static_cast<double>(index) / static_cast<double>(checks);
        if (!track.contains(s, constant_offset_m)) {
            throw std::invalid_argument("reference line offset leaves track boundaries");
        }
    }
}

DirectedLapProgress::DirectedLapProgress(double track_length_m, double initial_s_m)
    : length_m_(track_length_m), last_s_m_(initial_s_m) {
    if (!(length_m_ > 0.0) || !std::isfinite(initial_s_m)) {
        throw std::invalid_argument("directed lap progress requires finite positive length");
    }
    last_s_m_ = std::fmod(last_s_m_, length_m_);
    if (last_s_m_ < 0.0) last_s_m_ += length_m_;
}

void DirectedLapProgress::update(double wrapped_s_m) {
    wrapped_s_m = std::fmod(wrapped_s_m, length_m_);
    if (wrapped_s_m < 0.0) wrapped_s_m += length_m_;
    double increment = wrapped_s_m - last_s_m_;
    if (increment < -0.5 * length_m_) increment += length_m_;
    if (increment > 0.5 * length_m_) increment -= length_m_;
    unwrapped_s_m_ += increment;
    last_s_m_ = wrapped_s_m;
    completed_laps_ = std::max(completed_laps_,
                               std::max(0, static_cast<int>(std::floor(unwrapped_s_m_ / length_m_ + 1.0e-9))));
}

TrackFrame ReferenceLine::frame_at_s(double s_m) const {
    TrackFrame frame = track_->frame_at_s(s_m);
    frame.position.x += frame.left_normal.x * offset_m_;
    frame.position.y += frame.left_normal.y * offset_m_;
    // Exact parallel-curve curvature, valid while 1-kappa*offset remains nonzero.
    const double denominator = 1.0 - frame.curvature_1_m * offset_m_;
    if (std::abs(denominator) < 0.05) throw std::domain_error("reference offset reaches curvature singularity");
    frame.curvature_1_m /= denominator;
    return frame;
}

SpeedProfile::SpeedProfile(const PeriodicTrack& track, const ReferenceLine& line,
                           SpeedProfileOptions options)
    : length_m_(track.length_m()) {
    if (options.sample_spacing_m <= 0.0 || options.lateral_acceleration_limit_mps2 <= 0.0 ||
        options.acceleration_limit_mps2 <= 0.0 || options.braking_limit_mps2 <= 0.0 ||
        options.maximum_speed_mps <= 0.0 || options.safety_factor <= 0.0 ||
        options.safety_factor > 1.0) {
        throw std::invalid_argument("invalid speed profile options");
    }
    const std::size_t count = std::max<std::size_t>(64U, static_cast<std::size_t>(std::ceil(length_m_ / options.sample_spacing_m)));
    spacing_m_ = length_m_ / static_cast<double>(count);
    values_.resize(count);
    for (std::size_t index = 0; index < count; ++index) {
        const double curvature = std::abs(line.frame_at_s(static_cast<double>(index) * spacing_m_).curvature_1_m);
        const double lateral_limit = curvature < 1.0e-7
                                         ? options.maximum_speed_mps
                                         : std::sqrt(options.lateral_acceleration_limit_mps2 / curvature);
        values_[index] = options.safety_factor * std::min(options.maximum_speed_mps, lateral_limit);
    }
    // Repeated cyclic passes resolve the closed-loop coupling at the start/finish boundary.
    for (int pass = 0; pass < 8; ++pass) {
        for (std::size_t index = 0; index < count; ++index) {
            const std::size_t next = (index + 1U) % count;
            const double reachable = std::sqrt(values_[index] * values_[index] +
                                               2.0 * options.acceleration_limit_mps2 * spacing_m_);
            values_[next] = std::min(values_[next], reachable);
        }
        for (std::size_t reverse = count; reverse-- > 0U;) {
            const std::size_t next = (reverse + 1U) % count;
            const double admissible = std::sqrt(values_[next] * values_[next] +
                                                2.0 * options.braking_limit_mps2 * spacing_m_);
            values_[reverse] = std::min(values_[reverse], admissible);
        }
    }
}

double SpeedProfile::target_speed_mps(double s_m) const {
    double wrapped = std::fmod(s_m, length_m_);
    if (wrapped < 0.0) wrapped += length_m_;
    const double index_value = wrapped / spacing_m_;
    const auto index = static_cast<std::size_t>(std::floor(index_value)) % values_.size();
    const std::size_t next = (index + 1U) % values_.size();
    const double fraction = index_value - std::floor(index_value);
    return values_[index] + fraction * (values_[next] - values_[index]);
}

LapSimulationResult simulate_laps(const NonlinearPlanarVehicleModel& model,
                                  const PeriodicTrack& track, const ReferenceLine& line,
                                  const SpeedProfile& speed_profile,
                                  const LapSimulationOptions& options) {
    if (!track.definition().closed) {
        throw std::invalid_argument("lap simulation requires a closed track");
    }
    if (options.timestep.value() <= 0.0 || options.controller_timestep.value() <= 0.0 ||
        options.initial_speed_mps <= 0.0 || options.warmup_laps < 0 || options.timed_laps <= 0) {
        throw std::invalid_argument("invalid lap simulation options");
    }
    const TrackFrame initial_frame = line.frame_at_s(0.0);
    PlanarState state{si::meters(initial_frame.position.x), si::meters(initial_frame.position.y),
                      si::radians(initial_frame.heading_rad), si::meters_per_second(options.initial_speed_mps),
                      si::meters_per_second(0.0),
                      si::radians_per_second(options.initial_speed_mps * initial_frame.curvature_1_m)};
    std::optional<RoadVehicleModel> road_model;
    if (model.parameters().normal_load_model == "sprung_body") road_model.emplace(model.parameters());
    SprungState body;
    LapSimulationResult result;
    result.samples.reserve(static_cast<std::size_t>(options.maximum_duration_s / options.timestep.value()) + 1U);
    double time_s = 0.0;
    DirectedLapProgress progress{track.length_m()};
    double lap_start_time_s = 0.0;
    double sector_start_time_s = 0.0;
    int completed_laps = 0;
    int sector_index = 0;
    double next_control_time_s = 0.0;
    PlanarControl held_control{};
    std::optional<double> projection_hint = 0.0;
    const int total_laps = options.warmup_laps + options.timed_laps;
    const auto& sectors = track.definition().sector_boundaries_fraction;

    auto make_sample = [&](std::uint64_t step, const TrackCoordinate& coordinate,
                           const TrackFrame& reference, const NonlinearPlanarForces& forces,
                           double heading_error, double target_speed) {
        const double speed = std::hypot(state.velocity_x.value(), state.velocity_y.value());
        result.samples.push_back({lap_telemetry_schema_version, si::seconds(time_s), step, state,
                                  held_control, forces, speed,
                                  forces.body_longitudinal_force.value() / model.parameters().mass.value(),
                                  forces.body_lateral_force.value() / model.parameters().mass.value(),
                                  coordinate.s_m, progress.unwrapped_s_m(), coordinate.s_m / track.length_m(),
                                  coordinate.lateral_offset_m - line.offset_m(), heading_error,
                                  reference.curvature_1_m, target_speed, target_speed - speed,
                                  completed_laps, time_s - lap_start_time_s, sector_index,
                                  track.contains(coordinate.s_m, coordinate.lateral_offset_m), std::nullopt, {}, {}});
        if (road_model) {
            const auto road_forces = road_model->forces({state, body}, held_control, reference);
            result.samples.back().chassis = body;
            result.samples.back().suspension_compression_m = road_forces.compression_m;
            result.samples.back().gravity_body = road_forces.gravity_body;
        }
    };

    for (std::uint64_t step = 0; time_s <= options.maximum_duration_s; ++step) {
        const TrackCoordinate coordinate = track.project({state.position_x.value(), state.position_y.value()},
                                                         projection_hint, 80.0);
        projection_hint = coordinate.s_m;
        if (step > 0U) progress.update(coordinate.s_m);
        const TrackFrame reference = line.frame_at_s(coordinate.s_m);
        const double heading_error = wrap_angle(state.yaw.value() - reference.heading_rad);
        const double target_speed = speed_profile.target_speed_mps(coordinate.s_m);
        const double speed = std::hypot(state.velocity_x.value(), state.velocity_y.value());
        if (time_s + 1.0e-12 >= next_control_time_s) {
            const double steering = std::atan(model.parameters().planar->wheelbase.value() * reference.curvature_1_m) -
                                    options.controller.lateral_gain * (coordinate.lateral_offset_m - line.offset_m()) -
                                    options.controller.heading_gain * heading_error;
            const double speed_command = options.controller.speed_gain * (target_speed - speed);
            held_control = {si::radians(std::clamp(steering, -options.controller.maximum_steering_rad,
                                                   options.controller.maximum_steering_rad)),
                            std::clamp(speed_command, 0.0, 1.0),
                            std::clamp(-speed_command, 0.0, 1.0)};
            next_control_time_s += options.controller_timestep.value();
        }
        const NonlinearPlanarForces forces = road_model
            ? road_model->forces({state, body}, held_control, reference).tires
            : model.forces(state, held_control);
        make_sample(step, coordinate, reference, forces, heading_error, target_speed);

        const int newly_completed = progress.completed_laps();
        if (newly_completed > completed_laps) {
            const double lap_time = time_s - lap_start_time_s;
            if (completed_laps >= options.warmup_laps) {
                result.completed_lap_times_s.push_back(lap_time);
                if (!sectors.empty()) result.completed_sector_times_s.push_back(time_s - sector_start_time_s);
            }
            lap_start_time_s = time_s;
            sector_start_time_s = time_s;
            sector_index = 0;
            completed_laps = newly_completed;
            if (completed_laps >= total_laps) {
                result.completed = true;
                break;
            }
        }
        if (!sectors.empty() && sector_index < static_cast<int>(sectors.size()) - 1 &&
            coordinate.s_m / track.length_m() >= sectors[static_cast<std::size_t>(sector_index)]) {
            if (completed_laps >= options.warmup_laps)
                result.completed_sector_times_s.push_back(time_s - sector_start_time_s);
            sector_start_time_s = time_s;
            ++sector_index;
        }
        const auto derivative = [&model, held_control](const PlanarState& stage, Time) {
            return model.derivative(stage, held_control);
        };
        if (road_model) {
            const auto road_derivative = [&](const RoadVehicleState& stage, Time) {
                const auto projection = track.project({stage.planar.position_x.value(), stage.planar.position_y.value()}, coordinate.s_m, 80.0);
                return road_model->derivative(stage, held_control, track.frame_at_s(projection.s_m));
            };
            const auto next = integrate(options.integrator, RoadVehicleState{state,body}, si::seconds(time_s), options.timestep, road_derivative);
            state = next.planar;
            body = next.body;
        } else {
            state = integrate(options.integrator, state, si::seconds(time_s), options.timestep, derivative);
        }
        time_s += options.timestep.value();
    }
    return result;
}

void write_lap_telemetry_csv(const std::filesystem::path& path,
                             std::span<const LapTelemetrySample> samples) {
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
    std::ofstream stream(path);
    if (!stream) throw std::runtime_error("cannot write lap telemetry: " + path.string());
    stream << "schema_version,time_s,step,position_x_m,position_y_m,yaw_rad,yaw_rate_rad_s,vx_m_s,vy_m_s,speed_m_s,steering_angle_rad,throttle,brake,longitudinal_accel_m_s2,lateral_accel_m_s2,track_s_m,unwrapped_track_s_m,track_progress_fraction,lateral_error_m,heading_error_rad,reference_curvature_1_m,target_speed_m_s,speed_error_m_s,lap_number,lap_elapsed_time_s,sector_index,on_track";
    for (const char* suffix : {"fl", "fr", "rl", "rr"}) stream << ",friction_utilization_" << suffix;
    stream << '\n' << std::setprecision(17);
    for (const auto& sample : samples) {
        const auto& s = sample.state;
        stream << sample.schema_version << ',' << sample.time.value() << ',' << sample.step << ','
               << s.position_x.value() << ',' << s.position_y.value() << ',' << s.yaw.value() << ','
               << s.yaw_rate.value() << ',' << s.velocity_x.value() << ',' << s.velocity_y.value() << ','
               << sample.speed_mps << ',' << sample.input.steering_angle.value() << ','
               << sample.input.throttle << ',' << sample.input.brake << ','
               << sample.longitudinal_acceleration_mps2 << ',' << sample.lateral_acceleration_mps2 << ','
               << sample.track_s_m << ',' << sample.unwrapped_track_s_m << ','
               << sample.track_progress_fraction << ',' << sample.lateral_error_m << ','
               << sample.heading_error_rad << ',' << sample.reference_curvature_1_m << ','
               << sample.target_speed_mps << ',' << sample.speed_error_mps << ',' << sample.lap_number << ','
               << sample.lap_elapsed_time_s << ',' << sample.sector_index << ',' << (sample.on_track ? 1 : 0);
        for (const auto& wheel : sample.forces.wheel) stream << ',' << wheel.friction_utilization;
        stream << '\n';
    }
}

std::vector<SpatialTelemetrySample>
resample_lap_spatially(std::span<const LapTelemetrySample> samples, int lap_number,
                       double track_length_m, double spacing_m) {
    if (track_length_m <= 0.0 || spacing_m <= 0.0) throw std::invalid_argument("spatial resampling dimensions invalid");
    std::vector<const LapTelemetrySample*> lap;
    for (const auto& sample : samples) if (sample.lap_number == lap_number) lap.push_back(&sample);
    if (lap.size() < 2U) return {};
    std::sort(lap.begin(), lap.end(), [](auto* a, auto* b) { return a->track_s_m < b->track_s_m; });
    std::vector<SpatialTelemetrySample> output;
    for (double s = 0.0; s < track_length_m; s += spacing_m) {
        const auto upper = std::lower_bound(lap.begin(), lap.end(), s,
                                            [](auto* sample, double value) { return sample->track_s_m < value; });
        if (upper == lap.begin() || upper == lap.end()) continue;
        const auto* a = *std::prev(upper);
        const auto* b = *upper;
        const double f = (s - a->track_s_m) / (b->track_s_m - a->track_s_m);
        const auto mix = [f](double x, double y) { return x + f * (y - x); };
        output.push_back({s, mix(a->speed_mps, b->speed_mps), mix(a->input.throttle, b->input.throttle),
                          mix(a->input.brake, b->input.brake),
                          mix(a->input.steering_angle.value(), b->input.steering_angle.value()),
                          mix(a->lateral_acceleration_mps2, b->lateral_acceleration_mps2),
                          mix(max_utilization(a->forces), max_utilization(b->forces)),
                          mix(a->lateral_error_m, b->lateral_error_m)});
    }
    return output;
}

} // namespace apexlab

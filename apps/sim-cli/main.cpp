#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "replay_export.hpp"
#include "apexlab/scenario.hpp"
#include "apexlab/lap_simulation.hpp"
#include "apexlab/optimization_replay.hpp"
#include "apexlab/simulation.hpp"
#include "apexlab/track.hpp"
#include "apexlab/vehicle_parameters.hpp"

namespace {

struct Arguments {
    std::filesystem::path vehicle_path;
    std::filesystem::path scenario_path;
    std::filesystem::path track_path;
    std::filesystem::path track_geometry_path;
    std::filesystem::path replay_data_path;
    std::filesystem::path optimization_profile_path;
    std::filesystem::path replay_diagnostics_path;
    std::filesystem::path output_path{"telemetry.csv"};
    double duration_s{10.0};
    double timestep_s{0.005};
    double initial_speed_mps{0.0};
    double throttle{0.0};
    double brake{0.0};
    apexlab::IntegratorKind integrator{apexlab::IntegratorKind::rk4};
    bool stop_when_stationary{false};
    int warmup_laps{1};
    int timed_laps{1};
    double controller_dt_s{0.02};
    double reference_offset_m{0.0};
    double maximum_lap_duration_s{300.0};
};

[[noreturn]] void usage_error(const std::string& message) {
    throw std::invalid_argument(message + "\nUse --help for usage.");
}

[[nodiscard]] double parse_number(std::string_view text, std::string_view option) {
    std::size_t parsed = 0;
    const std::string value{text};
    const double result = std::stod(value, &parsed);
    if (parsed != value.size()) {
        usage_error("invalid number for " + std::string{option} + ": " + value);
    }
    return result;
}

void print_help() {
    std::cout << "ApexLab vehicle simulation CLI\n\n"
              << "Required:\n"
              << "  --vehicle PATH           Vehicle JSON configuration\n\n"
              << "Options:\n"
              << "  --output PATH            Output CSV (default: telemetry.csv)\n"
              << "  --scenario PATH          Planar scenario JSON; selects bicycle model\n"
              << "  --track PATH             Closed track JSON; selects nonlinear lap mode\n"
              << "  --replay-data DIR        Export wheel states, spatial samples and lap results\n"
              << "  --track-geometry PATH    Also write sampled centerline/boundary CSV\n"
              << "  --optimization-profile PATH  Replay optimized controls through production dynamics\n"
              << "  --replay-diagnostics PATH    Write optimization replay errors as JSON\n"
              << "  --warmup-laps COUNT      Untimed settling laps (default: 1)\n"
              << "  --laps COUNT             Timed laps (default: 1)\n"
              << "  --controller-dt SECONDS  Controller update period (default: 0.02)\n"
              << "  --reference-offset M     Constant left-positive line offset (default: 0)\n"
              << "  --maximum-lap-duration S Abort incomplete lap runs after this time (default: 300)\n"
              << "\nLongitudinal-mode options (used when --scenario is absent):\n"
              << "  --duration SECONDS       Simulated duration (default: 10)\n"
              << "  --dt SECONDS             Fixed physics timestep (default: 0.005)\n"
              << "  --integrator euler|rk4   Integration method (default: rk4)\n"
              << "  --initial-speed MPS      Initial signed speed (default: 0)\n"
              << "  --throttle FRACTION      Constant command in [0,1] (default: 0)\n"
              << "  --brake FRACTION         Constant command in [0,1] (default: 0)\n"
              << "  --stop-when-stationary   Stop at the first forward zero-speed event\n"
              << "  --help                   Show this help\n";
}

[[nodiscard]] Arguments parse_arguments(int argc, char** argv) {
    Arguments arguments;
    for (int index = 1; index < argc; ++index) {
        const std::string_view option{argv[index]};
        const auto value = [&]() -> std::string_view {
            if (index + 1 >= argc) {
                usage_error("missing value for " + std::string{option});
            }
            return argv[++index];
        };

        if (option == "--help") {
            print_help();
            std::exit(EXIT_SUCCESS);
        } else if (option == "--vehicle") {
            arguments.vehicle_path = value();
        } else if (option == "--output") {
            arguments.output_path = value();
        } else if (option == "--scenario") {
            arguments.scenario_path = value();
        } else if (option == "--track") {
            arguments.track_path = value();
        } else if (option == "--track-geometry") {
            arguments.track_geometry_path = value();
        } else if (option == "--replay-data") {
            arguments.replay_data_path = value();
        } else if (option == "--optimization-profile") {
            arguments.optimization_profile_path = value();
        } else if (option == "--replay-diagnostics") {
            arguments.replay_diagnostics_path = value();
        } else if (option == "--warmup-laps") {
            arguments.warmup_laps = static_cast<int>(parse_number(value(), option));
        } else if (option == "--laps") {
            arguments.timed_laps = static_cast<int>(parse_number(value(), option));
        } else if (option == "--controller-dt") {
            arguments.controller_dt_s = parse_number(value(), option);
        } else if (option == "--reference-offset") {
            arguments.reference_offset_m = parse_number(value(), option);
        } else if (option == "--maximum-lap-duration") {
            arguments.maximum_lap_duration_s = parse_number(value(), option);
        } else if (option == "--duration") {
            arguments.duration_s = parse_number(value(), option);
        } else if (option == "--dt") {
            arguments.timestep_s = parse_number(value(), option);
        } else if (option == "--initial-speed") {
            arguments.initial_speed_mps = parse_number(value(), option);
        } else if (option == "--throttle") {
            arguments.throttle = parse_number(value(), option);
        } else if (option == "--brake") {
            arguments.brake = parse_number(value(), option);
        } else if (option == "--integrator") {
            const std::string_view name = value();
            if (name == "euler") {
                arguments.integrator = apexlab::IntegratorKind::euler;
            } else if (name == "rk4") {
                arguments.integrator = apexlab::IntegratorKind::rk4;
            } else {
                usage_error("integrator must be euler or rk4");
            }
        } else if (option == "--stop-when-stationary") {
            arguments.stop_when_stationary = true;
        } else {
            usage_error("unknown option: " + std::string{option});
        }
    }

    if (!arguments.replay_data_path.empty() && arguments.track_path.empty()) {
        usage_error("--replay-data requires --track");
    }
    if (!arguments.optimization_profile_path.empty() && arguments.track_path.empty()) {
        usage_error("--optimization-profile requires --track");
    }
    if (!arguments.replay_diagnostics_path.empty() && arguments.optimization_profile_path.empty()) {
        usage_error("--replay-diagnostics requires --optimization-profile");
    }
    if (arguments.vehicle_path.empty()) {
        usage_error("--vehicle is required");
    }
    if (arguments.throttle < 0.0 || arguments.throttle > 1.0 || arguments.brake < 0.0 ||
        arguments.brake > 1.0) {
        usage_error("throttle and brake must be in [0,1]");
    }
    if (!arguments.track_path.empty() && !arguments.scenario_path.empty()) {
        usage_error("--track and --scenario are mutually exclusive");
    }
    if (arguments.warmup_laps < 0 || arguments.timed_laps <= 0 || arguments.controller_dt_s <= 0.0 ||
        arguments.maximum_lap_duration_s <= 0.0) {
        usage_error("lap counts and controller timestep are invalid");
    }
    return arguments;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Arguments arguments = parse_arguments(argc, argv);
        const apexlab::VehicleParameters parameters =
            apexlab::load_vehicle_parameters(arguments.vehicle_path);
        if (!arguments.track_path.empty()) {
            if (parameters.schema_version != apexlab::nonlinear_vehicle_schema_version) {
                usage_error("lap mode requires a schema-v3 nonlinear vehicle");
            }
            const apexlab::PeriodicTrack track{apexlab::load_track_definition(arguments.track_path)};
            if (!arguments.track_geometry_path.empty()) {
                if (arguments.track_geometry_path.has_parent_path())
                    std::filesystem::create_directories(arguments.track_geometry_path.parent_path());
                std::ofstream geometry(arguments.track_geometry_path);
                if (!geometry) throw std::runtime_error("cannot write track geometry CSV");
                geometry << "s_m,x_m,y_m,elevation_m,grade,heading_rad,curvature_1_m,left_width_m,right_width_m,left_x_m,left_y_m,right_x_m,right_y_m,projection_s_error_m,projection_lateral_error_m,projection_reconstruction_error_m\n"
                         << std::setprecision(17);
                constexpr std::size_t geometry_samples = 2000;
                for (std::size_t index = 0; index <= geometry_samples; ++index) {
                    const double s = track.length_m() * static_cast<double>(index) /
                                     static_cast<double>(geometry_samples);
                    const auto frame = track.frame_at_s(s);
                    const double offset = index % 2U == 0U ? 1.0 : -1.0;
                    const auto test_point = track.position_at_s(s, offset);
                    const auto projected = track.project(test_point, s, 20.0);
                    double projection_s_error = std::abs(projected.s_m - track.wrap_s(s));
                    projection_s_error = std::min(projection_s_error, track.length_m() - projection_s_error);
                    const auto reconstructed = track.position_at_s(projected.s_m, projected.lateral_offset_m);
                    geometry << s << ',' << frame.position.x << ',' << frame.position.y << ','
                             << frame.elevation_m << ',' << frame.grade << ','
                             << frame.heading_rad << ',' << frame.curvature_1_m << ','
                             << frame.left_width_m << ',' << frame.right_width_m << ','
                             << frame.position.x + frame.left_normal.x * frame.left_width_m << ','
                             << frame.position.y + frame.left_normal.y * frame.left_width_m << ','
                             << frame.position.x - frame.left_normal.x * frame.right_width_m << ','
                             << frame.position.y - frame.left_normal.y * frame.right_width_m << ','
                             << projection_s_error << ','
                             << std::abs(projected.lateral_offset_m - offset) << ','
                             << std::hypot(reconstructed.x - test_point.x, reconstructed.y - test_point.y)
                             << '\n';
                }
            }
            const apexlab::ReferenceLine line{track, arguments.reference_offset_m};
            const apexlab::NonlinearPlanarVehicleModel model{parameters};
            apexlab::LapSimulationOptions lap_options;
            lap_options.timestep = apexlab::si::seconds(arguments.timestep_s);
            lap_options.controller_timestep = apexlab::si::seconds(arguments.controller_dt_s);
            lap_options.warmup_laps = arguments.warmup_laps;
            lap_options.timed_laps = arguments.timed_laps;
            lap_options.maximum_duration_s = arguments.maximum_lap_duration_s;
            lap_options.integrator = arguments.integrator;
            apexlab::LapSimulationResult result;
            std::optional<apexlab::OptimizationReplayMetrics> replay_metrics;
            if (!arguments.optimization_profile_path.empty()) {
                lap_options.controller = apexlab::PathControllerGains{0.08, 1.40, 0.35, 0.40};
                const auto trajectory = apexlab::load_optimization_trajectory(
                    arguments.optimization_profile_path, track.length_m());
                lap_options.initial_speed_mps = trajectory.nodes().front().speed_mps;
                auto replay = apexlab::replay_optimization_trajectory(
                    model, track, trajectory, lap_options);
                result = std::move(replay.lap);
                replay_metrics = replay.metrics;
                if (!arguments.replay_diagnostics_path.empty()) {
                    if (arguments.replay_diagnostics_path.has_parent_path())
                        std::filesystem::create_directories(
                            arguments.replay_diagnostics_path.parent_path());
                    std::ofstream diagnostics{arguments.replay_diagnostics_path};
                    if (!diagnostics) throw std::runtime_error("cannot write replay diagnostics");
                    diagnostics << std::setprecision(17)
                                << "{\n  \"schema_version\": 1,\n"
                                << "  \"completed\": "
                                << (result.completed ? "true" : "false") << ",\n"
                                << "  \"replayed_lap_time_s\": "
                                << (result.completed_lap_times_s.empty()
                                        ? -1.0
                                        : result.completed_lap_times_s.back())
                                << ",\n  \"maximum_position_error_m\": "
                                << replay.metrics.maximum_position_error_m
                                << ",\n  \"maximum_speed_error_mps\": "
                                << replay.metrics.maximum_speed_error_mps
                                << ",\n  \"maximum_yaw_error_rad\": "
                                << replay.metrics.maximum_yaw_error_rad
                                << ",\n  \"maximum_tire_utilization\": "
                                << replay.metrics.maximum_tire_utilization
                                << ",\n  \"maximum_track_violation_m\": "
                                << replay.metrics.maximum_track_violation_m
                                << ",\n  \"maximum_steering_correction_rad\": "
                                << replay.metrics.maximum_steering_correction_rad
                                << ",\n  \"maximum_longitudinal_command_correction\": "
                                << replay.metrics.maximum_longitudinal_command_correction
                                << ",\n  \"rms_position_error_m\": "
                                << replay.metrics.rms_position_error_m
                                << ",\n  \"rms_speed_error_mps\": "
                                << replay.metrics.rms_speed_error_mps
                                << ",\n  \"rms_yaw_error_rad\": "
                                << replay.metrics.rms_yaw_error_rad
                                << "\n}\n";
                }
            } else {
                const double nominal_lateral_limit =
                    parameters.nonlinear_planar->tire_mu_reference *
                    apexlab::standard_gravity_mps2;
                const apexlab::SpeedProfile profile{
                    track, line,
                    apexlab::SpeedProfileOptions{nominal_lateral_limit, 3.5, 7.0, 55.0, 0.72,
                                                 1.0}};
                lap_options.initial_speed_mps =
                    std::max(5.0, profile.target_speed_mps(0.0) * 0.7);
                result = apexlab::simulate_laps(model, track, line, profile, lap_options);
            }
            apexlab::write_lap_telemetry_csv(arguments.output_path, result.samples);
            if (!arguments.replay_data_path.empty())
                write_replay_data(arguments.replay_data_path, result, track, lap_options);
            std::cout << "vehicle=\"" << parameters.name << "\" track=\""
                      << track.definition().name << "\" model="
                      << (replay_metrics ? "optimization-production-replay"
                                         : "nonlinear-four-tire-lap")
                      << " track_length_m=" << track.length_m() << " samples=" << result.samples.size()
                      << " completed=" << (result.completed ? 1 : 0);
            if (!result.completed_lap_times_s.empty())
                std::cout << " timed_lap_s=" << result.completed_lap_times_s.back();
            std::cout << " output=\"" << arguments.output_path.string() << "\"\n";
            return result.completed ? EXIT_SUCCESS : EXIT_FAILURE;
        }
        if (!arguments.scenario_path.empty()) {
            const apexlab::PlanarScenario scenario =
                apexlab::load_planar_scenario(arguments.scenario_path);
            const apexlab::SimulationOptions options{
                scenario.duration,
                scenario.timestep,
                scenario.integrator,
                false,
            };
            if (parameters.schema_version == apexlab::nonlinear_vehicle_schema_version) {
                const apexlab::NonlinearPlanarVehicleModel model{parameters};
                const auto samples = apexlab::simulate_nonlinear_planar(
                    model, scenario.initial_state, options,
                    [&scenario](apexlab::Time time, const apexlab::PlanarState&) {
                        return scenario.control_at(time);
                    });
                apexlab::write_nonlinear_planar_telemetry_csv(arguments.output_path, samples);
                const auto& final_sample = samples.back();
                std::cout << "vehicle=\"" << parameters.name << "\" scenario=\"" << scenario.name
                          << "\" model=nonlinear-four-tire integrator="
                          << apexlab::to_string(scenario.integrator) << " samples=" << samples.size()
                          << " final_time_s=" << final_sample.time.value()
                          << " final_position_x_m=" << final_sample.state.position_x.value()
                          << " final_position_y_m=" << final_sample.state.position_y.value()
                          << " final_yaw_rad=" << final_sample.state.yaw.value() << " output=\""
                          << arguments.output_path.string() << "\"\n";
                return EXIT_SUCCESS;
            }
            const apexlab::PlanarVehicleModel model{parameters};
            const auto samples = apexlab::simulate_planar(
                model, scenario.initial_state, options,
                [&scenario](apexlab::Time time, const apexlab::PlanarState&) {
                    return scenario.control_at(time);
                });
            apexlab::write_planar_telemetry_csv(arguments.output_path, samples);
            const auto& final_sample = samples.back();
            std::cout << "vehicle=\"" << parameters.name << "\" scenario=\"" << scenario.name
                      << "\" model=planar-bicycle integrator="
                      << apexlab::to_string(scenario.integrator) << " samples=" << samples.size()
                      << " final_time_s=" << final_sample.time.value()
                      << " final_position_x_m=" << final_sample.state.position_x.value()
                      << " final_position_y_m=" << final_sample.state.position_y.value()
                      << " final_yaw_rad=" << final_sample.state.yaw.value() << " output=\""
                      << arguments.output_path.string() << "\"\n";
            return EXIT_SUCCESS;
        }
        const apexlab::LongitudinalModel model{parameters};
        const apexlab::SimulationOptions options{
            apexlab::si::seconds(arguments.duration_s),
            apexlab::si::seconds(arguments.timestep_s),
            arguments.integrator,
            arguments.stop_when_stationary,
        };
        const apexlab::LongitudinalState initial_state{
            apexlab::si::meters(0.0),
            apexlab::si::meters_per_second(arguments.initial_speed_mps),
        };
        const apexlab::DriverInput constant_input{arguments.throttle, arguments.brake};
        const auto samples =
            apexlab::simulate(model, initial_state, options,
                              [constant_input](apexlab::Time, const apexlab::LongitudinalState&) {
                                  return constant_input;
                              });
        apexlab::write_telemetry_csv(arguments.output_path, samples);

        const auto& final_sample = samples.back();
        std::cout << "vehicle=\"" << parameters.name
                  << "\" integrator=" << apexlab::to_string(arguments.integrator)
                  << " samples=" << samples.size() << " final_time_s=" << final_sample.time.value()
                  << " final_position_m=" << final_sample.state.position.value()
                  << " final_velocity_mps=" << final_sample.state.velocity.value() << " output=\""
                  << arguments.output_path.string() << "\"\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "apexlab-sim: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}

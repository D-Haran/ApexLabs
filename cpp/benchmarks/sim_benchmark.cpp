#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

#include "apexlab/integrator.hpp"
#include "apexlab/lap_simulation.hpp"
#include "apexlab/longitudinal_model.hpp"
#include "apexlab/nonlinear_planar_model.hpp"
#include "apexlab/planar_model.hpp"
#include "apexlab/track.hpp"

int main() {
    apexlab::VehicleParameters parameters;
    parameters.name = "benchmark fixture";
    parameters.provenance = "benchmark-only assumed parameters";
    parameters.mass = apexlab::si::kilograms(1450.0);
    parameters.aero = {1.225, 0.32, 0.20, 2.20};
    parameters.powertrain = {apexlab::si::newtons(8000.0), 0.55};
    parameters.brakes = {apexlab::si::newtons(16000.0)};
    parameters.tires = {1.05, 0.015};
    const apexlab::LongitudinalModel model{parameters};

    constexpr std::size_t iterations = 2'000'000;
    constexpr std::size_t warmup_iterations = 20'000;
    const apexlab::Time dt = apexlab::si::seconds(0.005);
    const apexlab::DriverInput input{0.6, 0.0};
    apexlab::LongitudinalState state{apexlab::si::meters(0.0),
                                     apexlab::si::meters_per_second(15.0)};
    apexlab::Time time = apexlab::si::seconds(0.0);
    const auto derivative = [&model, input](const apexlab::LongitudinalState& value,
                                            apexlab::Time) {
        return model.derivative(value, input);
    };

    for (std::size_t index = 0; index < warmup_iterations; ++index) {
        state = apexlab::integrate_rk4(state, time, dt, derivative);
        time += dt;
    }
    state = {apexlab::si::meters(0.0), apexlab::si::meters_per_second(15.0)};
    time = apexlab::si::seconds(0.0);
    const auto start = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < iterations; ++index) {
        state = apexlab::integrate_rk4(state, time, dt, derivative);
        time += dt;
    }
    const auto end = std::chrono::steady_clock::now();
    const double elapsed_s = std::chrono::duration<double>(end - start).count();
    const double nanoseconds_per_step = elapsed_s * 1.0e9 / static_cast<double>(iterations);

    std::cout << std::fixed << std::setprecision(3) << "workload=longitudinal-rk4"
              << " iterations=" << iterations << " warmup_iterations=" << warmup_iterations
              << " elapsed_s=" << elapsed_s << " ns_per_step=" << nanoseconds_per_step
              << " checksum=" << state.position.value() + state.velocity.value() << '\n';

    parameters.schema_version = apexlab::planar_vehicle_schema_version;
    parameters.aero = {1.225, 0.0, 0.0, 0.0};
    parameters.tires = {1.05, 0.0};
    parameters.planar = apexlab::PlanarParameters{
        apexlab::si::meters(2.8),
        apexlab::si::meters(1.2),
        apexlab::si::meters(1.6),
        apexlab::si::kilogram_square_meters(2500.0),
        100000.0,
        75000.0,
    };
    const apexlab::PlanarVehicleModel planar_model{parameters};
    const apexlab::PlanarControl planar_input{apexlab::si::radians(0.005), 0.0003, 0.0};
    apexlab::PlanarState planar_state{
        apexlab::si::meters(0.0),
        apexlab::si::meters(0.0),
        apexlab::si::radians(0.0),
        apexlab::si::meters_per_second(15.0),
        apexlab::si::meters_per_second(0.0),
        apexlab::si::radians_per_second(0.0),
    };
    time = apexlab::si::seconds(0.0);
    const auto planar_derivative = [&planar_model, planar_input](const apexlab::PlanarState& value,
                                                                 apexlab::Time) {
        return planar_model.derivative(value, planar_input);
    };
    for (std::size_t index = 0; index < warmup_iterations; ++index) {
        planar_state = apexlab::integrate_rk4(planar_state, time, dt, planar_derivative);
        time += dt;
    }
    planar_state = {
        apexlab::si::meters(0.0),
        apexlab::si::meters(0.0),
        apexlab::si::radians(0.0),
        apexlab::si::meters_per_second(15.0),
        apexlab::si::meters_per_second(0.0),
        apexlab::si::radians_per_second(0.0),
    };
    time = apexlab::si::seconds(0.0);
    const auto planar_start = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < iterations; ++index) {
        planar_state = apexlab::integrate_rk4(planar_state, time, dt, planar_derivative);
        time += dt;
    }
    const auto planar_end = std::chrono::steady_clock::now();
    const double planar_elapsed_s =
        std::chrono::duration<double>(planar_end - planar_start).count();
    const double planar_nanoseconds_per_step =
        planar_elapsed_s * 1.0e9 / static_cast<double>(iterations);
    const double planar_checksum = planar_state.position_x.value() +
                                   planar_state.position_y.value() + planar_state.yaw.value() +
                                   planar_state.velocity_x.value() +
                                   planar_state.velocity_y.value() + planar_state.yaw_rate.value();
    std::cout << "workload=planar-bicycle-rk4"
              << " iterations=" << iterations << " warmup_iterations=" << warmup_iterations
              << " elapsed_s=" << planar_elapsed_s << " ns_per_step=" << planar_nanoseconds_per_step
              << " checksum=" << planar_checksum << '\n';

    parameters.schema_version = apexlab::nonlinear_vehicle_schema_version;
    parameters.powertrain = {apexlab::si::newtons(9000.0), 0.45};
    parameters.brakes = {apexlab::si::newtons(18000.0)};
    parameters.planar->front_cornering_stiffness_n_per_rad = 120000.0;
    parameters.planar->rear_cornering_stiffness_n_per_rad = 130000.0;
    parameters.nonlinear_planar = apexlab::NonlinearPlanarParameters{
        apexlab::si::meters(1.6), apexlab::si::meters(1.58), apexlab::si::meters(0.5),
        0.55, 0.62, "RWD", 0.0, 1.2, apexlab::si::newtons(3555.0), -0.08,
        1.0e-6, 80, 0.5,
    };
    const apexlab::NonlinearPlanarVehicleModel nonlinear_model{parameters};
    const apexlab::PlanarControl nonlinear_input{apexlab::si::radians(0.025), 0.25, 0.0};
    apexlab::PlanarState nonlinear_state{
        apexlab::si::meters(0.0), apexlab::si::meters(0.0), apexlab::si::radians(0.0),
        apexlab::si::meters_per_second(20.0), apexlab::si::meters_per_second(0.0),
        apexlab::si::radians_per_second(0.0),
    };
    const auto nonlinear_derivative =
        [&nonlinear_model, nonlinear_input](const apexlab::PlanarState& value, apexlab::Time) {
            return nonlinear_model.derivative(value, nonlinear_input);
        };
    constexpr std::size_t nonlinear_iterations = 100'000;
    constexpr std::size_t nonlinear_warmup_iterations = 2'000;
    time = apexlab::si::seconds(0.0);
    for (std::size_t index = 0; index < nonlinear_warmup_iterations; ++index) {
        nonlinear_state = apexlab::integrate_rk4(nonlinear_state, time, dt, nonlinear_derivative);
        time += dt;
    }
    nonlinear_state = {
        apexlab::si::meters(0.0), apexlab::si::meters(0.0), apexlab::si::radians(0.0),
        apexlab::si::meters_per_second(20.0), apexlab::si::meters_per_second(0.0),
        apexlab::si::radians_per_second(0.0),
    };
    time = apexlab::si::seconds(0.0);
    const auto nonlinear_start = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < nonlinear_iterations; ++index) {
        nonlinear_state = apexlab::integrate_rk4(nonlinear_state, time, dt, nonlinear_derivative);
        time += dt;
    }
    const auto nonlinear_end = std::chrono::steady_clock::now();
    const double nonlinear_elapsed_s =
        std::chrono::duration<double>(nonlinear_end - nonlinear_start).count();
    const double nonlinear_nanoseconds_per_step =
        nonlinear_elapsed_s * 1.0e9 / static_cast<double>(nonlinear_iterations);

    std::vector<int> iteration_counts;
    iteration_counts.reserve(10'000);
    nonlinear_state = {
        apexlab::si::meters(0.0), apexlab::si::meters(0.0), apexlab::si::radians(0.0),
        apexlab::si::meters_per_second(20.0), apexlab::si::meters_per_second(0.0),
        apexlab::si::radians_per_second(0.0),
    };
    time = apexlab::si::seconds(0.0);
    for (std::size_t index = 0; index < 10'000; ++index) {
        iteration_counts.push_back(nonlinear_model.forces(nonlinear_state, nonlinear_input).solver_iterations);
        nonlinear_state = apexlab::integrate_rk4(nonlinear_state, time, dt, nonlinear_derivative);
        time += dt;
    }
    std::sort(iteration_counts.begin(), iteration_counts.end());
    const double mean_iterations =
        static_cast<double>(std::accumulate(iteration_counts.begin(), iteration_counts.end(), 0LL)) /
        static_cast<double>(iteration_counts.size());
    const double nonlinear_checksum = nonlinear_state.position_x.value() +
                                      nonlinear_state.position_y.value() + nonlinear_state.yaw.value() +
                                      nonlinear_state.velocity_x.value() + nonlinear_state.velocity_y.value() +
                                      nonlinear_state.yaw_rate.value();
    std::cout << "workload=nonlinear-four-tire-rk4"
              << " iterations=" << nonlinear_iterations
              << " warmup_iterations=" << nonlinear_warmup_iterations
              << " elapsed_s=" << nonlinear_elapsed_s
              << " ns_per_step=" << nonlinear_nanoseconds_per_step
              << " solver_mean_iterations=" << mean_iterations
              << " solver_median_iterations=" << iteration_counts[iteration_counts.size() / 2U]
              << " solver_max_iterations=" << iteration_counts.back()
              << " checksum=" << nonlinear_checksum << '\n';
    const apexlab::PeriodicTrack track{apexlab::make_technical_track()};
    constexpr std::size_t geometry_iterations = 200'000;
    double geometry_checksum = 0.0;
    const auto position_start = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < geometry_iterations; ++index) {
        const double s = track.length_m() * static_cast<double>(index % 10000U) / 10000.0;
        const auto frame = track.frame_at_s(s);
        geometry_checksum += frame.position.x * 1.0e-12 + frame.curvature_1_m * 1.0e-9;
    }
    const auto position_end = std::chrono::steady_clock::now();
    const double query_ns = std::chrono::duration<double>(position_end - position_start).count() *
                            1.0e9 / static_cast<double>(geometry_iterations);
    constexpr std::size_t projection_iterations = 10'000;
    double projection_hint = 0.0;
    const auto projection_start = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < projection_iterations; ++index) {
        const double expected_s = track.length_m() * static_cast<double>(index) /
                                  static_cast<double>(projection_iterations);
        const auto point = track.position_at_s(expected_s, 1.0);
        const auto projected = track.project(point, projection_hint, 40.0);
        projection_hint = projected.s_m;
        geometry_checksum += projected.lateral_offset_m * 1.0e-8;
    }
    const auto projection_end = std::chrono::steady_clock::now();
    const double projection_ns =
        std::chrono::duration<double>(projection_end - projection_start).count() * 1.0e9 /
        static_cast<double>(projection_iterations);
    std::cout << "workload=track-geometry queries=" << geometry_iterations
              << " frame_at_s_ns=" << query_ns << " projections=" << projection_iterations
              << " coherent_projection_ns=" << projection_ns
              << " checksum=" << geometry_checksum << '\n';

    const apexlab::PeriodicTrack hairpin{apexlab::make_hairpin_track()};
    double max_hairpin_s_error = 0.0;
    double max_hairpin_lateral_error = 0.0;
    double max_hairpin_reconstruction_error = 0.0;
    double previous_hairpin_s = 0.05 * hairpin.length_m();
    for (std::size_t index = 5; index < 95U; ++index) {
        const double expected_s = hairpin.length_m() * static_cast<double>(index) / 100.0;
        const double expected_offset = index % 2U == 0U ? 2.0 : -2.5;
        const auto point = hairpin.position_at_s(expected_s, expected_offset);
        const auto projected = hairpin.project(point, previous_hairpin_s, 20.0);
        previous_hairpin_s = projected.s_m;
        double s_error = std::abs(projected.s_m - expected_s);
        s_error = std::min(s_error, hairpin.length_m() - s_error);
        const auto reconstructed = hairpin.position_at_s(projected.s_m, projected.lateral_offset_m);
        max_hairpin_s_error = std::max(max_hairpin_s_error, s_error);
        max_hairpin_lateral_error =
            std::max(max_hairpin_lateral_error, std::abs(projected.lateral_offset_m - expected_offset));
        max_hairpin_reconstruction_error =
            std::max(max_hairpin_reconstruction_error,
                     std::hypot(reconstructed.x - point.x, reconstructed.y - point.y));
    }
    std::cout << std::scientific << std::setprecision(9)
              << "workload=hairpin-coherent-projection max_s_error_m=" << max_hairpin_s_error
              << " max_lateral_error_m=" << max_hairpin_lateral_error
              << " max_reconstruction_error_m=" << max_hairpin_reconstruction_error << '\n'
              << std::fixed << std::setprecision(3);

    const apexlab::ReferenceLine reference_line{track};
    const apexlab::SpeedProfile speed_profile{track, reference_line};
    apexlab::LapSimulationOptions lap_options;
    lap_options.timestep = apexlab::si::seconds(0.01);
    lap_options.controller_timestep = apexlab::si::seconds(0.02);
    lap_options.initial_speed_mps = 12.0;
    lap_options.warmup_laps = 0;
    lap_options.timed_laps = 1;
    const auto lap_start = std::chrono::steady_clock::now();
    const auto lap_result = apexlab::simulate_laps(nonlinear_model, track, reference_line,
                                                   speed_profile, lap_options);
    const auto lap_end = std::chrono::steady_clock::now();
    const double lap_wall_s = std::chrono::duration<double>(lap_end - lap_start).count();
    const double lap_simulated_s =
        lap_result.samples.empty() ? 0.0 : lap_result.samples.back().time.value();
    std::cout << "workload=complete-synthetic-lap simulated_s=" << lap_simulated_s
              << " wall_s=" << lap_wall_s << " realtime_factor=" << lap_simulated_s / lap_wall_s
              << " physics_steps="
              << (lap_result.samples.empty() ? 0U : lap_result.samples.size() - 1U)
              << " completed=" << (lap_result.completed ? 1 : 0) << '\n';
    return EXIT_SUCCESS;
}

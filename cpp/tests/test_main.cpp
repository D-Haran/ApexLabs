#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "apexlab/scenario.hpp"
#include "apexlab/lap_simulation.hpp"
#include "apexlab/optimization_replay.hpp"
#include "apexlab/simulation.hpp"
#include "apexlab/track.hpp"
#include "apexlab/vehicle_parameters.hpp"

namespace {

class TestFailure : public std::runtime_error {
  public:
    using std::runtime_error::runtime_error;
};

void expect_true(bool condition, const std::string& message) {
    if (!condition) {
        throw TestFailure(message);
    }
}

void expect_near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        throw TestFailure(message + ": actual=" + std::to_string(actual) + " expected=" +
                          std::to_string(expected) + " tolerance=" + std::to_string(tolerance));
    }
}

template <typename Function> void expect_throws(Function&& function, const std::string& message) {
    try {
        function();
    } catch (const std::exception&) {
        return;
    }
    throw TestFailure(message);
}

[[nodiscard]] apexlab::VehicleParameters force_fixture() {
    apexlab::VehicleParameters parameters;
    parameters.name = "unit-test fixture";
    parameters.provenance = "analytical unit-test fixture";
    parameters.mass = apexlab::si::kilograms(1000.0);
    parameters.aero.air_density_kgpm3 = 1.225;
    parameters.aero.drag_coefficient = 0.0;
    parameters.aero.lift_coefficient_down = 0.0;
    parameters.aero.reference_area_m2 = 0.0;
    parameters.powertrain.maximum_drive_force = apexlab::si::newtons(2000.0);
    parameters.powertrain.driven_wheel_static_load_fraction = 1.0;
    parameters.brakes.maximum_brake_force = apexlab::si::newtons(8000.0);
    parameters.tires.friction_coefficient = 10.0;
    parameters.tires.rolling_resistance_coefficient = 0.0;
    return parameters;
}

[[nodiscard]] apexlab::VehicleParameters planar_fixture() {
    auto parameters = force_fixture();
    parameters.schema_version = apexlab::planar_vehicle_schema_version;
    parameters.name = "planar unit-test fixture";
    parameters.powertrain.maximum_drive_force = apexlab::si::newtons(0.0);
    parameters.brakes.maximum_brake_force = apexlab::si::newtons(0.0);
    parameters.planar = apexlab::PlanarParameters{
        apexlab::si::meters(2.8),
        apexlab::si::meters(1.2),
        apexlab::si::meters(1.6),
        apexlab::si::kilogram_square_meters(2500.0),
        100000.0,
        75000.0,
    };
    return parameters;
}

[[nodiscard]] apexlab::VehicleParameters nonlinear_fixture() {
    auto parameters = planar_fixture();
    parameters.schema_version = apexlab::nonlinear_vehicle_schema_version;
    parameters.name = "nonlinear unit-test fixture";
    parameters.powertrain.maximum_drive_force = apexlab::si::newtons(8000.0);
    parameters.brakes.maximum_brake_force = apexlab::si::newtons(14000.0);
    parameters.tires.friction_coefficient = 1.2;
    parameters.planar->front_cornering_stiffness_n_per_rad = 100000.0;
    parameters.planar->rear_cornering_stiffness_n_per_rad = 90000.0;
    parameters.nonlinear_planar = apexlab::NonlinearPlanarParameters{
        apexlab::si::meters(1.6), apexlab::si::meters(1.6), apexlab::si::meters(0.5),
        0.55, 0.6, "RWD", 0.0, 1.2, apexlab::si::newtons(2500.0), -0.08,
        1.0e-6, 80, 0.5,
    };
    return parameters;
}

[[nodiscard]] std::vector<apexlab::TelemetrySample>
run_constant(const apexlab::LongitudinalModel& model, apexlab::LongitudinalState state,
             apexlab::SimulationOptions options, apexlab::DriverInput input) {
    return apexlab::simulate(
        model, state, options,
        [input](apexlab::Time, const apexlab::LongitudinalState&) { return input; });
}

void test_no_forces_preserve_velocity() {
    auto parameters = force_fixture();
    parameters.powertrain.maximum_drive_force = apexlab::si::newtons(0.0);
    parameters.brakes.maximum_brake_force = apexlab::si::newtons(0.0);
    const apexlab::LongitudinalModel model{parameters};
    const auto result =
        run_constant(model, {apexlab::si::meters(0.0), apexlab::si::meters_per_second(12.5)},
                     {apexlab::si::seconds(4.0), apexlab::si::seconds(0.01),
                      apexlab::IntegratorKind::rk4, false},
                     {});
    expect_near(result.back().state.velocity.value(), 12.5, 1.0e-12,
                "force-free velocity must remain constant");
    expect_near(result.back().state.position.value(), 50.0, 1.0e-10,
                "force-free position must be linear in time");
}

void test_rk4_constant_acceleration_matches_analytic_solution() {
    const apexlab::LongitudinalModel model{force_fixture()};
    const auto result =
        run_constant(model, {apexlab::si::meters(0.0), apexlab::si::meters_per_second(0.0)},
                     {apexlab::si::seconds(10.0), apexlab::si::seconds(0.1),
                      apexlab::IntegratorKind::rk4, false},
                     {1.0, 0.0});
    expect_near(result.back().state.velocity.value(), 20.0, 1.0e-11,
                "constant-force final velocity");
    expect_near(result.back().state.position.value(), 100.0, 1.0e-10,
                "constant-force final position");
}

void test_euler_has_expected_constant_acceleration_position_error() {
    const apexlab::LongitudinalModel model{force_fixture()};
    const auto result =
        run_constant(model, {apexlab::si::meters(0.0), apexlab::si::meters_per_second(0.0)},
                     {apexlab::si::seconds(10.0), apexlab::si::seconds(1.0),
                      apexlab::IntegratorKind::euler, false},
                     {1.0, 0.0});
    expect_near(result.back().state.velocity.value(), 20.0, 1.0e-12,
                "Euler integrates constant acceleration velocity exactly");
    expect_near(result.back().state.position.value(), 90.0, 1.0e-12,
                "forward Euler uses the beginning-of-step velocity");
}

void test_force_directions_and_downforce() {
    auto parameters = force_fixture();
    parameters.aero.reference_area_m2 = 2.0;
    parameters.aero.drag_coefficient = 0.3;
    parameters.aero.lift_coefficient_down = 0.5;
    parameters.tires.rolling_resistance_coefficient = 0.01;
    const apexlab::LongitudinalModel model{parameters};
    const auto forward =
        model.forces({apexlab::si::meters(0.0), apexlab::si::meters_per_second(20.0)}, {});
    const auto reverse =
        model.forces({apexlab::si::meters(0.0), apexlab::si::meters_per_second(-20.0)}, {});
    expect_true(forward.aerodynamic_drag.value() < 0.0, "drag must oppose forward motion");
    expect_true(reverse.aerodynamic_drag.value() > 0.0, "drag must oppose reverse motion");
    expect_true(forward.rolling_resistance.value() < 0.0,
                "rolling resistance must oppose forward motion");
    expect_true(forward.aerodynamic_downforce.value() > 0.0,
                "downforce magnitude must be positive at speed");
    expect_true(forward.traction_limit.value() > parameters.tires.friction_coefficient *
                                                     parameters.mass.value() *
                                                     apexlab::standard_gravity_mps2,
                "downforce must increase the aggregate traction limit");
}

void test_traction_limits_drive_and_braking() {
    auto parameters = force_fixture();
    parameters.powertrain.maximum_drive_force = apexlab::si::newtons(100000.0);
    parameters.brakes.maximum_brake_force = apexlab::si::newtons(100000.0);
    parameters.powertrain.driven_wheel_static_load_fraction = 0.5;
    parameters.tires.friction_coefficient = 1.0;
    const apexlab::LongitudinalModel model{parameters};
    const auto forces =
        model.forces({apexlab::si::meters(0.0), apexlab::si::meters_per_second(10.0)}, {1.0, 1.0});
    expect_near(forces.drive.value(), 0.5 * forces.traction_limit.value(), 1.0e-9,
                "drive force must use driven-wheel traction fraction");
    expect_near(forces.brake.value(), -forces.traction_limit.value(), 1.0e-9,
                "braking must use aggregate traction limit");
}

void test_braking_stop_event() {
    const apexlab::LongitudinalModel model{force_fixture()};
    const auto result = run_constant(
        model, {apexlab::si::meters(0.0), apexlab::si::meters_per_second(10.0)},
        {apexlab::si::seconds(5.0), apexlab::si::seconds(0.03), apexlab::IntegratorKind::rk4, true},
        {0.0, 1.0});
    expect_near(result.back().time.value(), 1.25, 1.0e-10, "constant braking stop time");
    expect_near(result.back().state.position.value(), 6.25, 1.0e-9,
                "constant braking stop distance");
    expect_near(result.back().state.velocity.value(), 0.0, 0.0,
                "stop event must not reverse velocity");
}

void test_braking_with_drag_matches_analytic_distance() {
    auto parameters = force_fixture();
    parameters.aero.air_density_kgpm3 = 1.2;
    parameters.aero.drag_coefficient = 0.5;
    parameters.aero.reference_area_m2 = 2.0;
    const apexlab::LongitudinalModel model{parameters};
    const auto result =
        run_constant(model, {apexlab::si::meters(0.0), apexlab::si::meters_per_second(30.0)},
                     {apexlab::si::seconds(10.0), apexlab::si::seconds(0.02),
                      apexlab::IntegratorKind::rk4, true},
                     {0.0, 1.0});
    const double drag_factor_n_per_mps2 = 0.5 * 1.2 * 0.5 * 2.0;
    const double expected_distance = parameters.mass.value() / (2.0 * drag_factor_n_per_mps2) *
                                     std::log(1.0 + drag_factor_n_per_mps2 * 30.0 * 30.0 / 8000.0);
    expect_near(result.back().state.position.value(), expected_distance, 1.0e-8,
                "braking with quadratic drag stop distance");
    expect_near(result.back().state.velocity.value(), 0.0, 0.0,
                "drag braking event must stop at zero velocity");
}

void test_configuration_loader() {
    const std::filesystem::path path =
        std::filesystem::path{APEXLAB_SOURCE_DIR} / "configs/vehicles/generic_performance_car.json";
    const auto parameters = apexlab::load_vehicle_parameters(path);
    expect_true(parameters.schema_version == 1, "configuration schema version");
    expect_near(parameters.mass.value(), 1450.0, 0.0, "configuration mass");
    expect_near(parameters.aero.drag_coefficient, 0.32, 0.0, "configuration drag");
    expect_true(!parameters.provenance.empty(), "configuration provenance is required");

    const std::filesystem::path planar_path =
        std::filesystem::path{APEXLAB_SOURCE_DIR} / "configs/vehicles/planar_neutral.json";
    const auto planar_parameters = apexlab::load_vehicle_parameters(planar_path);
    expect_true(planar_parameters.schema_version == apexlab::planar_vehicle_schema_version,
                "planar configuration schema version");
    expect_true(planar_parameters.planar.has_value(), "planar configuration fields load");
    expect_near(planar_parameters.planar->cg_to_front_axle.value() +
                    planar_parameters.planar->cg_to_rear_axle.value(),
                planar_parameters.planar->wheelbase.value(), 1.0e-15,
                "loaded planar geometry is consistent");

    const std::filesystem::path nonlinear_path =
        std::filesystem::path{APEXLAB_SOURCE_DIR} /
        "configs/vehicles/generic_nonlinear_performance_car.json";
    const auto nonlinear_parameters = apexlab::load_vehicle_parameters(nonlinear_path);
    expect_true(nonlinear_parameters.schema_version == apexlab::nonlinear_vehicle_schema_version,
                "nonlinear configuration schema version");
    expect_true(nonlinear_parameters.nonlinear_planar.has_value(),
                "nonlinear configuration fields load");
    expect_near(nonlinear_parameters.nonlinear_planar->drive_front_fraction, 0.0, 0.0,
                "reference nonlinear vehicle is rear drive");
}

void test_deterministic_runs_are_identical() {
    const apexlab::LongitudinalModel model{force_fixture()};
    const apexlab::SimulationOptions options{apexlab::si::seconds(2.0), apexlab::si::seconds(0.007),
                                             apexlab::IntegratorKind::rk4, false};
    const apexlab::LongitudinalState state{apexlab::si::meters(0.0),
                                           apexlab::si::meters_per_second(3.0)};
    const auto first = run_constant(model, state, options, {0.6, 0.0});
    const auto second = run_constant(model, state, options, {0.6, 0.0});
    expect_true(first.size() == second.size(), "deterministic sample counts");
    for (std::size_t index = 0; index < first.size(); ++index) {
        expect_true(first[index].time.value() == second[index].time.value(),
                    "deterministic sample time");
        expect_true(first[index].state.position.value() == second[index].state.position.value(),
                    "deterministic position");
        expect_true(first[index].state.velocity.value() == second[index].state.velocity.value(),
                    "deterministic velocity");
    }
}

void test_planar_geometry_validation() {
    auto parameters = planar_fixture();
    apexlab::validate(parameters);
    parameters.planar->wheelbase = apexlab::si::meters(2.9);
    expect_throws([&parameters] { apexlab::validate(parameters); },
                  "inconsistent axle geometry must be rejected");
    parameters = planar_fixture();
    parameters.planar->yaw_moment_of_inertia = apexlab::si::kilogram_square_meters(0.0);
    expect_throws([&parameters] { apexlab::validate(parameters); },
                  "nonpositive yaw inertia must be rejected");
}

void test_planar_slip_and_tire_force_signs() {
    const apexlab::PlanarVehicleModel model{planar_fixture()};
    const apexlab::PlanarState left_velocity{
        apexlab::si::meters(0.0),
        apexlab::si::meters(0.0),
        apexlab::si::radians(0.0),
        apexlab::si::meters_per_second(10.0),
        apexlab::si::meters_per_second(1.0),
        apexlab::si::radians_per_second(0.0),
    };
    const auto opposing = model.forces(left_velocity, {});
    expect_true(opposing.front_slip_angle.value() < 0.0,
                "leftward axle velocity must produce negative front slip");
    expect_true(opposing.rear_slip_angle.value() < 0.0,
                "leftward axle velocity must produce negative rear slip");
    expect_true(opposing.front_lateral_force.value() < 0.0,
                "front tire force must oppose leftward axle velocity");
    expect_true(opposing.rear_lateral_force.value() < 0.0,
                "rear tire force must oppose leftward axle velocity");

    const apexlab::PlanarState straight{
        apexlab::si::meters(0.0),
        apexlab::si::meters(0.0),
        apexlab::si::radians(0.0),
        apexlab::si::meters_per_second(10.0),
        apexlab::si::meters_per_second(0.0),
        apexlab::si::radians_per_second(0.0),
    };
    const auto steered = model.forces(straight, {apexlab::si::radians(0.05), 0.0, 0.0});
    expect_near(steered.front_slip_angle.value(), 0.05, 1.0e-15,
                "positive steer produces positive front slip");
    expect_true(steered.front_lateral_force.value() > 0.0,
                "positive front slip produces left tire force");
    expect_true(steered.yaw_moment_nm > 0.0,
                "positive steer produces counter-clockwise yaw moment");
}

void test_planar_static_loads_and_low_speed_finiteness() {
    const apexlab::PlanarVehicleModel model{planar_fixture()};
    const apexlab::PlanarState state{};
    const auto forces = model.forces(state, {apexlab::si::radians(0.1), 0.0, 0.0});
    expect_near(forces.front_normal_load.value(),
                1000.0 * apexlab::standard_gravity_mps2 * 1.6 / 2.8, 1.0e-10,
                "front static load follows rear CG lever arm");
    expect_near(forces.rear_normal_load.value(),
                1000.0 * apexlab::standard_gravity_mps2 * 1.2 / 2.8, 1.0e-10,
                "rear static load follows front CG lever arm");
    expect_near(forces.front_lateral_force.value(), 0.0, 0.0,
                "lateral tire force fades to zero at rest");
    const auto derivative = model.derivative(state, {apexlab::si::radians(0.1), 0.0, 0.0});
    expect_true(std::isfinite(derivative.yaw_acceleration.value()),
                "zero-speed yaw acceleration must remain finite");
}

void test_planar_coordinate_transform() {
    const apexlab::PlanarVehicleModel model{planar_fixture()};
    const apexlab::PlanarState state{
        apexlab::si::meters(0.0),
        apexlab::si::meters(0.0),
        apexlab::si::radians(std::numbers::pi / 2.0),
        apexlab::si::meters_per_second(10.0),
        apexlab::si::meters_per_second(0.0),
        apexlab::si::radians_per_second(0.0),
    };
    const auto derivative = model.derivative(state, {});
    expect_near(derivative.position_x_rate.value(), 0.0, 1.0e-14,
                "body-forward velocity at 90 degrees has zero world x rate");
    expect_near(derivative.position_y_rate.value(), 10.0, 1.0e-14,
                "body-forward velocity at 90 degrees maps to world y rate");
}

void test_planar_straight_line_symmetry() {
    const apexlab::PlanarVehicleModel model{planar_fixture()};
    const apexlab::PlanarState initial{
        apexlab::si::meters(0.0),
        apexlab::si::meters(0.0),
        apexlab::si::radians(0.0),
        apexlab::si::meters_per_second(20.0),
        apexlab::si::meters_per_second(0.0),
        apexlab::si::radians_per_second(0.0),
    };
    const apexlab::SimulationOptions options{
        apexlab::si::seconds(5.0),
        apexlab::si::seconds(0.005),
        apexlab::IntegratorKind::rk4,
        false,
    };
    const auto result = apexlab::simulate_planar(
        model, initial, options,
        [](apexlab::Time, const apexlab::PlanarState&) { return apexlab::PlanarControl{}; });
    for (const auto& sample : result) {
        expect_near(sample.state.position_y.value(), 0.0, 0.0,
                    "zero-steer world y must remain exactly zero");
        expect_near(sample.state.velocity_y.value(), 0.0, 0.0,
                    "zero-steer lateral velocity must remain exactly zero");
        expect_near(sample.state.yaw.value(), 0.0, 0.0, "zero-steer yaw must remain exactly zero");
        expect_near(sample.state.yaw_rate.value(), 0.0, 0.0,
                    "zero-steer yaw rate must remain exactly zero");
    }
}

void test_planar_determinism_and_telemetry() {
    const apexlab::PlanarVehicleModel model{planar_fixture()};
    const apexlab::PlanarState initial{
        apexlab::si::meters(0.0),
        apexlab::si::meters(0.0),
        apexlab::si::radians(0.0),
        apexlab::si::meters_per_second(15.0),
        apexlab::si::meters_per_second(0.0),
        apexlab::si::radians_per_second(0.0),
    };
    const apexlab::SimulationOptions options{
        apexlab::si::seconds(1.0),
        apexlab::si::seconds(0.007),
        apexlab::IntegratorKind::rk4,
        false,
    };
    const auto controller = [](apexlab::Time time, const apexlab::PlanarState&) {
        return apexlab::PlanarControl{apexlab::si::radians(time.value() < 0.2 ? 0.0 : 0.01), 0.0,
                                      0.0};
    };
    const auto first = apexlab::simulate_planar(model, initial, options, controller);
    const auto second = apexlab::simulate_planar(model, initial, options, controller);
    expect_true(first.size() == second.size(), "planar deterministic sample count");
    for (std::size_t index = 0; index < first.size(); ++index) {
        expect_true(first[index].state.position_x.value() == second[index].state.position_x.value(),
                    "planar deterministic x position");
        expect_true(first[index].state.position_y.value() == second[index].state.position_y.value(),
                    "planar deterministic y position");
        expect_true(first[index].state.yaw_rate.value() == second[index].state.yaw_rate.value(),
                    "planar deterministic yaw rate");
    }
    const auto& final = first.back();
    expect_near(final.lateral_acceleration.value(),
                final.forces.body_lateral_force.value() / model.parameters().mass.value(), 1.0e-15,
                "telemetry lateral acceleration uses computed body force");
    expect_true(final.schema_version == apexlab::planar_telemetry_schema_version,
                "planar telemetry uses schema version 2");

    const auto path = std::filesystem::temp_directory_path() / "apexlab-planar-test.csv";
    apexlab::write_planar_telemetry_csv(path, first);
    std::ifstream stream(path);
    std::string header;
    std::getline(stream, header);
    expect_true(header.find("front_slip_angle_rad") != std::string::npos,
                "planar telemetry header contains front slip");
    expect_true(header.find("lateral_accel_m_s2") != std::string::npos,
                "planar telemetry header contains lateral acceleration units");
    std::filesystem::remove(path);
}

void test_scenario_profiles_and_loader() {
    apexlab::ScalarProfile step;
    step.kind = apexlab::ProfileKind::step;
    step.initial_value = 0.0;
    step.final_value = 0.1;
    step.start_time = apexlab::si::seconds(1.0);
    expect_near(step.value_at(apexlab::si::seconds(0.999)), 0.0, 0.0,
                "step profile before transition");
    expect_near(step.value_at(apexlab::si::seconds(1.0)), 0.1, 0.0, "step profile at transition");

    const auto path =
        std::filesystem::path{APEXLAB_SOURCE_DIR} / "configs/scenarios/step_steer.json";
    const auto scenario = apexlab::load_planar_scenario(path);
    expect_true(scenario.integrator == apexlab::IntegratorKind::rk4, "scenario integrator load");
    expect_near(scenario.control_at(apexlab::si::seconds(0.5)).steering_angle.value(), 0.0, 0.0,
                "loaded step profile before transition");
    expect_near(scenario.control_at(apexlab::si::seconds(1.0)).steering_angle.value(), 0.02, 0.0,
                "loaded step profile after transition");
}

void test_nonlinear_tire_curve_and_symmetry() {
    const apexlab::NonlinearTireModel tire{50000.0, 1.2, apexlab::si::newtons(2500.0), -0.08};
    const auto zero = tire.evaluate(
        {apexlab::si::newtons(2500.0), apexlab::si::radians(0.0), apexlab::si::newtons(0.0)});
    expect_near(zero.lateral_force.value(), 0.0, 0.0, "zero slip produces zero lateral force");
    constexpr double small_angle = 1.0e-7;
    const auto small = tire.evaluate({apexlab::si::newtons(2500.0),
                                      apexlab::si::radians(small_angle),
                                      apexlab::si::newtons(0.0)});
    expect_near(small.lateral_force.value() / small_angle, 50000.0, 0.2,
                "Fiala small-angle slope equals cornering stiffness");
    const auto positive = tire.evaluate({apexlab::si::newtons(2500.0),
                                         apexlab::si::radians(0.4),
                                         apexlab::si::newtons(0.0)});
    const auto negative = tire.evaluate({apexlab::si::newtons(2500.0),
                                         apexlab::si::radians(-0.4),
                                         apexlab::si::newtons(0.0)});
    expect_near(positive.lateral_force.value(), -negative.lateral_force.value(), 1.0e-10,
                "nonlinear tire has odd lateral symmetry");
    expect_true(positive.saturated, "large slip marks Fiala saturation");
    expect_true(std::abs(positive.lateral_force.value()) <= positive.force_capacity.value(),
                "large-slip lateral force remains finite");
}

void test_nonlinear_tire_load_sensitivity_and_combined_grip() {
    const apexlab::NonlinearTireModel tire{50000.0, 1.2, apexlab::si::newtons(2500.0), -0.1};
    const auto light = tire.evaluate({apexlab::si::newtons(2000.0), apexlab::si::radians(0.4),
                                      apexlab::si::newtons(0.0)});
    const auto heavy = tire.evaluate({apexlab::si::newtons(4000.0), apexlab::si::radians(0.4),
                                      apexlab::si::newtons(0.0)});
    expect_true(heavy.force_capacity.value() > light.force_capacity.value(),
                "load increases absolute tire capacity");
    expect_true(heavy.effective_friction_coefficient < light.effective_friction_coefficient,
                "load sensitivity decreases effective mu");
    const auto combined = tire.evaluate({apexlab::si::newtons(2500.0),
                                         apexlab::si::radians(0.08),
                                         apexlab::si::newtons(2800.0)});
    expect_true(combined.requested_friction_utilization > 1.0 && combined.saturated,
                "over-budget combined request is reported");
    expect_true(combined.friction_utilization <= 1.0 + 1.0e-14,
                "delivered combined force lies inside friction circle");
    expect_true(std::abs(combined.lateral_force.value()) <
                    std::abs(combined.requested_lateral_force.value()),
                "longitudinal demand reduces delivered lateral force");
}

void test_nonlinear_normal_load_transfer() {
    const apexlab::NonlinearPlanarVehicleModel model{nonlinear_fixture()};
    const auto static_loads = model.normal_loads(apexlab::si::meters_per_second_squared(0.0),
                                                 apexlab::si::meters_per_second_squared(0.0));
    double total = 0.0;
    for (const auto load : static_loads.wheel) {
        total += load.value();
    }
    expect_near(total, 1000.0 * apexlab::standard_gravity_mps2, 1.0e-10,
                "wheel loads conserve vehicle weight");
    expect_near(static_loads.wheel[0].value() + static_loads.wheel[1].value(),
                1000.0 * apexlab::standard_gravity_mps2 * 1.6 / 2.8, 1.0e-10,
                "static front axle load follows CG location");
    const auto braking = model.normal_loads(apexlab::si::meters_per_second_squared(-5.0),
                                            apexlab::si::meters_per_second_squared(0.0));
    const auto acceleration = model.normal_loads(apexlab::si::meters_per_second_squared(5.0),
                                                 apexlab::si::meters_per_second_squared(0.0));
    expect_true(braking.wheel[0].value() > static_loads.wheel[0].value(),
                "braking increases front load");
    expect_true(acceleration.wheel[2].value() > static_loads.wheel[2].value(),
                "acceleration increases rear load");
    const auto left_turn = model.normal_loads(apexlab::si::meters_per_second_squared(0.0),
                                              apexlab::si::meters_per_second_squared(4.0));
    const auto right_turn = model.normal_loads(apexlab::si::meters_per_second_squared(0.0),
                                               apexlab::si::meters_per_second_squared(-4.0));
    expect_true(left_turn.wheel[1].value() > left_turn.wheel[0].value(),
                "left turn loads outside right-front tire");
    expect_near(left_turn.wheel[0].value(), right_turn.wheel[1].value(), 1.0e-12,
                "mirrored lateral acceleration mirrors front loads");
    expect_near(left_turn.wheel[2].value(), right_turn.wheel[3].value(), 1.0e-12,
                "mirrored lateral acceleration mirrors rear loads");
    expect_throws(
        [&model] {
            static_cast<void>(model.normal_loads(apexlab::si::meters_per_second_squared(0.0),
                                                 apexlab::si::meters_per_second_squared(40.0)));
        },
        "negative contact load must surface as an envelope failure");
}

void test_nonlinear_wheel_kinematics_transform_and_iteration() {
    const apexlab::NonlinearPlanarVehicleModel model{nonlinear_fixture()};
    const apexlab::PlanarState state{apexlab::si::meters(0.0), apexlab::si::meters(0.0),
                                     apexlab::si::radians(0.0),
                                     apexlab::si::meters_per_second(20.0),
                                     apexlab::si::meters_per_second(0.0),
                                     apexlab::si::radians_per_second(0.0)};
    const auto straight = model.forces(state, {});
    expect_near(straight.wheel[0].slip_angle.value(), straight.wheel[1].slip_angle.value(), 0.0,
                "symmetric front wheel slips match");
    expect_near(straight.wheel[2].slip_angle.value(), straight.wheel[3].slip_angle.value(), 0.0,
                "symmetric rear wheel slips match");
    const auto steered =
        model.forces(state, {apexlab::si::radians(0.04), 0.0, 0.0});
    expect_true(steered.body_lateral_force.value() > 0.0,
                "positive steering transforms to positive body lateral force");
    expect_true(steered.body_longitudinal_force.value() < 0.0,
                "steered lateral force has rearward body component");
    expect_true(steered.yaw_moment_nm > 0.0,
                "front lateral forces produce positive yaw moment");
    expect_true(steered.solver_converged && steered.solver_iterations <= 80,
                "typical coupled force/load solve converges");
}

void test_nonlinear_solver_failure_determinism_and_telemetry() {
    auto failing_parameters = nonlinear_fixture();
    failing_parameters.nonlinear_planar->force_iteration_tolerance_mps2 = 1.0e-15;
    failing_parameters.nonlinear_planar->force_iteration_max_iterations = 1;
    const apexlab::NonlinearPlanarVehicleModel failing_model{failing_parameters};
    const apexlab::PlanarState initial{apexlab::si::meters(0.0), apexlab::si::meters(0.0),
                                       apexlab::si::radians(0.0),
                                       apexlab::si::meters_per_second(15.0),
                                       apexlab::si::meters_per_second(0.0),
                                       apexlab::si::radians_per_second(0.0)};
    expect_throws(
        [&failing_model, &initial] {
            static_cast<void>(failing_model.forces(
                initial, {apexlab::si::radians(0.03), 0.0, 0.0}));
        },
        "nonconvergent solve must be reported");

    const apexlab::NonlinearPlanarVehicleModel model{nonlinear_fixture()};
    const apexlab::SimulationOptions options{apexlab::si::seconds(0.1),
                                             apexlab::si::seconds(0.01),
                                             apexlab::IntegratorKind::rk4, false};
    const auto controller = [](apexlab::Time, const apexlab::PlanarState&) {
        return apexlab::PlanarControl{apexlab::si::radians(0.01), 0.0, 0.0};
    };
    const auto first = apexlab::simulate_nonlinear_planar(model, initial, options, controller);
    const auto second = apexlab::simulate_nonlinear_planar(model, initial, options, controller);
    expect_true(first.back().state.yaw_rate.value() == second.back().state.yaw_rate.value(),
                "nonlinear simulation is deterministic");
    const auto path = std::filesystem::temp_directory_path() / "apexlab-nonlinear-test.csv";
    apexlab::write_nonlinear_planar_telemetry_csv(path, first);
    std::ifstream stream(path);
    std::string header;
    std::getline(stream, header);
    expect_true(header.find("friction_utilization_fr") != std::string::npos,
                "nonlinear telemetry contains per-wheel utilization");
    expect_true(header.find("solver_iterations") != std::string::npos,
                "nonlinear telemetry contains convergence diagnostics");
    std::filesystem::remove(path);
}

void test_track_periodicity_arc_length_heading_and_curvature() {
    constexpr double radius = 50.0;
    const apexlab::PeriodicTrack track{apexlab::make_circle_track(radius, 6.0, 64), 512};
    const double expected_length = 2.0 * std::numbers::pi * radius;
    expect_near(track.length_m(), expected_length, 2.0e-3, "circle spline arc length");
    const auto start = track.frame_at_s(0.0);
    const auto end = track.frame_at_s(track.length_m());
    expect_near(start.position.x, end.position.x, 1.0e-12, "periodic position x");
    expect_near(start.position.y, end.position.y, 1.0e-12, "periodic position y");
    expect_near(start.heading_rad, std::numbers::pi / 2.0, 2.0e-3, "circle start heading");
    for (int index = 0; index < 20; ++index) {
        const auto frame = track.frame_at_s(track.length_m() * static_cast<double>(index) / 20.0);
        expect_near(frame.curvature_1_m, 1.0 / radius, 3.0e-4, "circle signed curvature");
    }
    const apexlab::PeriodicTrack straight{apexlab::make_straight_test_path(200.0, 5.0)};
    expect_near(straight.length_m(), 200.0, 1.0e-10, "open straight arc length");
    expect_near(straight.frame_at_s(100.0).heading_rad, 0.0, 1.0e-12, "straight heading");
    expect_near(straight.frame_at_s(100.0).curvature_1_m, 0.0, 1.0e-12, "straight curvature");
    const auto straight_projection = straight.project({75.0, 3.0}, 74.0, 20.0);
    expect_near(straight_projection.s_m, 75.0, 1.0e-7, "straight projection progress");
    expect_near(straight_projection.lateral_offset_m, 3.0, 1.0e-12, "straight projection offset");
}

void test_track_projection_roundtrip_wrap_and_boundaries() {
    const apexlab::PeriodicTrack track{apexlab::make_hairpin_track()};
    for (int index = 5; index < 95; ++index) {
        const double expected_s = track.length_m() * static_cast<double>(index) / 100.0;
        const double expected_offset = index % 2 == 0 ? 2.0 : -2.5;
        const auto point = track.position_at_s(expected_s, expected_offset);
        const auto projected = track.project(point, expected_s, 20.0);
        double s_error = std::abs(projected.s_m - expected_s);
        s_error = std::min(s_error, track.length_m() - s_error);
        expect_near(s_error, 0.0, 2.0e-3, "coherent projection s roundtrip");
        expect_near(projected.lateral_offset_m, expected_offset, 2.0e-5,
                    "projection lateral roundtrip");
    }
    expect_true(track.contains(10.0, 6.9), "left inside boundary");
    expect_true(!track.contains(10.0, 7.1), "left outside boundary");
    expect_true(track.contains(10.0, -6.9), "right inside boundary");
    expect_true(!track.contains(10.0, -7.1), "right outside boundary");
    expect_near(track.wrap_s(-1.0), track.length_m() - 1.0, 1.0e-12, "negative progress wrap");
}

void test_track_elevation_grade_and_legacy_loading() {
    auto definition = apexlab::make_circle_track(50.0, 6.0, 32);
    for (std::size_t index = 0; index < definition.control_points.size(); ++index) {
        const double angle = 2.0 * std::numbers::pi * static_cast<double>(index) /
                             static_cast<double>(definition.control_points.size());
        definition.control_points[index].elevation_m = 10.0 * std::sin(angle);
    }
    const apexlab::PeriodicTrack elevated{definition, 256};
    const auto start = elevated.frame_at_s(0.0);
    const auto opposite = elevated.frame_at_s(elevated.length_m() * 0.5);
    expect_near(start.elevation_m, 0.0, 1.0e-12, "elevation interpolates at control point");
    expect_true(start.grade > 0.15, "elevation profile exposes positive grade");
    expect_true(opposite.grade < -0.15, "elevation profile exposes negative grade");
    const auto position = elevated.position_3d_at_s(elevated.length_m() * 0.25);
    expect_true(position.z > 9.9, "3D track position carries elevation");

    const auto legacy = apexlab::load_track_definition(
        std::filesystem::path{APEXLAB_SOURCE_DIR} / "configs/tracks/technical_test_circuit.json");
    expect_true(legacy.schema_version == 1, "legacy track schema remains loadable");
    expect_near(legacy.control_points.front().elevation_m, 0.0, 0.0,
                "legacy track elevation defaults to zero");
}

void test_imported_real_track_lengths_and_spa_elevation() {
    struct ExpectedTrack {
        const char* filename;
        double imported_length_m;
    };
    const std::vector<ExpectedTrack> expected{
        {"spa-francorchamps.json", 7014.887},
        {"monza.json", 5796.971},
        {"silverstone.json", 5890.022},
        {"suzuka.json", 5805.847},
    };
    for (const auto& item : expected) {
        const auto definition = apexlab::load_track_definition(
            std::filesystem::path{APEXLAB_SOURCE_DIR} / "configs/tracks" / item.filename);
        expect_true(definition.schema_version == 2, "real tracks use schema v2");
        const apexlab::PeriodicTrack track{definition};
        expect_near(track.length_m(), item.imported_length_m, 0.02,
                    "C++ track length matches importer validation");
    }

    const apexlab::PeriodicTrack spa{apexlab::load_track_definition(
        std::filesystem::path{APEXLAB_SOURCE_DIR} / "configs/tracks/spa-francorchamps.json")};
    double minimum_elevation = std::numeric_limits<double>::infinity();
    double maximum_elevation = -std::numeric_limits<double>::infinity();
    double maximum_grade = 0.0;
    for (int index = 0; index < 400; ++index) {
        const auto frame = spa.frame_at_s(spa.length_m() * static_cast<double>(index) / 400.0);
        minimum_elevation = std::min(minimum_elevation, frame.elevation_m);
        maximum_elevation = std::max(maximum_elevation, frame.elevation_m);
        maximum_grade = std::max(maximum_grade, std::abs(frame.grade));
    }
    expect_true(maximum_elevation - minimum_elevation > 95.0,
                "Spa retains its terrain-derived elevation range");
    expect_true(maximum_grade > 0.03 && maximum_grade < 0.30,
                "Spa exposes non-flat but numerically bounded grade");
}

void test_reference_speed_profile_and_lap_simulation() {
    const apexlab::PeriodicTrack track{apexlab::make_circle_track(80.0, 8.0, 64)};
    const apexlab::ReferenceLine line{track, 0.0};
    const apexlab::SpeedProfile profile{
        track, line, apexlab::SpeedProfileOptions{7.0, 2.5, 5.0, 25.0, 0.70, 1.0}};
    expect_true(profile.values().size() > 100U, "dense spatial speed profile");
    for (double speed : profile.values()) {
        expect_true(speed > 0.0 && speed <= 17.0, "circle profile respects conservative lateral cap");
    }
    const apexlab::NonlinearPlanarVehicleModel model{nonlinear_fixture()};
    apexlab::LapSimulationOptions options;
    options.timestep = apexlab::si::seconds(0.01);
    options.controller_timestep = apexlab::si::seconds(0.02);
    options.initial_speed_mps = 14.0;
    options.warmup_laps = 0;
    options.timed_laps = 1;
    options.maximum_duration_s = 60.0;
    const auto first = apexlab::simulate_laps(model, track, line, profile, options);
    const auto second = apexlab::simulate_laps(model, track, line, profile, options);
    expect_true(first.completed && first.completed_lap_times_s.size() == 1U,
                "forward progress completes one circle lap");
    expect_true(first.completed_sector_times_s.size() == 3U,
                "generalized sector timer records all ordered sectors");
    expect_true(first.completed_lap_times_s == second.completed_lap_times_s &&
                    first.samples.back().state.position_x.value() == second.samples.back().state.position_x.value(),
                "lap simulation is deterministic");
    double max_error = 0.0;
    for (const auto& sample : first.samples) max_error = std::max(max_error, std::abs(sample.lateral_error_m));
    expect_true(max_error < 8.0, "controller keeps CG within circle track");
    const auto spatial = apexlab::resample_lap_spatially(first.samples, 0, track.length_m(), 5.0);
    expect_true(spatial.size() > 80U, "lap telemetry resamples in track distance");
}

void test_directed_lap_progress_rejects_reverse_finish_crossing() {
    apexlab::DirectedLapProgress backwards{100.0, 1.0};
    backwards.update(99.0);
    backwards.update(1.0);
    backwards.update(99.5);
    expect_true(backwards.completed_laps() == 0,
                "backward and oscillating finish crossings do not count a lap");
    apexlab::DirectedLapProgress forwards{100.0, 0.0};
    for (double s : {25.0, 50.0, 75.0, 99.0, 1.0}) forwards.update(s);
    expect_true(forwards.completed_laps() == 1, "one directed traversal counts exactly one lap");
}

void test_optimization_trajectory_periodic_interpolation() {
    std::vector<apexlab::OptimizationTrajectoryNode> nodes;
    for (int index = 0; index < 20; ++index) {
        nodes.push_back({5.0 * index, 0.5 * index, 10.0 + index, 0.01 * index,
                         0.001 * index, 0.0, 0.0, 0.0, 0.02 * index, 0.0});
    }
    const apexlab::OptimizationTrajectory trajectory{100.0, nodes};
    expect_near(trajectory.at_s(2.5).speed_mps, 10.5, 1.0e-12,
                "optimizer trajectory interpolates interior nodes");
    expect_near(trajectory.at_s(97.5).speed_mps, 19.5, 1.0e-12,
                "optimizer trajectory interpolates periodically across finish");
    expect_near(trajectory.at_s(102.5).speed_mps, 10.5, 1.0e-12,
                "optimizer trajectory wraps progress");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"no forces preserve velocity", test_no_forces_preserve_velocity},
        {"RK4 constant acceleration", test_rk4_constant_acceleration_matches_analytic_solution},
        {"Euler reference behavior", test_euler_has_expected_constant_acceleration_position_error},
        {"force directions and downforce", test_force_directions_and_downforce},
        {"traction limits", test_traction_limits_drive_and_braking},
        {"braking stop event", test_braking_stop_event},
        {"braking with drag", test_braking_with_drag_matches_analytic_distance},
        {"configuration loader", test_configuration_loader},
        {"deterministic repeat", test_deterministic_runs_are_identical},
        {"planar geometry validation", test_planar_geometry_validation},
        {"planar slip and tire signs", test_planar_slip_and_tire_force_signs},
        {"planar loads and low speed", test_planar_static_loads_and_low_speed_finiteness},
        {"planar coordinate transform", test_planar_coordinate_transform},
        {"planar straight-line symmetry", test_planar_straight_line_symmetry},
        {"planar determinism and telemetry", test_planar_determinism_and_telemetry},
        {"scenario profiles and loader", test_scenario_profiles_and_loader},
        {"nonlinear tire curve and symmetry", test_nonlinear_tire_curve_and_symmetry},
        {"nonlinear tire load and combined grip", test_nonlinear_tire_load_sensitivity_and_combined_grip},
        {"nonlinear normal load transfer", test_nonlinear_normal_load_transfer},
        {"nonlinear kinematics transform iteration", test_nonlinear_wheel_kinematics_transform_and_iteration},
        {"nonlinear failure determinism telemetry", test_nonlinear_solver_failure_determinism_and_telemetry},
        {"track periodic geometry", test_track_periodicity_arc_length_heading_and_curvature},
        {"track projection and boundaries", test_track_projection_roundtrip_wrap_and_boundaries},
        {"track elevation and legacy loading", test_track_elevation_grade_and_legacy_loading},
        {"real track lengths and Spa elevation", test_imported_real_track_lengths_and_spa_elevation},
        {"reference speed and lap simulation", test_reference_speed_profile_and_lap_simulation},
        {"directed lap progress", test_directed_lap_progress_rejects_reverse_finish_crossing},
        {"optimization trajectory interpolation", test_optimization_trajectory_periodic_interpolation},
    };

    std::size_t failures = 0;
    for (const auto& [name, test] : tests) {
        try {
            test();
            std::cout << "[PASS] " << name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size() - failures << '/' << tests.size() << " tests passed\n";
    return failures == 0U ? 0 : 1;
}

#include "apexlab/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace apexlab {
namespace {

[[nodiscard]] DriverInput clamp_input(DriverInput input) {
    input.throttle = std::clamp(input.throttle, 0.0, 1.0);
    input.brake = std::clamp(input.brake, 0.0, 1.0);
    return input;
}

[[nodiscard]] PlanarControl clamp_input(PlanarControl input) {
    input.throttle = std::clamp(input.throttle, 0.0, 1.0);
    input.brake = std::clamp(input.brake, 0.0, 1.0);
    return input;
}

[[nodiscard]] TelemetrySample sample(const LongitudinalModel& model, const LongitudinalState& state,
                                     Time time, std::uint64_t step, DriverInput input) {
    const LongitudinalForces forces = model.forces(state, input);
    return {
        telemetry_schema_version,
        time,
        step,
        state,
        si::meters_per_second_squared(forces.net().value() / model.parameters().mass.value()),
        input,
        forces,
    };
}

[[nodiscard]] PlanarTelemetrySample sample(const PlanarVehicleModel& model,
                                           const PlanarState& state, Time time, std::uint64_t step,
                                           PlanarControl input) {
    const PlanarForces forces = model.forces(state, input);
    const double mass = model.parameters().mass.value();
    return {
        planar_telemetry_schema_version,
        time,
        step,
        state,
        std::hypot(state.velocity_x.value(), state.velocity_y.value()),
        input,
        forces,
        si::meters_per_second_squared(forces.body_longitudinal_force.value() / mass),
        si::meters_per_second_squared(forces.body_lateral_force.value() / mass),
        si::radians_per_second_squared(forces.yaw_moment_nm /
                                       model.parameters().planar->yaw_moment_of_inertia.value()),
    };
}

[[nodiscard]] NonlinearPlanarTelemetrySample sample(const NonlinearPlanarVehicleModel& model,
                                                     const PlanarState& state, Time time,
                                                     std::uint64_t step, PlanarControl input) {
    const NonlinearPlanarForces forces = model.forces(state, input);
    const double mass = model.parameters().mass.value();
    return {
        nonlinear_planar_telemetry_schema_version,
        time,
        step,
        state,
        std::hypot(state.velocity_x.value(), state.velocity_y.value()),
        input,
        forces,
        si::meters_per_second_squared(forces.body_longitudinal_force.value() / mass),
        si::meters_per_second_squared(forces.body_lateral_force.value() / mass),
        si::radians_per_second_squared(forces.yaw_moment_nm /
                                       model.parameters().planar->yaw_moment_of_inertia.value()),
    };
}

} // namespace

std::vector<TelemetrySample> simulate(const LongitudinalModel& model,
                                      LongitudinalState initial_state,
                                      const SimulationOptions& options,
                                      const DriverController& controller) {
    if (!std::isfinite(options.duration.value()) || options.duration.value() < 0.0) {
        throw std::invalid_argument("simulation duration must be finite and nonnegative");
    }
    if (!std::isfinite(options.timestep.value()) || options.timestep.value() <= 0.0) {
        throw std::invalid_argument("simulation timestep must be finite and positive");
    }
    if (!controller) {
        throw std::invalid_argument("simulation controller must be callable");
    }

    const auto estimated_steps =
        static_cast<std::size_t>(std::ceil(options.duration.value() / options.timestep.value()));
    std::vector<TelemetrySample> samples;
    samples.reserve(estimated_steps + 1U);

    Time time = si::seconds(0.0);
    std::uint64_t step = 0;
    LongitudinalState state = initial_state;
    DriverInput input = clamp_input(controller(time, state));
    samples.push_back(sample(model, state, time, step, input));

    constexpr double time_tolerance_s = 1.0e-13;
    while (time.value() + time_tolerance_s < options.duration.value()) {
        const double remaining = options.duration.value() - time.value();
        const Time timestep = si::seconds(std::min(options.timestep.value(), remaining));
        input = clamp_input(controller(time, state));
        const auto derivative = [&model, input](const LongitudinalState& stage_state, Time) {
            return model.derivative(stage_state, input);
        };

        if (options.stop_when_stationary && state.velocity.value() > 0.0) {
            // Continue resistance forces in the forward-opposing direction while
            // bracketing zero velocity. This smooth continuation is used only for
            // root finding; the reported stopped sample uses the ordinary model.
            const auto continuing_derivative = [&model, input](const LongitudinalState& stage_state,
                                                               Time) {
                return model.derivative_continuing_motion(stage_state, input, 1.0);
            };
            const LongitudinalState continued =
                integrate(options.integrator, state, time, timestep, continuing_derivative);
            constexpr double stop_velocity_tolerance_mps = 1.0e-10;
            if (continued.velocity.value() <= stop_velocity_tolerance_mps) {
                double low_s = 0.0;
                double high_s = timestep.value();
                constexpr int bisection_iterations = 60;
                for (int iteration = 0; iteration < bisection_iterations; ++iteration) {
                    const double midpoint_s = 0.5 * (low_s + high_s);
                    const LongitudinalState midpoint =
                        integrate(options.integrator, state, time, si::seconds(midpoint_s),
                                  continuing_derivative);
                    if (midpoint.velocity.value() > stop_velocity_tolerance_mps) {
                        low_s = midpoint_s;
                    } else {
                        high_s = midpoint_s;
                    }
                }
                LongitudinalState stopped = integrate(options.integrator, state, time,
                                                      si::seconds(high_s), continuing_derivative);
                stopped.velocity = si::meters_per_second(0.0);
                time += si::seconds(high_s);
                ++step;
                const DriverInput stopped_input = clamp_input(controller(time, stopped));
                samples.push_back(sample(model, stopped, time, step, stopped_input));
                break;
            }
        }

        LongitudinalState next = integrate(options.integrator, state, time, timestep, derivative);
        if (options.stop_when_stationary && state.velocity.value() > 0.0 &&
            next.velocity.value() < 0.0) {
            next.velocity = si::meters_per_second(0.0);
        }
        state = next;
        time += timestep;
        ++step;
        const DriverInput next_input = clamp_input(controller(time, state));
        samples.push_back(sample(model, state, time, step, next_input));

        if (options.stop_when_stationary && state.velocity.value() == 0.0 &&
            initial_state.velocity.value() > 0.0) {
            break;
        }
    }

    return samples;
}

std::vector<PlanarTelemetrySample> simulate_planar(const PlanarVehicleModel& model,
                                                   PlanarState initial_state,
                                                   const SimulationOptions& options,
                                                   const PlanarController& controller) {
    if (!std::isfinite(options.duration.value()) || options.duration.value() < 0.0) {
        throw std::invalid_argument("simulation duration must be finite and nonnegative");
    }
    if (!std::isfinite(options.timestep.value()) || options.timestep.value() <= 0.0) {
        throw std::invalid_argument("simulation timestep must be finite and positive");
    }
    if (!controller) {
        throw std::invalid_argument("simulation controller must be callable");
    }
    if (initial_state.velocity_x.value() < 0.0) {
        throw std::invalid_argument("planar simulation does not support negative initial vx");
    }

    const auto estimated_steps =
        static_cast<std::size_t>(std::ceil(options.duration.value() / options.timestep.value()));
    std::vector<PlanarTelemetrySample> samples;
    samples.reserve(estimated_steps + 1U);

    Time time = si::seconds(0.0);
    std::uint64_t step = 0;
    PlanarState state = initial_state;
    samples.push_back(sample(model, state, time, step, clamp_input(controller(time, state))));

    constexpr double time_tolerance_s = 1.0e-13;
    while (time.value() + time_tolerance_s < options.duration.value()) {
        const double remaining = options.duration.value() - time.value();
        const Time timestep = si::seconds(std::min(options.timestep.value(), remaining));
        const auto derivative = [&model, &controller](const PlanarState& stage_state,
                                                      Time stage_time) {
            return model.derivative(stage_state, clamp_input(controller(stage_time, stage_state)));
        };
        state = integrate(options.integrator, state, time, timestep, derivative);
        time += timestep;
        ++step;
        samples.push_back(sample(model, state, time, step, clamp_input(controller(time, state))));
    }
    return samples;
}

std::vector<NonlinearPlanarTelemetrySample>
simulate_nonlinear_planar(const NonlinearPlanarVehicleModel& model, PlanarState initial_state,
                          const SimulationOptions& options, const PlanarController& controller) {
    if (!std::isfinite(options.duration.value()) || options.duration.value() < 0.0) {
        throw std::invalid_argument("simulation duration must be finite and nonnegative");
    }
    if (!std::isfinite(options.timestep.value()) || options.timestep.value() <= 0.0) {
        throw std::invalid_argument("simulation timestep must be finite and positive");
    }
    if (!controller) {
        throw std::invalid_argument("simulation controller must be callable");
    }
    if (initial_state.velocity_x.value() < 0.0) {
        throw std::invalid_argument("nonlinear planar simulation does not support negative initial vx");
    }
    const auto estimated_steps =
        static_cast<std::size_t>(std::ceil(options.duration.value() / options.timestep.value()));
    std::vector<NonlinearPlanarTelemetrySample> samples;
    samples.reserve(estimated_steps + 1U);
    Time time = si::seconds(0.0);
    std::uint64_t step = 0;
    PlanarState state = initial_state;
    samples.push_back(sample(model, state, time, step, clamp_input(controller(time, state))));
    constexpr double time_tolerance_s = 1.0e-13;
    while (time.value() + time_tolerance_s < options.duration.value()) {
        const Time timestep = si::seconds(
            std::min(options.timestep.value(), options.duration.value() - time.value()));
        const auto derivative = [&model, &controller](const PlanarState& stage_state,
                                                      Time stage_time) {
            return model.derivative(stage_state, clamp_input(controller(stage_time, stage_state)));
        };
        state = integrate(options.integrator, state, time, timestep, derivative);
        time += timestep;
        ++step;
        samples.push_back(sample(model, state, time, step, clamp_input(controller(time, state))));
    }
    return samples;
}

} // namespace apexlab

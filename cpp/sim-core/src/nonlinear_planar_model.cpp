#include "apexlab/nonlinear_planar_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace apexlab {
namespace {

constexpr double slip_denominator_floor_mps = 0.5;
constexpr double full_lateral_force_speed_mps = 1.0;

[[nodiscard]] constexpr std::size_t index(WheelIndex wheel) {
    return static_cast<std::size_t>(wheel);
}

[[nodiscard]] double smoothstep(double value) {
    const double clamped = std::clamp(value, 0.0, 1.0);
    return clamped * clamped * (3.0 - 2.0 * clamped);
}

[[nodiscard]] double regularized_velocity(double value) {
    return std::copysign(std::max(std::abs(value), slip_denominator_floor_mps),
                         value == 0.0 ? 1.0 : value);
}

struct ContactGeometry {
    double x_m;
    double y_m;
    double steering_rad;
    bool front;
};

} // namespace

NonlinearPlanarVehicleModel::NonlinearPlanarVehicleModel(VehicleParameters parameters)
    : parameters_(std::move(parameters)), longitudinal_model_(parameters_),
      front_tire_(parameters_.planar ? parameters_.planar->front_cornering_stiffness_n_per_rad / 2.0
                                     : 1.0,
                  parameters_.nonlinear_planar ? parameters_.nonlinear_planar->tire_mu_reference
                                               : 1.0,
                  parameters_.nonlinear_planar ? parameters_.nonlinear_planar->tire_reference_load
                                               : si::newtons(1.0),
                  parameters_.nonlinear_planar
                      ? parameters_.nonlinear_planar->tire_load_sensitivity_exponent
                      : 0.0),
      rear_tire_(parameters_.planar ? parameters_.planar->rear_cornering_stiffness_n_per_rad / 2.0
                                    : 1.0,
                 parameters_.nonlinear_planar ? parameters_.nonlinear_planar->tire_mu_reference
                                              : 1.0,
                 parameters_.nonlinear_planar ? parameters_.nonlinear_planar->tire_reference_load
                                              : si::newtons(1.0),
                 parameters_.nonlinear_planar
                     ? parameters_.nonlinear_planar->tire_load_sensitivity_exponent
                     : 0.0) {
    validate(parameters_);
    if (parameters_.schema_version != nonlinear_vehicle_schema_version || !parameters_.planar ||
        !parameters_.nonlinear_planar) {
        throw std::invalid_argument("nonlinear planar model requires vehicle schema version 3");
    }
}

NormalLoadState
NonlinearPlanarVehicleModel::normal_loads(Acceleration longitudinal_acceleration,
                                          Acceleration lateral_acceleration) const {
    const auto& planar = *parameters_.planar;
    const auto& nonlinear = *parameters_.nonlinear_planar;
    const double mass = parameters_.mass.value();
    const double gravity_load = mass * standard_gravity_mps2;
    const double wheelbase = planar.wheelbase.value();
    const double front_static = gravity_load * planar.cg_to_rear_axle.value() / wheelbase;
    const double rear_static = gravity_load * planar.cg_to_front_axle.value() / wheelbase;
    const double longitudinal_transfer = mass * longitudinal_acceleration.value() *
                                         nonlinear.cg_height.value() / wheelbase;
    const double front_axle = front_static - longitudinal_transfer;
    const double rear_axle = rear_static + longitudinal_transfer;
    const double roll_moment = mass * lateral_acceleration.value() * nonlinear.cg_height.value();
    const double front_lateral_transfer =
        nonlinear.front_roll_moment_fraction * roll_moment / nonlinear.front_track.value();
    const double rear_lateral_transfer =
        (1.0 - nonlinear.front_roll_moment_fraction) * roll_moment /
        nonlinear.rear_track.value();

    NormalLoadState result;
    result.wheel[index(WheelIndex::front_left)] =
        si::newtons(0.5 * front_axle - front_lateral_transfer);
    result.wheel[index(WheelIndex::front_right)] =
        si::newtons(0.5 * front_axle + front_lateral_transfer);
    result.wheel[index(WheelIndex::rear_left)] =
        si::newtons(0.5 * rear_axle - rear_lateral_transfer);
    result.wheel[index(WheelIndex::rear_right)] =
        si::newtons(0.5 * rear_axle + rear_lateral_transfer);
    result.longitudinal_transfer = si::newtons(longitudinal_transfer);
    result.front_lateral_transfer = si::newtons(front_lateral_transfer);
    result.rear_lateral_transfer = si::newtons(rear_lateral_transfer);
    for (const Force load : result.wheel) {
        if (!std::isfinite(load.value()) || load.value() < 0.0) {
            throw std::domain_error(
                "quasi-static load transfer produced negative wheel load; operating envelope exceeded");
        }
    }
    return result;
}

NonlinearPlanarForces NonlinearPlanarVehicleModel::forces(const PlanarState& state,
                                                           PlanarControl input) const {
    return forces_impl(state, input, nullptr);
}

NonlinearPlanarForces NonlinearPlanarVehicleModel::forces_with_loads(
    const PlanarState& state, PlanarControl input, const NormalLoadState& loads) const {
    for (const auto load : loads.wheel)
        if (!std::isfinite(load.value()) || load.value() < 0)
            throw std::domain_error("supplied contact load must be finite and nonnegative");
    return forces_impl(state, input, &loads);
}

NonlinearPlanarForces NonlinearPlanarVehicleModel::forces_impl(
    const PlanarState& state, PlanarControl input, const NormalLoadState* supplied) const {
    if (!std::isfinite(input.steering_angle.value())) {
        throw std::invalid_argument("steering angle must be finite");
    }
    input.throttle = std::clamp(input.throttle, 0.0, 1.0);
    input.brake = std::clamp(input.brake, 0.0, 1.0);
    const auto& planar = *parameters_.planar;
    const auto& nonlinear = *parameters_.nonlinear_planar;
    const double steering = input.steering_angle.value();
    const std::array<ContactGeometry, 4> geometry{{
        {planar.cg_to_front_axle.value(), nonlinear.front_track.value() / 2.0, steering, true},
        {planar.cg_to_front_axle.value(), -nonlinear.front_track.value() / 2.0, steering, true},
        {-planar.cg_to_rear_axle.value(), nonlinear.rear_track.value() / 2.0, 0.0, false},
        {-planar.cg_to_rear_axle.value(), -nonlinear.rear_track.value() / 2.0, 0.0, false},
    }};

    const double requested_drive = input.throttle * parameters_.powertrain.maximum_drive_force.value();
    const double requested_brake = state.velocity_x.value() > 0.0
                                       ? -input.brake * parameters_.brakes.maximum_brake_force.value()
                                       : 0.0;
    const double front_request = 0.5 *
                                 (requested_drive * nonlinear.drive_front_fraction +
                                  requested_brake * nonlinear.front_brake_bias);
    const double rear_request = 0.5 *
                                (requested_drive * (1.0 - nonlinear.drive_front_fraction) +
                                 requested_brake * (1.0 - nonlinear.front_brake_bias));
    const std::array<double, 4> requested_fx{{front_request, front_request, rear_request,
                                              rear_request}};

    const LongitudinalForces resistance = longitudinal_model_.forces(
        {si::meters(0.0), state.velocity_x}, DriverInput{});
    const double speed_scale = smoothstep(std::abs(state.velocity_x.value()) /
                                          full_lateral_force_speed_mps);
    double acceleration_x = 0.0;
    double acceleration_y = 0.0;
    NonlinearPlanarForces result;
    for (int iteration = 1; iteration <= nonlinear.force_iteration_max_iterations; ++iteration) {
        const NormalLoadState loads =
            supplied ? *supplied : normal_loads(si::meters_per_second_squared(acceleration_x),
                         si::meters_per_second_squared(acceleration_y));
        NonlinearPlanarForces candidate;
        candidate.longitudinal_load_transfer = loads.longitudinal_transfer;
        candidate.front_lateral_load_transfer = loads.front_lateral_transfer;
        candidate.rear_lateral_load_transfer = loads.rear_lateral_transfer;
        double body_x = resistance.aerodynamic_drag.value() +
                        resistance.rolling_resistance.value();
        double body_y = 0.0;
        double yaw_moment = 0.0;
        double delivered_wheel_x = 0.0;
        double total_capacity = 0.0;
        for (std::size_t wheel = 0; wheel < geometry.size(); ++wheel) {
            const ContactGeometry& contact = geometry[wheel];
            const double contact_vx =
                state.velocity_x.value() - state.yaw_rate.value() * contact.y_m;
            const double contact_vy =
                state.velocity_y.value() + state.yaw_rate.value() * contact.x_m;
            const double cosine = std::cos(contact.steering_rad);
            const double sine = std::sin(contact.steering_rad);
            const double wheel_vx = cosine * contact_vx + sine * contact_vy;
            const double wheel_vy = -sine * contact_vx + cosine * contact_vy;
            const Angle slip =
                si::radians(-std::atan2(wheel_vy, regularized_velocity(wheel_vx)));
            const NonlinearTireModel& tire = contact.front ? front_tire_ : rear_tire_;
            const TireContactOutput tire_force = tire.evaluate(
                {loads.wheel[wheel], slip * speed_scale, si::newtons(requested_fx[wheel])});
            const double wheel_body_x = cosine * tire_force.longitudinal_force.value() -
                                        sine * tire_force.lateral_force.value();
            const double wheel_body_y = sine * tire_force.longitudinal_force.value() +
                                        cosine * tire_force.lateral_force.value();
            candidate.wheel[wheel] = {
                loads.wheel[wheel],
                slip,
                tire_force.requested_longitudinal_force,
                tire_force.requested_lateral_force,
                tire_force.longitudinal_force,
                tire_force.lateral_force,
                si::newtons(wheel_body_x),
                si::newtons(wheel_body_y),
                tire_force.force_capacity,
                tire_force.effective_friction_coefficient,
                tire_force.requested_friction_utilization,
                tire_force.friction_utilization,
                tire_force.saturated,
            };
            body_x += wheel_body_x;
            body_y += wheel_body_y;
            yaw_moment += contact.x_m * wheel_body_y - contact.y_m * wheel_body_x;
            delivered_wheel_x += tire_force.longitudinal_force.value();
            total_capacity += tire_force.force_capacity.value();
        }
        candidate.body_longitudinal_force = si::newtons(body_x);
        candidate.body_lateral_force = si::newtons(body_y);
        candidate.yaw_moment_nm = yaw_moment;
        candidate.longitudinal = resistance;
        candidate.longitudinal.drive = si::newtons(std::max(0.0, delivered_wheel_x));
        candidate.longitudinal.brake = si::newtons(std::min(0.0, delivered_wheel_x));
        candidate.longitudinal.traction_limit = si::newtons(total_capacity);
        const double next_x = body_x / parameters_.mass.value();
        const double next_y = body_y / parameters_.mass.value();
        const double residual = std::max(std::abs(next_x - acceleration_x),
                                         std::abs(next_y - acceleration_y));
        candidate.solver_iterations = iteration;
        candidate.solver_residual_mps2 = residual;
        if (supplied || residual <= nonlinear.force_iteration_tolerance_mps2) {
            if (supplied) candidate.solver_residual_mps2 = 0.0;
            candidate.solver_converged = true;
            return candidate;
        }
        const double relaxation = nonlinear.force_iteration_relaxation;
        acceleration_x += relaxation * (next_x - acceleration_x);
        acceleration_y += relaxation * (next_y - acceleration_y);
        result = candidate;
    }
    throw std::runtime_error("nonlinear force/load fixed-point iteration did not converge; residual=" +
                             std::to_string(result.solver_residual_mps2));
}

PlanarDerivative NonlinearPlanarVehicleModel::derivative(const PlanarState& state,
                                                          PlanarControl input) const {
    const NonlinearPlanarForces force_state = forces(state, input);
    const double mass = parameters_.mass.value();
    const double yaw = state.yaw.value();
    const double vx = state.velocity_x.value();
    const double vy = state.velocity_y.value();
    const double yaw_rate = state.yaw_rate.value();
    return {
        si::meters_per_second(vx * std::cos(yaw) - vy * std::sin(yaw)),
        si::meters_per_second(vx * std::sin(yaw) + vy * std::cos(yaw)),
        state.yaw_rate,
        si::meters_per_second_squared(force_state.body_longitudinal_force.value() / mass +
                                      yaw_rate * vy),
        si::meters_per_second_squared(force_state.body_lateral_force.value() / mass - yaw_rate * vx),
        si::radians_per_second_squared(force_state.yaw_moment_nm /
                                       parameters_.planar->yaw_moment_of_inertia.value()),
    };
}

} // namespace apexlab

#include "apexlab/planar_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace apexlab {
namespace {

constexpr double slip_denominator_floor_mps = 0.5;
constexpr double full_lateral_force_speed_mps = 1.0;

[[nodiscard]] double smoothstep(double value) {
    const double clamped = std::clamp(value, 0.0, 1.0);
    return clamped * clamped * (3.0 - 2.0 * clamped);
}

[[nodiscard]] double regularized_longitudinal_velocity(double velocity_x_mps) {
    const double magnitude = std::max(std::abs(velocity_x_mps), slip_denominator_floor_mps);
    return std::copysign(magnitude, velocity_x_mps == 0.0 ? 1.0 : velocity_x_mps);
}

} // namespace

PlanarVehicleModel::PlanarVehicleModel(VehicleParameters parameters,
                                       std::shared_ptr<const LateralTireModel> front_tire,
                                       std::shared_ptr<const LateralTireModel> rear_tire)
    : parameters_(std::move(parameters)), longitudinal_model_(parameters_) {
    validate(parameters_);
    if (!parameters_.planar) {
        throw std::invalid_argument("planar model requires vehicle schema version 2 parameters");
    }
    const PlanarParameters& planar = *parameters_.planar;
    front_tire_ =
        front_tire ? std::move(front_tire)
                   : std::make_shared<LinearTireModel>(planar.front_cornering_stiffness_n_per_rad);
    rear_tire_ = rear_tire
                     ? std::move(rear_tire)
                     : std::make_shared<LinearTireModel>(planar.rear_cornering_stiffness_n_per_rad);
}

PlanarForces PlanarVehicleModel::forces(const PlanarState& state, PlanarControl input) const {
    if (!std::isfinite(input.steering_angle.value())) {
        throw std::invalid_argument("steering angle must be finite");
    }
    const PlanarParameters& planar = *parameters_.planar;
    const double mass = parameters_.mass.value();
    const double wheelbase = planar.wheelbase.value();
    const double front_distance = planar.cg_to_front_axle.value();
    const double rear_distance = planar.cg_to_rear_axle.value();
    const double vx = state.velocity_x.value();
    const double vy = state.velocity_y.value();
    const double yaw_rate = state.yaw_rate.value();
    const double steering = input.steering_angle.value();
    const double reference_vx = regularized_longitudinal_velocity(vx);

    const Angle front_slip =
        si::radians(steering - std::atan2(vy + front_distance * yaw_rate, reference_vx));
    const Angle rear_slip = si::radians(-std::atan2(vy - rear_distance * yaw_rate, reference_vx));
    const Force front_load = si::newtons(mass * standard_gravity_mps2 * rear_distance / wheelbase);
    const Force rear_load = si::newtons(mass * standard_gravity_mps2 * front_distance / wheelbase);
    const double lateral_force_scale = smoothstep(std::abs(vx) / full_lateral_force_speed_mps);
    const Force front_lateral =
        front_tire_->lateral_force({front_slip, front_load}) * lateral_force_scale;
    const Force rear_lateral =
        rear_tire_->lateral_force({rear_slip, rear_load}) * lateral_force_scale;

    const DriverInput longitudinal_input{input.throttle, input.brake};
    const LongitudinalForces longitudinal =
        longitudinal_model_.forces({si::meters(0.0), state.velocity_x}, longitudinal_input);
    const double front_body_x = -front_lateral.value() * std::sin(steering);
    const double front_body_y = front_lateral.value() * std::cos(steering);
    const double body_x = longitudinal.net().value() + front_body_x;
    const double body_y = front_body_y + rear_lateral.value();
    const double yaw_moment = front_distance * front_body_y - rear_distance * rear_lateral.value();

    return {
        longitudinal,  front_load,   rear_load,           front_slip,          rear_slip,
        front_lateral, rear_lateral, si::newtons(body_x), si::newtons(body_y), yaw_moment,
    };
}

PlanarDerivative PlanarVehicleModel::derivative(const PlanarState& state,
                                                PlanarControl input) const {
    const PlanarForces force_state = forces(state, input);
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
        si::meters_per_second_squared(force_state.body_lateral_force.value() / mass -
                                      yaw_rate * vx),
        si::radians_per_second_squared(force_state.yaw_moment_nm /
                                       parameters_.planar->yaw_moment_of_inertia.value()),
    };
}

} // namespace apexlab

#include "apexlab/longitudinal_model.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace apexlab {
namespace {

constexpr double velocity_epsilon_mps = 1.0e-12;

[[nodiscard]] double sign_of_motion(double velocity_mps) {
    if (velocity_mps > velocity_epsilon_mps) {
        return 1.0;
    }
    if (velocity_mps < -velocity_epsilon_mps) {
        return -1.0;
    }
    return 0.0;
}

} // namespace

Force LongitudinalForces::net() const {
    return drive + brake + aerodynamic_drag + rolling_resistance;
}

LongitudinalModel::LongitudinalModel(VehicleParameters parameters)
    : parameters_(std::move(parameters)) {
    validate(parameters_);
}

LongitudinalForces LongitudinalModel::forces(const LongitudinalState& state,
                                             DriverInput input) const {
    return forces_with_direction(state, input, sign_of_motion(state.velocity.value()));
}

LongitudinalForces LongitudinalModel::forces_with_direction(const LongitudinalState& state,
                                                            DriverInput input,
                                                            double motion_direction) const {
    input.throttle = std::clamp(input.throttle, 0.0, 1.0);
    input.brake = std::clamp(input.brake, 0.0, 1.0);

    const double velocity = state.velocity.value();
    const double speed = std::abs(velocity);
    const double direction = motion_direction;
    const auto& aero = parameters_.aero;
    const double dynamic_pressure = 0.5 * aero.air_density_kgpm3 * speed * speed;
    const double downforce = dynamic_pressure * aero.lift_coefficient_down * aero.reference_area_m2;
    const double total_normal_load = parameters_.mass.value() * standard_gravity_mps2 + downforce;
    const double tire_limit = parameters_.tires.friction_coefficient * total_normal_load;
    const double drive_limit =
        tire_limit * parameters_.powertrain.driven_wheel_static_load_fraction;
    const double requested_drive =
        input.throttle * parameters_.powertrain.maximum_drive_force.value();
    const double drive = std::min(requested_drive, drive_limit);

    const double requested_brake = input.brake * parameters_.brakes.maximum_brake_force.value();
    const double brake =
        direction == 0.0 ? 0.0 : -direction * std::min(requested_brake, tire_limit);
    const double drag =
        -direction * dynamic_pressure * aero.drag_coefficient * aero.reference_area_m2;
    const double rolling = -direction * parameters_.tires.rolling_resistance_coefficient *
                           parameters_.mass.value() * standard_gravity_mps2;

    return {
        si::newtons(drive),   si::newtons(brake),     si::newtons(drag),
        si::newtons(rolling), si::newtons(downforce), si::newtons(tire_limit),
    };
}

LongitudinalDerivative LongitudinalModel::derivative(const LongitudinalState& state,
                                                     DriverInput input) const {
    const LongitudinalForces force_components = forces(state, input);
    return {
        state.velocity,
        si::meters_per_second_squared(force_components.net().value() / parameters_.mass.value()),
    };
}

LongitudinalDerivative
LongitudinalModel::derivative_continuing_motion(const LongitudinalState& state, DriverInput input,
                                                double motion_direction) const {
    const LongitudinalForces force_components =
        forces_with_direction(state, input, motion_direction);
    return {
        state.velocity,
        si::meters_per_second_squared(force_components.net().value() / parameters_.mass.value()),
    };
}

} // namespace apexlab

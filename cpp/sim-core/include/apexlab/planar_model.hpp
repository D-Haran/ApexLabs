#pragma once

#include <memory>

#include "apexlab/longitudinal_model.hpp"
#include "apexlab/tire_model.hpp"

namespace apexlab {

struct PlanarState {
    Distance position_x{si::meters(0.0)};
    Distance position_y{si::meters(0.0)};
    Angle yaw{si::radians(0.0)};
    Velocity velocity_x{si::meters_per_second(0.0)};
    Velocity velocity_y{si::meters_per_second(0.0)};
    AngularVelocity yaw_rate{si::radians_per_second(0.0)};
};

struct PlanarDerivative {
    Velocity position_x_rate{si::meters_per_second(0.0)};
    Velocity position_y_rate{si::meters_per_second(0.0)};
    AngularVelocity yaw_rate{si::radians_per_second(0.0)};
    Acceleration velocity_x_rate{si::meters_per_second_squared(0.0)};
    Acceleration velocity_y_rate{si::meters_per_second_squared(0.0)};
    AngularAcceleration yaw_acceleration{si::radians_per_second_squared(0.0)};
};

[[nodiscard]] constexpr PlanarDerivative operator+(PlanarDerivative lhs, PlanarDerivative rhs) {
    return {
        lhs.position_x_rate + rhs.position_x_rate,
        lhs.position_y_rate + rhs.position_y_rate,
        lhs.yaw_rate + rhs.yaw_rate,
        lhs.velocity_x_rate + rhs.velocity_x_rate,
        lhs.velocity_y_rate + rhs.velocity_y_rate,
        lhs.yaw_acceleration + rhs.yaw_acceleration,
    };
}

[[nodiscard]] constexpr PlanarDerivative operator*(PlanarDerivative derivative, double scalar) {
    return {
        derivative.position_x_rate * scalar, derivative.position_y_rate * scalar,
        derivative.yaw_rate * scalar,        derivative.velocity_x_rate * scalar,
        derivative.velocity_y_rate * scalar, derivative.yaw_acceleration * scalar,
    };
}

[[nodiscard]] constexpr PlanarState advance(PlanarState state, PlanarDerivative derivative,
                                            Time timestep) {
    const double dt = timestep.value();
    return {
        state.position_x + Distance{derivative.position_x_rate.value() * dt},
        state.position_y + Distance{derivative.position_y_rate.value() * dt},
        state.yaw + Angle{derivative.yaw_rate.value() * dt},
        state.velocity_x + Velocity{derivative.velocity_x_rate.value() * dt},
        state.velocity_y + Velocity{derivative.velocity_y_rate.value() * dt},
        state.yaw_rate + AngularVelocity{derivative.yaw_acceleration.value() * dt},
    };
}

struct PlanarControl {
    Angle steering_angle{si::radians(0.0)};
    double throttle{0.0};
    double brake{0.0};
};

struct PlanarForces {
    LongitudinalForces longitudinal;
    Force front_normal_load{si::newtons(0.0)};
    Force rear_normal_load{si::newtons(0.0)};
    Angle front_slip_angle{si::radians(0.0)};
    Angle rear_slip_angle{si::radians(0.0)};
    Force front_lateral_force{si::newtons(0.0)};
    Force rear_lateral_force{si::newtons(0.0)};
    Force body_longitudinal_force{si::newtons(0.0)};
    Force body_lateral_force{si::newtons(0.0)};
    double yaw_moment_nm{0.0};
};

class PlanarVehicleModel {
  public:
    explicit PlanarVehicleModel(VehicleParameters parameters,
                                std::shared_ptr<const LateralTireModel> front_tire = nullptr,
                                std::shared_ptr<const LateralTireModel> rear_tire = nullptr);

    [[nodiscard]] const VehicleParameters& parameters() const { return parameters_; }
    [[nodiscard]] PlanarForces forces(const PlanarState& state, PlanarControl input) const;
    [[nodiscard]] PlanarDerivative derivative(const PlanarState& state, PlanarControl input) const;

  private:
    VehicleParameters parameters_;
    LongitudinalModel longitudinal_model_;
    std::shared_ptr<const LateralTireModel> front_tire_;
    std::shared_ptr<const LateralTireModel> rear_tire_;
};

} // namespace apexlab

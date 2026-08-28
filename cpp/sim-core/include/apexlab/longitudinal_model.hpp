#pragma once

#include "apexlab/units.hpp"
#include "apexlab/vehicle_parameters.hpp"

namespace apexlab {

struct LongitudinalState {
    Distance position{si::meters(0.0)};
    Velocity velocity{si::meters_per_second(0.0)};
};

struct LongitudinalDerivative {
    Velocity position_rate{si::meters_per_second(0.0)};
    Acceleration velocity_rate{si::meters_per_second_squared(0.0)};
};

[[nodiscard]] constexpr LongitudinalDerivative operator+(LongitudinalDerivative lhs,
                                                         LongitudinalDerivative rhs) {
    return {lhs.position_rate + rhs.position_rate, lhs.velocity_rate + rhs.velocity_rate};
}

[[nodiscard]] constexpr LongitudinalDerivative operator*(LongitudinalDerivative derivative,
                                                         double scalar) {
    return {derivative.position_rate * scalar, derivative.velocity_rate * scalar};
}

[[nodiscard]] constexpr LongitudinalState
advance(LongitudinalState state, LongitudinalDerivative derivative, Time timestep) {
    return {
        state.position + Distance{derivative.position_rate.value() * timestep.value()},
        state.velocity + Velocity{derivative.velocity_rate.value() * timestep.value()},
    };
}

struct DriverInput {
    double throttle{0.0};
    double brake{0.0};
};

struct LongitudinalForces {
    Force drive{si::newtons(0.0)};
    Force brake{si::newtons(0.0)};
    Force aerodynamic_drag{si::newtons(0.0)};
    Force rolling_resistance{si::newtons(0.0)};
    Force aerodynamic_downforce{si::newtons(0.0)};
    Force traction_limit{si::newtons(0.0)};

    [[nodiscard]] Force net() const;
};

class LongitudinalModel {
  public:
    explicit LongitudinalModel(VehicleParameters parameters);

    [[nodiscard]] const VehicleParameters& parameters() const { return parameters_; }
    [[nodiscard]] LongitudinalForces forces(const LongitudinalState& state,
                                            DriverInput input) const;
    [[nodiscard]] LongitudinalDerivative derivative(const LongitudinalState& state,
                                                    DriverInput input) const;
    [[nodiscard]] LongitudinalDerivative
    derivative_continuing_motion(const LongitudinalState& state, DriverInput input,
                                 double motion_direction) const;

  private:
    [[nodiscard]] LongitudinalForces forces_with_direction(const LongitudinalState& state,
                                                           DriverInput input,
                                                           double motion_direction) const;
    VehicleParameters parameters_;
};

} // namespace apexlab

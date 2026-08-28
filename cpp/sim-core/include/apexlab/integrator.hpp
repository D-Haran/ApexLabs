#pragma once

#include <stdexcept>

#include "apexlab/units.hpp"

namespace apexlab {

enum class IntegratorKind { euler, rk4 };

[[nodiscard]] inline const char* to_string(IntegratorKind kind) {
    switch (kind) {
    case IntegratorKind::euler:
        return "euler";
    case IntegratorKind::rk4:
        return "rk4";
    }
    throw std::logic_error("unknown integrator kind");
}

template <typename State, typename DerivativeFunction>
[[nodiscard]] State integrate_euler(const State& state, Time time, Time timestep,
                                    DerivativeFunction&& derivative) {
    return advance(state, derivative(state, time), timestep);
}

template <typename State, typename DerivativeFunction>
[[nodiscard]] State integrate_rk4(const State& state, Time time, Time timestep,
                                  DerivativeFunction&& derivative) {
    const double dt = timestep.value();
    const auto k1 = derivative(state, time);
    const auto k2 = derivative(advance(state, k1, Time{0.5 * dt}), time + Time{0.5 * dt});
    const auto k3 = derivative(advance(state, k2, Time{0.5 * dt}), time + Time{0.5 * dt});
    const auto k4 = derivative(advance(state, k3, timestep), time + timestep);
    const auto weighted_derivative = (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (1.0 / 6.0);
    return advance(state, weighted_derivative, timestep);
}

template <typename State, typename DerivativeFunction>
[[nodiscard]] State integrate(IntegratorKind kind, const State& state, Time time, Time timestep,
                              DerivativeFunction&& derivative) {
    switch (kind) {
    case IntegratorKind::euler:
        return integrate_euler(state, time, timestep, derivative);
    case IntegratorKind::rk4:
        return integrate_rk4(state, time, timestep, derivative);
    }
    throw std::logic_error("unknown integrator kind");
}

} // namespace apexlab

#pragma once

#include <functional>
#include <vector>

#include "apexlab/integrator.hpp"
#include "apexlab/telemetry.hpp"

namespace apexlab {

struct SimulationOptions {
    Time duration{si::seconds(10.0)};
    Time timestep{si::seconds(0.005)};
    IntegratorKind integrator{IntegratorKind::rk4};
    bool stop_when_stationary{false};
};

using DriverController = std::function<DriverInput(Time, const LongitudinalState&)>;
using PlanarController = std::function<PlanarControl(Time, const PlanarState&)>;

[[nodiscard]] std::vector<TelemetrySample> simulate(const LongitudinalModel& model,
                                                    LongitudinalState initial_state,
                                                    const SimulationOptions& options,
                                                    const DriverController& controller);

[[nodiscard]] std::vector<PlanarTelemetrySample>
simulate_planar(const PlanarVehicleModel& model, PlanarState initial_state,
                const SimulationOptions& options, const PlanarController& controller);

[[nodiscard]] std::vector<NonlinearPlanarTelemetrySample>
simulate_nonlinear_planar(const NonlinearPlanarVehicleModel& model, PlanarState initial_state,
                          const SimulationOptions& options, const PlanarController& controller);

} // namespace apexlab

#pragma once

#include <array>

#include "apexlab/planar_model.hpp"

namespace apexlab {

enum class WheelIndex : std::size_t { front_left = 0, front_right = 1, rear_left = 2, rear_right = 3 };

struct WheelContactForces {
    Force normal_load{si::newtons(0.0)};
    Angle slip_angle{si::radians(0.0)};
    Force requested_longitudinal_force{si::newtons(0.0)};
    Force requested_lateral_force{si::newtons(0.0)};
    Force longitudinal_force{si::newtons(0.0)};
    Force lateral_force{si::newtons(0.0)};
    Force body_longitudinal_force{si::newtons(0.0)};
    Force body_lateral_force{si::newtons(0.0)};
    Force force_capacity{si::newtons(0.0)};
    double effective_friction_coefficient{0.0};
    double requested_friction_utilization{0.0};
    double friction_utilization{0.0};
    bool saturated{false};
};

struct NormalLoadState {
    std::array<Force, 4> wheel{};
    Force longitudinal_transfer{si::newtons(0.0)};
    Force front_lateral_transfer{si::newtons(0.0)};
    Force rear_lateral_transfer{si::newtons(0.0)};
};

struct NonlinearPlanarForces {
    std::array<WheelContactForces, 4> wheel{};
    LongitudinalForces longitudinal;
    Force body_longitudinal_force{si::newtons(0.0)};
    Force body_lateral_force{si::newtons(0.0)};
    double yaw_moment_nm{0.0};
    Force longitudinal_load_transfer{si::newtons(0.0)};
    Force front_lateral_load_transfer{si::newtons(0.0)};
    Force rear_lateral_load_transfer{si::newtons(0.0)};
    int solver_iterations{0};
    double solver_residual_mps2{0.0};
    bool solver_converged{false};
};

class NonlinearPlanarVehicleModel {
  public:
    explicit NonlinearPlanarVehicleModel(VehicleParameters parameters);

    [[nodiscard]] const VehicleParameters& parameters() const { return parameters_; }
    [[nodiscard]] NormalLoadState normal_loads(Acceleration longitudinal_acceleration,
                                                Acceleration lateral_acceleration) const;
    [[nodiscard]] NonlinearPlanarForces forces(const PlanarState& state,
                                                PlanarControl input) const;
    [[nodiscard]] PlanarDerivative derivative(const PlanarState& state,
                                               PlanarControl input) const;

    // Additive contact-force adapter: supplied loads bypass quasi-static transfer.
    [[nodiscard]] NonlinearPlanarForces forces_with_loads(const PlanarState& state,
        PlanarControl input, const NormalLoadState& loads) const;

  private:
    [[nodiscard]] NonlinearPlanarForces forces_impl(const PlanarState& state,
        PlanarControl input, const NormalLoadState* supplied) const;
    VehicleParameters parameters_;
    LongitudinalModel longitudinal_model_;
    NonlinearTireModel front_tire_;
    NonlinearTireModel rear_tire_;
};

} // namespace apexlab

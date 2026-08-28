#pragma once

#include "apexlab/units.hpp"

namespace apexlab {

struct AxleTireInput {
    Angle slip_angle{si::radians(0.0)};
    Force normal_load{si::newtons(0.0)};
};

class LateralTireModel {
  public:
    virtual ~LateralTireModel() = default;
    [[nodiscard]] virtual Force lateral_force(const AxleTireInput& input) const = 0;
};

class LinearTireModel final : public LateralTireModel {
  public:
    explicit LinearTireModel(double cornering_stiffness_n_per_rad);

    [[nodiscard]] double cornering_stiffness_n_per_rad() const {
        return cornering_stiffness_n_per_rad_;
    }
    [[nodiscard]] Force lateral_force(const AxleTireInput& input) const override;

  private:
    double cornering_stiffness_n_per_rad_;
};

struct TireContactInput {
    Force normal_load{si::newtons(0.0)};
    Angle slip_angle{si::radians(0.0)};
    Force requested_longitudinal_force{si::newtons(0.0)};
};

struct TireContactOutput {
    Force requested_longitudinal_force{si::newtons(0.0)};
    Force requested_lateral_force{si::newtons(0.0)};
    Force longitudinal_force{si::newtons(0.0)};
    Force lateral_force{si::newtons(0.0)};
    Force force_capacity{si::newtons(0.0)};
    double effective_friction_coefficient{0.0};
    double requested_friction_utilization{0.0};
    double friction_utilization{0.0};
    bool saturated{false};
};

class NonlinearTireModel {
  public:
    NonlinearTireModel(double cornering_stiffness_n_per_rad, double reference_mu,
                       Force reference_load, double load_sensitivity_exponent);

    [[nodiscard]] TireContactOutput evaluate(const TireContactInput& input) const;
    [[nodiscard]] double effective_mu(Force normal_load) const;
    [[nodiscard]] double cornering_stiffness_n_per_rad() const {
        return cornering_stiffness_n_per_rad_;
    }

  private:
    double cornering_stiffness_n_per_rad_;
    double reference_mu_;
    Force reference_load_;
    double load_sensitivity_exponent_;
};

} // namespace apexlab

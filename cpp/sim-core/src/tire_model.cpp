#include "apexlab/tire_model.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace apexlab {

LinearTireModel::LinearTireModel(double cornering_stiffness_n_per_rad)
    : cornering_stiffness_n_per_rad_(cornering_stiffness_n_per_rad) {
    if (!std::isfinite(cornering_stiffness_n_per_rad_) || cornering_stiffness_n_per_rad_ <= 0.0) {
        throw std::invalid_argument("cornering stiffness must be finite and positive");
    }
}

Force LinearTireModel::lateral_force(const AxleTireInput& input) const {
    if (!std::isfinite(input.slip_angle.value()) || !std::isfinite(input.normal_load.value()) ||
        input.normal_load.value() < 0.0) {
        throw std::invalid_argument("tire input must contain finite slip and nonnegative load");
    }
    // Radians are dimensionless in SI. Normal load is accepted now so later tire models can use it.
    return si::newtons(cornering_stiffness_n_per_rad_ * input.slip_angle.value());
}

NonlinearTireModel::NonlinearTireModel(double cornering_stiffness_n_per_rad, double reference_mu,
                                       Force reference_load,
                                       double load_sensitivity_exponent)
    : cornering_stiffness_n_per_rad_(cornering_stiffness_n_per_rad),
      reference_mu_(reference_mu), reference_load_(reference_load),
      load_sensitivity_exponent_(load_sensitivity_exponent) {
    if (!std::isfinite(cornering_stiffness_n_per_rad_) ||
        cornering_stiffness_n_per_rad_ <= 0.0) {
        throw std::invalid_argument("cornering stiffness must be finite and positive");
    }
    if (!std::isfinite(reference_mu_) || reference_mu_ <= 0.0 ||
        !std::isfinite(reference_load_.value()) || reference_load_.value() <= 0.0) {
        throw std::invalid_argument("reference tire friction and load must be finite and positive");
    }
    if (!std::isfinite(load_sensitivity_exponent_) || load_sensitivity_exponent_ > 0.0 ||
        load_sensitivity_exponent_ < -0.5) {
        throw std::invalid_argument("load-sensitivity exponent must be in [-0.5, 0]");
    }
}

double NonlinearTireModel::effective_mu(Force normal_load) const {
    if (!std::isfinite(normal_load.value()) || normal_load.value() <= 0.0) {
        throw std::invalid_argument("effective_mu requires a finite positive normal load");
    }
    return reference_mu_ *
           std::pow(normal_load.value() / reference_load_.value(), load_sensitivity_exponent_);
}

TireContactOutput NonlinearTireModel::evaluate(const TireContactInput& input) const {
    const double fz = input.normal_load.value();
    const double alpha = input.slip_angle.value();
    const double requested_fx = input.requested_longitudinal_force.value();
    if (!std::isfinite(fz) || fz < 0.0 || !std::isfinite(alpha) ||
        !std::isfinite(requested_fx)) {
        throw std::invalid_argument("nonlinear tire input must be finite with nonnegative load");
    }
    if (fz == 0.0) {
        return {input.requested_longitudinal_force, si::newtons(0.0), si::newtons(0.0),
                si::newtons(0.0), si::newtons(0.0), 0.0,
                requested_fx == 0.0 ? 0.0 : std::numeric_limits<double>::infinity(), 0.0,
                requested_fx != 0.0 || alpha != 0.0};
    }

    const double mu = effective_mu(input.normal_load);
    const double capacity = mu * fz;
    const double tangent = std::tan(alpha);
    const double absolute_tangent = std::abs(tangent);
    const double saturation_tangent = 3.0 * capacity / cornering_stiffness_n_per_rad_;
    double requested_fy = 0.0;
    if (absolute_tangent < saturation_tangent) {
        const double stiffness = cornering_stiffness_n_per_rad_;
        requested_fy = stiffness * tangent -
                       stiffness * stiffness * absolute_tangent * tangent / (3.0 * capacity) +
                       stiffness * stiffness * stiffness * tangent * tangent * tangent /
                           (27.0 * capacity * capacity);
    } else {
        requested_fy = std::copysign(capacity, tangent);
    }

    const double requested_magnitude = std::hypot(requested_fx, requested_fy);
    const double requested_utilization = requested_magnitude / capacity;
    const double scale = requested_utilization > 1.0 ? 1.0 / requested_utilization : 1.0;
    const double fx = requested_fx * scale;
    const double fy = requested_fy * scale;
    return {
        input.requested_longitudinal_force,
        si::newtons(requested_fy),
        si::newtons(fx),
        si::newtons(fy),
        si::newtons(capacity),
        mu,
        requested_utilization,
        std::hypot(fx, fy) / capacity,
        requested_utilization > 1.0 || absolute_tangent >= saturation_tangent,
    };
}

} // namespace apexlab

#include "apexlab/vehicle_parameters.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace apexlab {
namespace {

[[nodiscard]] std::string read_file(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream) {
        throw std::runtime_error("cannot open vehicle configuration: " + path.string());
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    return buffer.str();
}

[[nodiscard]] double required_number(const std::string& document, const std::string& key) {
    const std::regex expression{
        "\"" + key + "\"\\s*:\\s*(-?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][+-]?[0-9]+)?)"};
    std::smatch match;
    if (!std::regex_search(document, match, expression)) {
        throw std::runtime_error("missing or invalid numeric vehicle field: " + key);
    }
    return std::stod(match[1].str());
}

[[nodiscard]] std::string required_string(const std::string& document, const std::string& key) {
    const std::regex expression{"\"" + key + "\"\\s*:\\s*\"([^\"]*)\""};
    std::smatch match;
    if (!std::regex_search(document, match, expression)) {
        throw std::runtime_error("missing or invalid string vehicle field: " + key);
    }
    return match[1].str();
}

[[nodiscard]] int required_integer(const std::string& document, const std::string& key) {
    const double value = required_number(document, key);
    if (std::floor(value) != value || value < static_cast<double>(std::numeric_limits<int>::min()) ||
        value > static_cast<double>(std::numeric_limits<int>::max())) {
        throw std::runtime_error("vehicle field must be an integer: " + key);
    }
    return static_cast<int>(value);
}

void require_finite_nonnegative(double value, const char* name) {
    if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(std::string{name} + " must be finite and nonnegative");
    }
}

} // namespace

void validate(const VehicleParameters& parameters) {
    if (parameters.schema_version != vehicle_schema_version &&
        parameters.schema_version != planar_vehicle_schema_version &&
        parameters.schema_version != nonlinear_vehicle_schema_version) {
        throw std::invalid_argument("unsupported vehicle schema version");
    }
    if (parameters.normal_load_model != "quasi_static" && parameters.normal_load_model != "sprung_body")
        throw std::invalid_argument("unknown normal_load_model");
    if (parameters.normal_load_model == "sprung_body" && (!parameters.sprung_body || !parameters.nonlinear_planar))
        throw std::invalid_argument("sprung_body mode requires suspension and nonlinear vehicle parameters");
    if (parameters.sprung_body) {
        const auto& b = *parameters.sprung_body;
        if (!std::isfinite(b.roll_inertia_kg_m2) || b.roll_inertia_kg_m2 <= 0 ||
            !std::isfinite(b.pitch_inertia_kg_m2) || b.pitch_inertia_kg_m2 <= 0)
            throw std::invalid_argument("sprung body inertias must be positive");
        for (const auto& c : b.corners) {
            if (!std::isfinite(c.spring_rate_n_m) || c.spring_rate_n_m <= 0 ||
                !std::isfinite(c.nominal_ride_height_m) || c.nominal_ride_height_m <= 0)
                throw std::invalid_argument("spring rate and nominal ride height must be positive");
            require_finite_nonnegative(c.compression_damping_ns_m, "compression damping");
            require_finite_nonnegative(c.rebound_damping_ns_m, "rebound damping");
        }
    }
    if (parameters.name.empty()) {
        throw std::invalid_argument("vehicle name must not be empty");
    }
    if (parameters.provenance.empty()) {
        throw std::invalid_argument("vehicle provenance must not be empty");
    }
    if (!std::isfinite(parameters.mass.value()) || parameters.mass.value() <= 0.0) {
        throw std::invalid_argument("mass_kg must be finite and positive");
    }

    require_finite_nonnegative(parameters.aero.air_density_kgpm3, "air_density_kgpm3");
    require_finite_nonnegative(parameters.aero.drag_coefficient, "drag_coefficient");
    require_finite_nonnegative(parameters.aero.lift_coefficient_down, "lift_coefficient_down");
    require_finite_nonnegative(parameters.aero.reference_area_m2, "reference_area_m2");
    require_finite_nonnegative(parameters.powertrain.maximum_drive_force.value(),
                               "maximum_drive_force_n");
    require_finite_nonnegative(parameters.brakes.maximum_brake_force.value(),
                               "maximum_brake_force_n");
    require_finite_nonnegative(parameters.tires.friction_coefficient, "friction_coefficient");
    require_finite_nonnegative(parameters.tires.rolling_resistance_coefficient,
                               "rolling_resistance_coefficient");

    const double driven_fraction = parameters.powertrain.driven_wheel_static_load_fraction;
    if (!std::isfinite(driven_fraction) || driven_fraction <= 0.0 || driven_fraction > 1.0) {
        throw std::invalid_argument(
            "driven_wheel_static_load_fraction must be in the interval (0, 1]");
    }

    if (parameters.schema_version >= planar_vehicle_schema_version && !parameters.planar) {
        throw std::invalid_argument("planar vehicle schemas require planar parameters");
    }
    if (parameters.planar) {
        const PlanarParameters& planar = *parameters.planar;
        if (!std::isfinite(planar.wheelbase.value()) || planar.wheelbase.value() <= 0.0) {
            throw std::invalid_argument("wheelbase_m must be finite and positive");
        }
        if (!std::isfinite(planar.cg_to_front_axle.value()) ||
            planar.cg_to_front_axle.value() <= 0.0) {
            throw std::invalid_argument("cg_to_front_axle_m must be finite and positive");
        }
        if (!std::isfinite(planar.cg_to_rear_axle.value()) ||
            planar.cg_to_rear_axle.value() <= 0.0) {
            throw std::invalid_argument("cg_to_rear_axle_m must be finite and positive");
        }
        const double axle_sum = planar.cg_to_front_axle.value() + planar.cg_to_rear_axle.value();
        const double geometry_tolerance = 1.0e-9 * std::max(1.0, planar.wheelbase.value());
        if (std::abs(axle_sum - planar.wheelbase.value()) > geometry_tolerance) {
            throw std::invalid_argument(
                "cg_to_front_axle_m + cg_to_rear_axle_m must equal wheelbase_m");
        }
        if (!std::isfinite(planar.yaw_moment_of_inertia.value()) ||
            planar.yaw_moment_of_inertia.value() <= 0.0) {
            throw std::invalid_argument("yaw_moment_of_inertia_kg_m2 must be finite and positive");
        }
        if (!std::isfinite(planar.front_cornering_stiffness_n_per_rad) ||
            planar.front_cornering_stiffness_n_per_rad <= 0.0) {
            throw std::invalid_argument(
                "front_cornering_stiffness_n_per_rad must be finite and positive");
        }
        if (!std::isfinite(planar.rear_cornering_stiffness_n_per_rad) ||
            planar.rear_cornering_stiffness_n_per_rad <= 0.0) {
            throw std::invalid_argument(
                "rear_cornering_stiffness_n_per_rad must be finite and positive");
        }
    }
    if (parameters.schema_version == nonlinear_vehicle_schema_version &&
        !parameters.nonlinear_planar) {
        throw std::invalid_argument("vehicle schema version 3 requires nonlinear parameters");
    }
    if (parameters.nonlinear_planar) {
        const NonlinearPlanarParameters& nonlinear = *parameters.nonlinear_planar;
        if (!std::isfinite(nonlinear.front_track.value()) || nonlinear.front_track.value() <= 0.0 ||
            !std::isfinite(nonlinear.rear_track.value()) || nonlinear.rear_track.value() <= 0.0 ||
            !std::isfinite(nonlinear.cg_height.value()) || nonlinear.cg_height.value() < 0.0) {
            throw std::invalid_argument("track widths must be positive and CG height nonnegative");
        }
        const auto fraction_valid = [](double value) {
            return std::isfinite(value) && value >= 0.0 && value <= 1.0;
        };
        if (!fraction_valid(nonlinear.front_roll_moment_fraction) ||
            !fraction_valid(nonlinear.front_brake_bias) ||
            !fraction_valid(nonlinear.drive_front_fraction)) {
            throw std::invalid_argument("nonlinear distribution fractions must be in [0,1]");
        }
        if ((nonlinear.drivetrain_type == "FWD" && nonlinear.drive_front_fraction != 1.0) ||
            (nonlinear.drivetrain_type == "RWD" && nonlinear.drive_front_fraction != 0.0) ||
            (nonlinear.drivetrain_type == "AWD" &&
             (nonlinear.drive_front_fraction <= 0.0 || nonlinear.drive_front_fraction >= 1.0)) ||
            (nonlinear.drivetrain_type != "FWD" && nonlinear.drivetrain_type != "RWD" &&
             nonlinear.drivetrain_type != "AWD")) {
            throw std::invalid_argument("drivetrain_type and drive_front_fraction are inconsistent");
        }
        if (!std::isfinite(nonlinear.tire_mu_reference) || nonlinear.tire_mu_reference <= 0.0 ||
            !std::isfinite(nonlinear.tire_reference_load.value()) ||
            nonlinear.tire_reference_load.value() <= 0.0 ||
            !std::isfinite(nonlinear.tire_load_sensitivity_exponent) ||
            nonlinear.tire_load_sensitivity_exponent < -0.5 ||
            nonlinear.tire_load_sensitivity_exponent > 0.0) {
            throw std::invalid_argument("invalid nonlinear tire load-sensitivity parameters");
        }
        if (!std::isfinite(nonlinear.force_iteration_tolerance_mps2) ||
            nonlinear.force_iteration_tolerance_mps2 <= 0.0 ||
            nonlinear.force_iteration_max_iterations <= 0 ||
            !std::isfinite(nonlinear.force_iteration_relaxation) ||
            nonlinear.force_iteration_relaxation <= 0.0 ||
            nonlinear.force_iteration_relaxation > 1.0) {
            throw std::invalid_argument("invalid nonlinear force iteration controls");
        }
    }
}

VehicleParameters load_vehicle_parameters(const std::filesystem::path& path) {
    const std::string document = read_file(path);
    VehicleParameters parameters;
    const double version = required_number(document, "schema_version");
    if (std::floor(version) != version) {
        throw std::runtime_error("schema_version must be an integer");
    }
    parameters.schema_version = static_cast<int>(version);
    parameters.name = required_string(document, "name");
    parameters.provenance = required_string(document, "provenance");
    parameters.mass = si::kilograms(required_number(document, "mass_kg"));
    parameters.aero.air_density_kgpm3 = required_number(document, "air_density_kgpm3");
    parameters.aero.drag_coefficient = required_number(document, "drag_coefficient");
    parameters.aero.lift_coefficient_down = required_number(document, "lift_coefficient_down");
    parameters.aero.reference_area_m2 = required_number(document, "reference_area_m2");
    parameters.powertrain.maximum_drive_force =
        si::newtons(required_number(document, "maximum_drive_force_n"));
    parameters.powertrain.driven_wheel_static_load_fraction =
        required_number(document, "driven_wheel_static_load_fraction");
    parameters.brakes.maximum_brake_force =
        si::newtons(required_number(document, "maximum_brake_force_n"));
    parameters.tires.friction_coefficient =
        parameters.schema_version == nonlinear_vehicle_schema_version
            ? required_number(document, "tire_mu_reference")
            : required_number(document, "friction_coefficient");
    parameters.tires.rolling_resistance_coefficient =
        required_number(document, "rolling_resistance_coefficient");
    if (parameters.schema_version >= planar_vehicle_schema_version) {
        parameters.planar = PlanarParameters{
            si::meters(required_number(document, "wheelbase_m")),
            si::meters(required_number(document, "cg_to_front_axle_m")),
            si::meters(required_number(document, "cg_to_rear_axle_m")),
            si::kilogram_square_meters(required_number(document, "yaw_moment_of_inertia_kg_m2")),
            required_number(document, "front_cornering_stiffness_n_per_rad"),
            required_number(document, "rear_cornering_stiffness_n_per_rad"),
        };
    }
    if (parameters.schema_version == nonlinear_vehicle_schema_version) {
        parameters.nonlinear_planar = NonlinearPlanarParameters{
            si::meters(required_number(document, "front_track_m")),
            si::meters(required_number(document, "rear_track_m")),
            si::meters(required_number(document, "cg_height_m")),
            required_number(document, "front_roll_moment_fraction"),
            required_number(document, "front_brake_bias"),
            required_string(document, "drivetrain_type"),
            required_number(document, "drive_front_fraction"),
            required_number(document, "tire_mu_reference"),
            si::newtons(required_number(document, "tire_reference_load_n")),
            required_number(document, "tire_load_sensitivity_exponent"),
            required_number(document, "force_iteration_tolerance_mps2"),
            required_integer(document, "force_iteration_max_iterations"),
            required_number(document, "force_iteration_relaxation"),
        };
    }
    if (document.find("\"normal_load_model\"") != std::string::npos)
        parameters.normal_load_model = required_string(document, "normal_load_model");
    if (parameters.normal_load_model == "sprung_body") {
        SprungBodyParameters body;
        body.roll_inertia_kg_m2 = required_number(document, "roll_inertia_kg_m2");
        body.pitch_inertia_kg_m2 = required_number(document, "pitch_inertia_kg_m2");
        std::size_t index = 0;
        for (const std::string corner : {"fl", "fr", "rl", "rr"}) {
            auto& c = body.corners[index++];
            c.spring_rate_n_m = required_number(document, corner + "_spring_rate_n_m");
            c.compression_damping_ns_m = required_number(document, corner + "_compression_damping_ns_m");
            c.rebound_damping_ns_m = required_number(document, corner + "_rebound_damping_ns_m");
            c.nominal_ride_height_m = required_number(document, corner + "_nominal_ride_height_m");
        }
        parameters.sprung_body = body;
    }
    validate(parameters);
    return parameters;
}

} // namespace apexlab

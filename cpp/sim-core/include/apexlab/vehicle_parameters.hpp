#pragma once

#include <filesystem>
#include <array>
#include <optional>
#include <string>

#include "apexlab/units.hpp"

namespace apexlab {

inline constexpr int vehicle_schema_version = 1;
inline constexpr int planar_vehicle_schema_version = 2;
inline constexpr int nonlinear_vehicle_schema_version = 3;

struct AeroParameters {
    double air_density_kgpm3{1.225};
    double drag_coefficient{0.0};
    double lift_coefficient_down{0.0};
    double reference_area_m2{0.0};
};

struct PowertrainParameters {
    Force maximum_drive_force{si::newtons(0.0)};
    double driven_wheel_static_load_fraction{1.0};
};

struct BrakeParameters {
    Force maximum_brake_force{si::newtons(0.0)};
};

struct TireParameters {
    double friction_coefficient{1.0};
    double rolling_resistance_coefficient{0.0};
};

struct PlanarParameters {
    Distance wheelbase{si::meters(0.0)};
    Distance cg_to_front_axle{si::meters(0.0)};
    Distance cg_to_rear_axle{si::meters(0.0)};
    MomentOfInertia yaw_moment_of_inertia{si::kilogram_square_meters(0.0)};
    double front_cornering_stiffness_n_per_rad{0.0};
    double rear_cornering_stiffness_n_per_rad{0.0};
};

struct NonlinearPlanarParameters {
    Distance front_track{si::meters(0.0)};
    Distance rear_track{si::meters(0.0)};
    Distance cg_height{si::meters(0.0)};
    double front_roll_moment_fraction{0.5};
    double front_brake_bias{0.5};
    std::string drivetrain_type{"RWD"};
    double drive_front_fraction{0.0};
    double tire_mu_reference{1.0};
    Force tire_reference_load{si::newtons(1.0)};
    double tire_load_sensitivity_exponent{0.0};
    double force_iteration_tolerance_mps2{1.0e-7};
    int force_iteration_max_iterations{50};
    double force_iteration_relaxation{0.5};
};

struct SuspensionCorner {
    double spring_rate_n_m{60000.0};
    double compression_damping_ns_m{3500.0};
    double rebound_damping_ns_m{4500.0};
    double nominal_ride_height_m{0.10};
};
struct SprungBodyParameters {
    std::array<SuspensionCorner, 4> corners{};
    double roll_inertia_kg_m2{650.0};
    double pitch_inertia_kg_m2{2200.0};
};

struct VehicleParameters {
    int schema_version{vehicle_schema_version};
    std::string name;
    std::string provenance;
    Mass mass{si::kilograms(0.0)};
    AeroParameters aero;
    PowertrainParameters powertrain;
    BrakeParameters brakes;
    TireParameters tires;
    std::string normal_load_model{"quasi_static"};
    std::optional<SprungBodyParameters> sprung_body;
    std::optional<PlanarParameters> planar;
    std::optional<NonlinearPlanarParameters> nonlinear_planar;
};

void validate(const VehicleParameters& parameters);

[[nodiscard]] VehicleParameters load_vehicle_parameters(const std::filesystem::path& path);

} // namespace apexlab

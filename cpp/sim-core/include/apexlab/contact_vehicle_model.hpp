#pragma once

#include "apexlab/nonlinear_planar_model.hpp"
#include "apexlab/rigid_math.hpp"
#include "apexlab/road_surface.hpp"

namespace apexlab {
struct ContactCornerParameters {
    Vec3 mount_body_m; // x/y fixed, wheel center z = mount.z - extension
    double unsprung_mass_kg{40};
    double radius_m{.34};
    double tire_stiffness_n_m{240000};
    double tire_damping_ns_m{1000};
    double spring_n_m{65000};
    double compression_damping_ns_m{3500};
    double rebound_damping_ns_m{4500};
    double rest_extension_m{.2};
    double min_extension_m{.04};
    double max_extension_m{.3};
    double stop_stiffness_n_m{250000};
    double stop_damping_ns_m{3500};
};
struct ContactVehicleParameters {
    VehicleParameters legacy;
    double sprung_mass_kg{1330};
    Vec3 sprung_inertia_kg_m2{650,2200,2400};
    std::array<ContactCornerParameters,4> corner;
    Vec3 aero_application_body_m{};
    double max_steer_rad{.65};
};
// Explicit engineering estimates for new vertical properties, not OEM data.
ContactVehicleParameters estimated_contact_parameters(const VehicleParameters& legacy);
void validate(const ContactVehicleParameters& parameters);

struct ContactVehicleState {
    Vec3 position_world_m;
    rigid::Quaternion body_to_world;
    Vec3 velocity_world_mps;
    Vec3 omega_body_radps;
    std::array<double,4> extension_m{};
    std::array<double,4> extension_rate_mps{};
};
struct ContactVehicleDerivative {
    Vec3 velocity_world_mps;
    rigid::Quaternion quaternion_rate{0,0,0,0};
    Vec3 acceleration_world_mps2;
    Vec3 angular_acceleration_body_radps2;
    std::array<double,4> extension_rate_mps{};
    std::array<double,4> extension_acceleration_mps2{};
};
struct ContactWheelTelemetry {
    Vec3 center_world_m, velocity_world_mps, patch_world_m, patch_velocity_world_mps;
    Vec3 force_world_n, forward_world, left_world;
    SurfacePoint surface;
    double compression_m{0}, compression_rate_mps{0};
    double suspension_compression_m{0}, suspension_rate_mps{0};
    double suspension_force_n{0}, damper_force_n{0}, normal_force_n{0};
    double slip_angle_rad{0};
    TireContactOutput tire;
    bool contacting{false};
};
struct ContactVehicleEvaluation {
    ContactVehicleDerivative derivative;
    std::array<ContactWheelTelemetry,4> wheel;
    Vec3 aero_force_world_n, aero_moment_body_nm;
    Vec3 external_force_world_n; // includes gravity on sprung and unsprung masses
    Vec3 center_of_mass_world_m, center_of_mass_velocity_world_mps;
    double kinetic_energy_j{0};
    double potential_energy_j{0};
    double max_equation_residual_n{0}; // mixed force/moment equation residual
};

struct ContactOperatingPoint {
    bool override_forces{false};
    double drive_force_n{0}, brake_force_n{0}, extra_sprung_mass_kg{0};
    Vec3 aero_force_world_n{}, aero_moment_body_nm{};
    std::array<double,4> friction_scale{1,1,1,1};
};
using ContactForceProvider = std::function<ContactOperatingPoint(const ContactVehicleState&)>;

// 6-DOF sprung rigid body + four body-axis prismatic unsprung masses.
// No dependency on legacy lap integration or its quasi-static load solver.
class ContactVehicleModel {
  public:
    explicit ContactVehicleModel(ContactVehicleParameters parameters);
    [[nodiscard]] const ContactVehicleParameters& parameters() const { return parameters_; }
    [[nodiscard]] ContactVehicleState flat_equilibrium(double speed_mps=0) const;
    [[nodiscard]] ContactVehicleEvaluation evaluate(const ContactVehicleState& state,
        PlanarControl input,const RoadSurfaceQuery& road, const ContactOperatingPoint& operating = {}) const;
    // Classical RK4, quaternion normalized after the step; <=2 ms physical substeps.
    [[nodiscard]] ContactVehicleState step(const ContactVehicleState& state, PlanarControl input,
        const RoadSurfaceQuery& road,double dt_s, const ContactForceProvider& provider = {}) const;
  private:
    ContactVehicleParameters parameters_;
};
} // namespace apexlab

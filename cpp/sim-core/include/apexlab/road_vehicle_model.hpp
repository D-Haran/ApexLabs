#pragma once
#include "apexlab/nonlinear_planar_model.hpp"
#include "apexlab/track.hpp"
namespace apexlab {
inline constexpr Vec3 world_gravity_mps2{0.0, 0.0, -9.80665};
struct RoadFrame3D { Vec3 position, tangent, lateral, normal; };
[[nodiscard]] RoadFrame3D road_frame_3d(const TrackFrame& frame);
[[nodiscard]] Vec3 project_world_gravity(const RoadFrame3D& frame);
struct SprungState { double heave{0}, roll{0}, pitch{0}, heave_rate{0}, roll_rate{0}, pitch_rate{0}; };
using SprungDerivative = SprungState;
SprungState operator+(SprungState a, SprungState b);
SprungState operator*(SprungState a, double b);
struct RoadVehicleState { PlanarState planar; SprungState body; };
struct RoadVehicleDerivative { PlanarDerivative planar; SprungDerivative body; };
RoadVehicleDerivative operator+(RoadVehicleDerivative a, RoadVehicleDerivative b);
RoadVehicleDerivative operator*(RoadVehicleDerivative a, double b);
RoadVehicleState advance(RoadVehicleState state, RoadVehicleDerivative derivative, Time dt);
struct RoadVehicleForces {
    NonlinearPlanarForces tires;
    SprungDerivative body_derivative;
    std::array<double, 4> compression_m{};
    Vec3 gravity_body;
};
class RoadVehicleModel {
  public:
    explicit RoadVehicleModel(VehicleParameters parameters);
    [[nodiscard]] RoadVehicleForces forces(const RoadVehicleState& state, PlanarControl input,
                                           const TrackFrame& road) const;
    [[nodiscard]] RoadVehicleDerivative derivative(const RoadVehicleState& state, PlanarControl input,
                                                   const TrackFrame& road) const;
    [[nodiscard]] NormalLoadState suspension_loads(const SprungState& state) const;
    [[nodiscard]] SprungDerivative body_derivative(const SprungState& state,
        double tire_fx_n, double tire_fy_n, double downforce_n, double normal_gravity_mps2) const;
    [[nodiscard]] const VehicleParameters& parameters() const { return tires_.parameters(); }
  private:
    NonlinearPlanarVehicleModel tires_;
};
} // namespace apexlab

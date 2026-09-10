#pragma once

#include "apexlab/track.hpp"
#include <array>
#include <functional>
#include <vector>

namespace apexlab {
enum class SurfaceKind { asphalt, curb, runoff, grass };
const char* surface_name(SurfaceKind kind);
struct SurfaceMaterial {
    double friction_scale{1}; // multiplier on the selected tire's reference mu
    double rolling_resistance{.012};
    double roughness_amplitude_m{0};
    double roughness_wavelength_m{3};
};
struct SurfacePoint {
    Vec3 position;
    Vec3 normal{0,0,1};
    SurfaceKind kind{SurfaceKind::asphalt};
    SurfaceMaterial material;
};
using RoadSurfaceQuery = std::function<SurfacePoint(Vec3)>;
// Synthetic curb geometry, never claimed to be surveyed Spa curb locations/heights.
struct CurbSpan { double start_s_m{0}, end_s_m{0}; int side{1}; };
struct RoadSurfaceParameters {
    double curb_width_m{.9};
    double curb_height_m{.045};
    double curb_end_ramp_m{2};
    double runoff_width_m{3};
    std::vector<CurbSpan> curbs;
    std::array<SurfaceMaterial,4> materials{{{1,.012,0,3},{.9,.016,0,3},
        {.95,.018,0,3},{.45,.06,.006,3}}};
};
// Each query projects the wheel's own XY, not the body CG. Far-off-track height is
// an explicit extension of the local road profile, not a survey of the facility.
class TrackRoadSurface {
  public:
    explicit TrackRoadSurface(const PeriodicTrack& track, RoadSurfaceParameters parameters = {});
    [[nodiscard]] SurfacePoint operator()(Vec3 world) const;
    [[nodiscard]] SurfacePoint at(double s_m, double lateral_m) const;
    [[nodiscard]] const RoadSurfaceParameters& parameters() const { return parameters_; }
  private:
    [[nodiscard]] SurfacePoint height_at(double s_m, double lateral_m) const;
    const PeriodicTrack& track_;
    RoadSurfaceParameters parameters_;
};
// Heightfield helper includes analytical slope. Useful for deterministic benches.
RoadSurfaceQuery plane_surface(double grade_x = 0, double grade_y = 0, double height_m = 0);
RoadSurfaceQuery crest_surface(double height_m = 4, double half_length_m = 25);
} // namespace apexlab

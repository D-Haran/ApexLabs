#include "apexlab/road_surface.hpp"
#include "apexlab/rigid_math.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace apexlab {
using namespace rigid;
const char* surface_name(SurfaceKind kind) {
    switch(kind) {
        case SurfaceKind::asphalt:return "asphalt";
        case SurfaceKind::curb:return "curb";
        case SurfaceKind::runoff:return "runoff";
        case SurfaceKind::grass:return "grass";
    }
    throw std::invalid_argument("invalid road material");
}
TrackRoadSurface::TrackRoadSurface(const PeriodicTrack& track, RoadSurfaceParameters parameters)
    : track_(track), parameters_(std::move(parameters)) {
    for (double v:{parameters_.curb_width_m,parameters_.curb_end_ramp_m})
        if (!std::isfinite(v)||v<=0) throw std::invalid_argument("curb widths/ramps must be positive");
    for (double v:{parameters_.curb_height_m,parameters_.runoff_width_m})
        if (!std::isfinite(v)||v<0) throw std::invalid_argument("surface sizes must be nonnegative");
    for(const auto& c:parameters_.curbs)
        if(!std::isfinite(c.start_s_m)||!std::isfinite(c.end_s_m)||c.start_s_m<0||
           c.end_s_m>track_.length_m()||c.end_s_m<=c.start_s_m||(c.side!=1&&c.side!=-1))
            throw std::invalid_argument("curb span must be ordered within the lap; split wrapped spans");
    for(const auto& m:parameters_.materials)
        if(!std::isfinite(m.friction_scale)||m.friction_scale<=0||
           !std::isfinite(m.rolling_resistance)||m.rolling_resistance<0||
           !std::isfinite(m.roughness_amplitude_m)||m.roughness_amplitude_m<0||
           !std::isfinite(m.roughness_wavelength_m)||m.roughness_wavelength_m<=0)
            throw std::invalid_argument("invalid surface material");
}
SurfacePoint TrackRoadSurface::height_at(double s, double lateral) const {
    s=track_.wrap_s(s);
    const auto f=track_.frame_at_s(s);
    const double edge=lateral>=0?f.left_width_m:f.right_width_m;
    const double beyond=std::abs(lateral)-edge;
    SurfacePoint out;out.position=track_.position_3d_at_s(s,lateral);
    double curb=0;
    for(const auto& span:parameters_.curbs) {
        if(s>=span.start_s_m&&s<=span.end_s_m&&lateral*span.side>0) {
            const double ramp=std::clamp(std::min(s-span.start_s_m,span.end_s_m-s)/parameters_.curb_end_ramp_m,0.,1.);
            if(beyond>0&&beyond<parameters_.curb_width_m) {
                out.kind=SurfaceKind::curb;
                const double u=beyond/parameters_.curb_width_m;
                curb=parameters_.curb_height_m*std::pow(std::sin(std::numbers::pi*u),2)*ramp*ramp*(3-2*ramp);
            }
        }
    }
    if(beyond>0&&out.kind!=SurfaceKind::curb)
        out.kind=beyond<=parameters_.runoff_width_m?SurfaceKind::runoff:SurfaceKind::grass;
    out.material=parameters_.materials[static_cast<std::size_t>(out.kind)];
    // Roughness fades in smoothly beyond runoff to avoid a vertical step.
    const double fade=std::clamp((beyond-parameters_.runoff_width_m)/2,0.,1.);
    const double phase=2*std::numbers::pi/out.material.roughness_wavelength_m;
    out.position.z+=curb+out.material.roughness_amplitude_m*fade*fade*(3-2*fade)*
        std::sin(phase*s)*std::sin(phase*lateral);
    return out;
}
SurfacePoint TrackRoadSurface::at(double s, double lateral) const {
    if(!std::isfinite(s)||!std::isfinite(lateral))throw std::invalid_argument("nonfinite road coordinates");
    auto out=height_at(s,lateral);
    constexpr double h=.01;
    const auto along=sub(height_at(s+h,lateral).position,height_at(s-h,lateral).position);
    const auto across=sub(height_at(s,lateral+h).position,height_at(s,lateral-h).position);
    out.normal=unit(cross(along,across));
    if(out.normal.z<0)out.normal=mul(out.normal,-1);
    return out;
}
SurfacePoint TrackRoadSurface::operator()(Vec3 world) const {
    if(!finite(world))throw std::invalid_argument("nonfinite wheel query");
    const auto p=track_.project({world.x,world.y});
    return at(p.s_m,p.lateral_offset_m);
}
RoadSurfaceQuery plane_surface(double gx,double gy,double height) {
    if(!std::isfinite(gx)||!std::isfinite(gy)||!std::isfinite(height))
        throw std::invalid_argument("nonfinite plane");
    return [=](Vec3 p){return SurfacePoint{{p.x,p.y,height+gx*p.x+gy*p.y},unit({-gx,-gy,1}),SurfaceKind::asphalt,{}};};
}
RoadSurfaceQuery crest_surface(double height,double half_length) {
    if(!std::isfinite(height)||height<=0||!std::isfinite(half_length)||half_length<=0)
        throw std::invalid_argument("crest dimensions must be positive");
    // cos^4 hump: height, slope and curvature all join the flat approach continuously.
    return [=](Vec3 p){
        double z=0, slope=0;
        if(std::abs(p.x)<half_length) {
            const double k=std::numbers::pi/(2*half_length),c=std::cos(k*p.x),s=std::sin(k*p.x);
            z=height*c*c*c*c;slope=-4*height*k*c*c*c*s;
        }
        return SurfacePoint{{p.x,p.y,z},unit({-slope,0,1}),SurfaceKind::asphalt,{}};
    };
}
} // namespace apexlab

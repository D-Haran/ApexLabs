#include "apexlab/road_vehicle_model.hpp"
#include <cmath>
#include <stdexcept>
#include <utility>
namespace apexlab {
RoadFrame3D road_frame_3d(const TrackFrame& f) {
    if (!std::isfinite(f.grade) || !std::isfinite(f.heading_rad)) throw std::invalid_argument("nonfinite road frame");
    const double c=std::cos(f.heading_rad), s=std::sin(f.heading_rad), q=1/std::hypot(1.0,f.grade);
    return {{f.position.x,f.position.y,f.elevation_m},{c*q,s*q,f.grade*q},
            {-s,c,0},{-f.grade*c*q,-f.grade*s*q,q}};
}
Vec3 project_world_gravity(const RoadFrame3D& f) {
    const auto dot=[](Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;};
    return {dot(world_gravity_mps2,f.tangent),dot(world_gravity_mps2,f.lateral),dot(world_gravity_mps2,f.normal)};
}
SprungState operator+(SprungState a,SprungState b) {
    return {a.heave+b.heave,a.roll+b.roll,a.pitch+b.pitch,a.heave_rate+b.heave_rate,
            a.roll_rate+b.roll_rate,a.pitch_rate+b.pitch_rate};
}
SprungState operator*(SprungState a,double b) {
    return {a.heave*b,a.roll*b,a.pitch*b,a.heave_rate*b,a.roll_rate*b,a.pitch_rate*b};
}
RoadVehicleDerivative operator+(RoadVehicleDerivative a,RoadVehicleDerivative b) {return {a.planar+b.planar,a.body+b.body};}
RoadVehicleDerivative operator*(RoadVehicleDerivative a,double b) {return {a.planar*b,a.body*b};}
RoadVehicleState advance(RoadVehicleState s,RoadVehicleDerivative d,Time dt) {return {advance(s.planar,d.planar,dt),s.body+d.body*dt.value()};}
RoadVehicleModel::RoadVehicleModel(VehicleParameters p):tires_(std::move(p)) {
    if (!parameters().sprung_body) throw std::invalid_argument("road body model requires suspension parameters");
}
NormalLoadState RoadVehicleModel::suspension_loads(const SprungState& s) const {
    const auto& p=parameters(); const auto& a=*p.planar; const auto& n=*p.nonlinear_planar;
    const std::array<double,4> x={a.cg_to_front_axle.value(),a.cg_to_front_axle.value(),-a.cg_to_rear_axle.value(),-a.cg_to_rear_axle.value()};
    const std::array<double,4> y={n.front_track.value()/2,-n.front_track.value()/2,n.rear_track.value()/2,-n.rear_track.value()/2};
    NormalLoadState loads;
    for (std::size_t i=0;i<4;++i) {
        const auto& c=p.sprung_body->corners[i];
        const double compression=-s.heave-y[i]*s.roll-x[i]*s.pitch;
        const double rate=-s.heave_rate-y[i]*s.roll_rate-x[i]*s.pitch_rate;
        const double static_fraction=(i<2?a.cg_to_rear_axle.value():a.cg_to_front_axle.value())/a.wheelbase.value()/2;
        const double force=p.mass.value()*9.80665*static_fraction+c.spring_rate_n_m*compression+
            (rate>=0?c.compression_damping_ns_m:c.rebound_damping_ns_m)*rate;
        if (!std::isfinite(force)||force<0||std::abs(compression)>c.nominal_ride_height_m)
            throw std::domain_error("sprung-body contact/travel envelope exceeded");
        loads.wheel[i]=si::newtons(force);
    }
    return loads;
}
SprungDerivative RoadVehicleModel::body_derivative(const SprungState& s,double fx,double fy,double down,double gn) const {
    const auto& p=parameters();const auto& a=*p.planar;const auto& n=*p.nonlinear_planar;
    const auto f=suspension_loads(s);
    const double fl=f.wheel[0].value(),fr=f.wheel[1].value(),rl=f.wheel[2].value(),rr=f.wheel[3].value();
    const double mx=(fl-fr)*n.front_track.value()/2+(rl-rr)*n.rear_track.value()/2+n.cg_height.value()*fy;
    const double nose=(fl+fr)*a.cg_to_front_axle.value()-(rl+rr)*a.cg_to_rear_axle.value()+n.cg_height.value()*fx;
    return {s.heave_rate,s.roll_rate,s.pitch_rate,(fl+fr+rl+rr-p.mass.value()*gn-down)/p.mass.value(),
            mx/p.sprung_body->roll_inertia_kg_m2,nose/p.sprung_body->pitch_inertia_kg_m2};
}
RoadVehicleForces RoadVehicleModel::forces(const RoadVehicleState& s,PlanarControl input,const TrackFrame& road) const {
    const auto& p=parameters();const auto& a=*p.planar;const auto& n=*p.nonlinear_planar;
    const auto gravity=project_world_gravity(road_frame_3d(road));
    const double diff=s.planar.yaw.value()-road.heading_rad;
    const double e=std::atan2(std::sin(diff)/std::hypot(1.0,road.grade),std::cos(diff));
    RoadVehicleForces out;
    out.gravity_body={gravity.x*std::cos(e)+gravity.y*std::sin(e),-gravity.x*std::sin(e)+gravity.y*std::cos(e),gravity.z};
    auto loads=suspension_loads(s.body);
    out.tires=tires_.forces_with_loads(s.planar,input,loads);
    // Replace level-road rolling resistance with the actual normal support; aero acts at CG.
    double support=0,fx=0,fy=0;
    for (const auto& w:out.tires.wheel) {support+=w.normal_load.value();fx+=w.body_longitudinal_force.value();fy+=w.body_lateral_force.value();}
    const double sign=s.planar.velocity_x.value()>0?-1:s.planar.velocity_x.value()<0?1:0;
    const double rolling=sign*p.tires.rolling_resistance_coefficient*support;
    out.tires.body_longitudinal_force+=si::newtons(rolling-out.tires.longitudinal.rolling_resistance.value()+p.mass.value()*out.gravity_body.x);
    out.tires.body_lateral_force+=si::newtons(p.mass.value()*out.gravity_body.y);
    out.tires.longitudinal.rolling_resistance=si::newtons(rolling);
    out.body_derivative=body_derivative(s.body,fx,fy,out.tires.longitudinal.aerodynamic_downforce.value(),-gravity.z);
    for (std::size_t i=0;i<4;++i) {
        const double x=i<2?a.cg_to_front_axle.value():-a.cg_to_rear_axle.value();
        const double y=(i<2?n.front_track.value():n.rear_track.value())*(i%2==0?.5:-.5);
        out.compression_m[i]=-s.body.heave-y*s.body.roll-x*s.body.pitch;
    }
    return out;
}
RoadVehicleDerivative RoadVehicleModel::derivative(const RoadVehicleState& s,PlanarControl input,const TrackFrame& road) const {
    const auto out=forces(s,input,road);const auto& p=parameters();const auto frame=road_frame_3d(road);
    const double diff=s.planar.yaw.value()-road.heading_rad,q=1/std::hypot(1.0,road.grade);
    const double e=std::atan2(q*std::sin(diff),std::cos(diff)),c=std::cos(e),sn=std::sin(e);
    const double vx=s.planar.velocity_x.value(),vy=s.planar.velocity_y.value(),r=s.planar.yaw_rate.value();
    const double vt=vx*c-vy*sn,vl=vx*sn+vy*c;
    return {{si::meters_per_second(vt*frame.tangent.x+vl*frame.lateral.x),
             si::meters_per_second(vt*frame.tangent.y+vl*frame.lateral.y),
             si::radians_per_second(r*q/(c*c*q*q+sn*sn)),
             si::meters_per_second_squared(out.tires.body_longitudinal_force.value()/p.mass.value()+r*vy),
             si::meters_per_second_squared(out.tires.body_lateral_force.value()/p.mass.value()-r*vx),
             si::radians_per_second_squared(out.tires.yaw_moment_nm/p.planar->yaw_moment_of_inertia.value())},out.body_derivative};
}
} // namespace apexlab

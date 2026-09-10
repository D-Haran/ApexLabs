#include "apexlab/contact_vehicle_model.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace apexlab {
namespace {
using namespace rigid;
constexpr double g=9.80665;
constexpr std::size_t dofs=10;
using Vector=std::array<double,dofs>;
using Matrix=std::array<Vector,dofs>;
using Jacobian=std::array<Vec3,dofs>;
constexpr std::array<Vec3,3> axes{{{1,0,0},{0,1,0},{0,0,1}}};

Vector solve(Matrix a, Vector b) {
    // Small physical mass matrix; this is a linear solve, not an optimization solver.
    for(std::size_t k=0;k<dofs;++k) {
        std::size_t pivot=k;
        for(std::size_t i=k+1;i<dofs;++i)if(std::abs(a[i][k])>std::abs(a[pivot][k]))pivot=i;
        if(!std::isfinite(a[pivot][k])||std::abs(a[pivot][k])<1e-10)
            throw std::domain_error("singular contact-model mass matrix");
        std::swap(a[k],a[pivot]);std::swap(b[k],b[pivot]);
        for(std::size_t i=k+1;i<dofs;++i) {
            const double f=a[i][k]/a[k][k];
            for(std::size_t j=k+1;j<dofs;++j)a[i][j]-=f*a[k][j];
            b[i]-=f*b[k];a[i][k]=0;
        }
    }
    Vector x{};
    for(std::size_t i=dofs;i-->0;) {
        double v=b[i];for(std::size_t j=i+1;j<dofs;++j)v-=a[i][j]*x[j];
        x[i]=v/a[i][i];
        if(!std::isfinite(x[i]))throw std::domain_error("nonfinite contact acceleration");
    }
    return x;
}
ContactVehicleState shifted(const ContactVehicleState& s,const ContactVehicleDerivative& d,double h) {
    auto out=s;
    out.position_world_m=add(s.position_world_m,mul(d.velocity_world_mps,h));
    out.velocity_world_mps=add(s.velocity_world_mps,mul(d.acceleration_world_mps2,h));
    out.omega_body_radps=add(s.omega_body_radps,mul(d.angular_acceleration_body_radps2,h));
    out.body_to_world={s.body_to_world.w+h*d.quaternion_rate.w,s.body_to_world.x+h*d.quaternion_rate.x,
        s.body_to_world.y+h*d.quaternion_rate.y,s.body_to_world.z+h*d.quaternion_rate.z};
    for(std::size_t i=0;i<4;++i) {
        out.extension_m[i]+=h*d.extension_rate_mps[i];
        out.extension_rate_mps[i]+=h*d.extension_acceleration_mps2[i];
    }
    return out;
}
}
ContactVehicleParameters estimated_contact_parameters(const VehicleParameters& legacy) {
    validate(legacy);
    if(!legacy.planar||!legacy.nonlinear_planar||!legacy.sprung_body)
        throw std::invalid_argument("contact estimates require planar, nonlinear and suspension parameters");
    ContactVehicleParameters p;p.legacy=legacy;
    p.sprung_mass_kg=legacy.mass.value()-160;
    p.sprung_inertia_kg_m2={legacy.sprung_body->roll_inertia_kg_m2,
        legacy.sprung_body->pitch_inertia_kg_m2,legacy.planar->yaw_moment_of_inertia.value()};
    const auto& a=*legacy.planar;const auto& n=*legacy.nonlinear_planar;
    for(std::size_t i=0;i<4;++i) {
        auto& c=p.corner[i];const auto& old=legacy.sprung_body->corners[i];
        c.mount_body_m={i<2?a.cg_to_front_axle.value():-a.cg_to_rear_axle.value(),
            (i%2==0?.5:-.5)*(i<2?n.front_track.value():n.rear_track.value()),0};
        c.spring_n_m=old.spring_rate_n_m;c.compression_damping_ns_m=old.compression_damping_ns_m;
        c.rebound_damping_ns_m=old.rebound_damping_ns_m;
        const double fraction=(i<2?a.cg_to_rear_axle.value():a.cg_to_front_axle.value())/a.wheelbase.value()/2;
        const double spring_load=p.sprung_mass_kg*g*fraction;
        const double tire_load=spring_load+c.unsprung_mass_kg*g;
        const double length=n.cg_height.value()-c.radius_m+tire_load/c.tire_stiffness_n_m;
        c.rest_extension_m=length+spring_load/c.spring_n_m;
        c.min_extension_m=length-.08;c.max_extension_m=length+.12;
    }
    validate(p);return p;
}
void validate(const ContactVehicleParameters& p) {
    validate(p.legacy);
    if(!p.legacy.planar||!p.legacy.nonlinear_planar)
        throw std::invalid_argument("contact model requires nonlinear tire parameters");
    for(double v:{p.sprung_mass_kg,p.sprung_inertia_kg_m2.x,p.sprung_inertia_kg_m2.y,
        p.sprung_inertia_kg_m2.z,p.max_steer_rad})
        if(!std::isfinite(v)||v<=0)throw std::invalid_argument("contact masses/inertias/steering must be positive");
    if(p.max_steer_rad>1.2||!finite(p.aero_application_body_m))throw std::invalid_argument("invalid aero/steering geometry");
    double mass=p.sprung_mass_kg;
    for(const auto& c:p.corner) {
        for(double v:{c.unsprung_mass_kg,c.radius_m,c.tire_stiffness_n_m,c.spring_n_m,c.stop_stiffness_n_m})
            if(!std::isfinite(v)||v<=0)throw std::invalid_argument("wheel masses/stiffness/radius must be positive");
        for(double v:{c.tire_damping_ns_m,c.compression_damping_ns_m,c.rebound_damping_ns_m,c.stop_damping_ns_m})
            if(!std::isfinite(v)||v<0)throw std::invalid_argument("wheel damping must be nonnegative");
        if(!finite(c.mount_body_m)||!std::isfinite(c.rest_extension_m)||
           !std::isfinite(c.min_extension_m)||!std::isfinite(c.max_extension_m)||
           c.min_extension_m>=c.rest_extension_m||c.rest_extension_m>=c.max_extension_m)
            throw std::invalid_argument("invalid suspension geometry/rest/stops");
        mass+=c.unsprung_mass_kg;
    }
    if(std::abs(mass-p.legacy.mass.value())>1e-7)throw std::invalid_argument("sprung plus unsprung mass must equal vehicle mass");
}
ContactVehicleModel::ContactVehicleModel(ContactVehicleParameters p):parameters_(std::move(p)) {validate(parameters_);}
ContactVehicleState ContactVehicleModel::flat_equilibrium(double speed) const {
    if(!std::isfinite(speed))throw std::invalid_argument("nonfinite initial speed");
    const auto& p=parameters_;const auto& a=*p.legacy.planar;
    ContactVehicleState s;s.position_world_m.z=p.legacy.nonlinear_planar->cg_height.value();
    s.velocity_world_mps.x=speed;
    for(std::size_t i=0;i<4;++i) {
        const auto& c=p.corner[i];
        const double fraction=(i<2?a.cg_to_rear_axle.value():a.cg_to_front_axle.value())/a.wheelbase.value()/2;
        s.extension_m[i]=c.rest_extension_m-p.sprung_mass_kg*g*fraction/c.spring_n_m;
    }
    return s;
}
ContactVehicleEvaluation ContactVehicleModel::evaluate(const ContactVehicleState& s,
    PlanarControl input,const RoadSurfaceQuery& road,const ContactOperatingPoint& operating) const {
    if(!finite(s.position_world_m)||!finite(s.velocity_world_mps)||!finite(s.omega_body_radps)||
        !std::isfinite(input.steering_angle.value())||!std::isfinite(input.throttle)||!std::isfinite(input.brake))
        throw std::invalid_argument("nonfinite contact state/control");
    const auto& p=parameters_;const auto& v=p.legacy;const auto& n=*v.nonlinear_planar;
    const double sprung_mass=p.sprung_mass_kg+operating.extra_sprung_mass_kg;
    const double total_mass=v.mass.value()+operating.extra_sprung_mass_kg;
    if(!std::isfinite(sprung_mass)||sprung_mass<=0||!finite(operating.aero_force_world_n)||
       !finite(operating.aero_moment_body_nm)||!std::isfinite(operating.drive_force_n)||
       !std::isfinite(operating.brake_force_n)||operating.brake_force_n<0)
        throw std::invalid_argument("invalid contact operating point");
    for(double grip:operating.friction_scale)if(!std::isfinite(grip)||grip<=0)
        throw std::invalid_argument("invalid contact friction scaling");
    const auto q=normalized(s.body_to_world);const Vec3 omega=s.omega_body_radps;
    const auto up=rotate(q,{0,0,1});
    const double steer=std::clamp(input.steering_angle.value(),-p.max_steer_rad,p.max_steer_rad);
    const double drive=operating.override_forces?operating.drive_force_n:std::clamp(input.throttle,0.,1.)*v.powertrain.maximum_drive_force.value();
    const double brake=operating.override_forces?operating.brake_force_n:std::clamp(input.brake,0.,1.)*v.brakes.maximum_brake_force.value();
    ContactVehicleEvaluation out;
    Matrix mass{};Vector rhs{};
    for(std::size_t i=0;i<3;++i)mass[i][i]=sprung_mass;
    mass[3][3]=p.sprung_inertia_kg_m2.x;mass[4][4]=p.sprung_inertia_kg_m2.y;mass[5][5]=p.sprung_inertia_kg_m2.z;
    const Vec3 angular_momentum{mass[3][3]*omega.x,mass[4][4]*omega.y,mass[5][5]*omega.z};
    const auto gyro=cross(omega,angular_momentum);
    rhs[3]=-gyro.x;rhs[4]=-gyro.y;rhs[5]=-gyro.z;
    const double speed=norm(s.velocity_world_mps);
    const double dynamic_pressure=.5*v.aero.air_density_kgpm3*speed*speed*v.aero.reference_area_m2;
    const auto drag=mul(s.velocity_world_mps,-.5*v.aero.air_density_kgpm3*speed*v.aero.drag_coefficient*v.aero.reference_area_m2);
    out.aero_force_world_n=add(drag,mul(up,-dynamic_pressure*v.aero.lift_coefficient_down));
    out.aero_moment_body_nm=cross(p.aero_application_body_m,inverse_rotate(q,out.aero_force_world_n));
    if(operating.override_forces){out.aero_force_world_n=operating.aero_force_world_n;out.aero_moment_body_nm=operating.aero_moment_body_nm;}
    rhs[0]=out.aero_force_world_n.x;rhs[1]=out.aero_force_world_n.y;rhs[2]=out.aero_force_world_n.z-sprung_mass*g;
    rhs[3]+=out.aero_moment_body_nm.x;rhs[4]+=out.aero_moment_body_nm.y;rhs[5]+=out.aero_moment_body_nm.z;
    out.external_force_world_n=add(out.aero_force_world_n,{0,0,-total_mass*g});
    out.center_of_mass_world_m=mul(s.position_world_m,sprung_mass);
    out.center_of_mass_velocity_world_mps=mul(s.velocity_world_mps,sprung_mass);
    out.kinetic_energy_j=.5*sprung_mass*dot(s.velocity_world_mps,s.velocity_world_mps)+.5*dot(omega,angular_momentum);
    out.potential_energy_j=sprung_mass*g*s.position_world_m.z;
    for(std::size_t i=0;i<4;++i) {
        const auto& c=p.corner[i];auto& w=out.wheel[i];
        const double length=s.extension_m[i],rate=s.extension_rate_mps[i];
        if(!std::isfinite(length)||!std::isfinite(rate))throw std::invalid_argument("nonfinite unsprung state");
        const Vec3 r=add(c.mount_body_m,{0,0,-length});
        const Vec3 relative_velocity=add(cross(omega,r),{0,0,-rate});
        w.center_world_m=add(s.position_world_m,rotate(q,r));
        w.velocity_world_mps=add(s.velocity_world_mps,rotate(q,relative_velocity));
        w.surface=road(w.center_world_m);
        if(!finite(w.surface.position)||!finite(w.surface.normal)||
            !std::isfinite(w.surface.material.friction_scale)||w.surface.material.friction_scale<=0||
            !std::isfinite(w.surface.material.rolling_resistance)||w.surface.material.rolling_resistance<0)
            throw std::domain_error("invalid road surface response");
        w.surface.normal=unit(w.surface.normal);
        const auto normal=w.surface.normal;
        const double gap=dot(sub(w.center_world_m,w.surface.position),normal);
        w.compression_m=std::max(0.,c.radius_m-gap);
        w.compression_rate_mps=-dot(w.velocity_world_mps,normal);
        // Unilateral Kelvin-Voigt: zero outside the tire and never attractive inside.
        w.normal_force_n=w.compression_m>0?std::max(0.,c.tire_stiffness_n_m*w.compression_m+
            c.tire_damping_ns_m*w.compression_rate_mps):0;
        w.contacting=w.normal_force_n>0;
        w.patch_world_m=sub(w.center_world_m,mul(normal,gap));
        w.patch_velocity_world_mps=add(w.velocity_world_mps,
            cross(rotate(q,omega),sub(w.patch_world_m,w.center_world_m)));
        const auto forward=rotate(q,{std::cos(i<2?steer:0),std::sin(i<2?steer:0),0});
        const auto tangent=sub(forward,mul(normal,dot(forward,normal)));
        // Upside-down/vertical wheel attitudes are outside tire calibration, but
        // must remain numerically defined during airborne rotations.
        const Vec3 fallback=std::abs(normal.y)<.8?Vec3{0,1,0}:Vec3{1,0,0};
        w.forward_world=norm(tangent)>1e-9?unit(tangent):unit(cross(fallback,normal));
        w.left_world=unit(cross(normal,w.forward_world));
        const double vx=dot(w.patch_velocity_world_mps,w.forward_world),vy=dot(w.patch_velocity_world_mps,w.left_world);
        const double ramp=std::clamp(std::abs(vx),0.,1.);
        w.slip_angle_rad=-std::atan2(vy,std::max(std::abs(vx),.5))*ramp*ramp*(3-2*ramp);
        const double axle_drive=i<2?n.drive_front_fraction:1-n.drive_front_fraction;
        const double axle_brake=i<2?n.front_brake_bias:1-n.front_brake_bias;
        const double requested=.5*(drive*axle_drive-brake*axle_brake*std::tanh(vx/.2))-
            w.surface.material.rolling_resistance*w.normal_force_n*std::tanh(vx/.2);
        if(w.contacting) {
            const double stiffness=(i<2?v.planar->front_cornering_stiffness_n_per_rad:v.planar->rear_cornering_stiffness_n_per_rad)/2;
            const NonlinearTireModel tire(stiffness,n.tire_mu_reference*w.surface.material.friction_scale*operating.friction_scale[i],
                n.tire_reference_load,n.tire_load_sensitivity_exponent);
            w.tire=tire.evaluate({si::newtons(w.normal_force_n),si::radians(w.slip_angle_rad),si::newtons(requested)});
            w.force_world_n=add(mul(normal,w.normal_force_n),add(mul(w.forward_world,w.tire.longitudinal_force.value()),
                mul(w.left_world,w.tire.lateral_force.value())));
        } // airborne telemetry stays finite and Fx=Fy=Fz=0, even with pedal input
        w.suspension_compression_m=c.rest_extension_m-length;w.suspension_rate_mps=-rate;
        w.damper_force_n=-(rate<0?c.compression_damping_ns_m:c.rebound_damping_ns_m)*rate;
        w.suspension_force_n=c.spring_n_m*w.suspension_compression_m+w.damper_force_n;
        double stop_energy=0;
        if(length<c.min_extension_m) {
            const double travel=c.min_extension_m-length;
            w.suspension_force_n+=c.stop_stiffness_n_m*travel-c.stop_damping_ns_m*std::min(rate,0.);
            stop_energy=.5*c.stop_stiffness_n_m*travel*travel;
        }
        if(length>c.max_extension_m) {
            const double travel=length-c.max_extension_m;
            w.suspension_force_n-=c.stop_stiffness_n_m*travel+c.stop_damping_ns_m*std::max(rate,0.);
            stop_energy=.5*c.stop_stiffness_n_m*travel*travel;
        }
        Jacobian j{};
        for(std::size_t k=0;k<3;++k){j[k]=axes[k];j[k+3]=rotate(q,cross(axes[k],r));}
        j[6+i]=mul(up,-1);
        const auto bias=rotate(q,sub(cross(omega,cross(omega,r)),mul(cross(omega,{0,0,1}),2*rate)));
        const auto external=add(w.force_world_n,{0,0,-c.unsprung_mass_kg*g});
        const auto force_minus_bias=sub(external,mul(bias,c.unsprung_mass_kg));
        for(std::size_t row=0;row<dofs;++row) {
            rhs[row]+=dot(j[row],force_minus_bias);
            for(std::size_t col=0;col<dofs;++col)mass[row][col]+=c.unsprung_mass_kg*dot(j[row],j[col]);
        }
        // The force acts at the patch, not the wheel center. Wheel spin dynamics
        // are not modeled; the contact couple transfers through the ideal carrier.
        const auto patch_torque=inverse_rotate(q,cross(sub(w.patch_world_m,w.center_world_m),w.force_world_n));
        rhs[3]+=patch_torque.x;rhs[4]+=patch_torque.y;rhs[5]+=patch_torque.z;
        rhs[6+i]+=w.suspension_force_n;
        out.external_force_world_n=add(out.external_force_world_n,w.force_world_n);
        out.center_of_mass_world_m=add(out.center_of_mass_world_m,mul(w.center_world_m,c.unsprung_mass_kg));
        out.center_of_mass_velocity_world_mps=add(out.center_of_mass_velocity_world_mps,mul(w.velocity_world_mps,c.unsprung_mass_kg));
        out.kinetic_energy_j+=.5*c.unsprung_mass_kg*dot(w.velocity_world_mps,w.velocity_world_mps);
        out.potential_energy_j+=c.unsprung_mass_kg*g*w.center_world_m.z+
            .5*c.spring_n_m*w.suspension_compression_m*w.suspension_compression_m+
            .5*c.tire_stiffness_n_m*w.compression_m*w.compression_m+stop_energy;
    }
    out.center_of_mass_world_m=mul(out.center_of_mass_world_m,1/total_mass);
    out.center_of_mass_velocity_world_mps=mul(out.center_of_mass_velocity_world_mps,1/total_mass);
    const auto a=solve(mass,rhs);
    for(std::size_t row=0;row<dofs;++row) {
        double residual=-rhs[row];for(std::size_t col=0;col<dofs;++col)residual+=mass[row][col]*a[col];
        out.max_equation_residual_n=std::max(out.max_equation_residual_n,std::abs(residual));
    }
    auto& d=out.derivative;d.velocity_world_mps=s.velocity_world_mps;
    const auto qdot=product(s.body_to_world,{0,omega.x,omega.y,omega.z});
    d.quaternion_rate={qdot.w*.5,qdot.x*.5,qdot.y*.5,qdot.z*.5};
    d.acceleration_world_mps2={a[0],a[1],a[2]};d.angular_acceleration_body_radps2={a[3],a[4],a[5]};
    for(std::size_t i=0;i<4;++i){d.extension_rate_mps[i]=s.extension_rate_mps[i];d.extension_acceleration_mps2[i]=a[6+i];}
    return out;
}
ContactVehicleState ContactVehicleModel::step(const ContactVehicleState& initial,PlanarControl input,
    const RoadSurfaceQuery& road,double dt,const ContactForceProvider& provider) const {
    if(!std::isfinite(dt)||dt<=0||dt>.05)throw std::invalid_argument("contact step must be in (0, .05] seconds");
    const int count=static_cast<int>(std::ceil(dt/.002));const double h=dt/count;
    auto s=initial;
    const auto derivative=[&](const ContactVehicleState& state){return evaluate(state,input,road,provider?provider(state):ContactOperatingPoint{}).derivative;};
    for(int substep=0;substep<count;++substep) {
        const auto a=derivative(s);
        const auto b=derivative(shifted(s,a,h/2));
        const auto c=derivative(shifted(s,b,h/2));
        const auto d=derivative(shifted(s,c,h));
        auto next=shifted(s,a,h/6);next=shifted(next,b,h/3);next=shifted(next,c,h/3);next=shifted(next,d,h/6);
        next.body_to_world=normalized(next.body_to_world);s=next;
    }
    return s;
}
} // namespace apexlab

#include "apexlab/contact_vehicle_model.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace apexlab;
using namespace apexlab::rigid;
namespace {
void require(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
void near(double a,double b,double tolerance,const char* message) {
    if(!std::isfinite(a)||!std::isfinite(b)||std::abs(a-b)>tolerance) {
        std::cerr<<message<<": "<<a<<" vs "<<b<<" tolerance "<<tolerance<<'\n';
        throw std::runtime_error(message);
    }
}
RoadSurfaceQuery lossless(RoadSurfaceQuery road) {
    return [road](Vec3 p){auto f=road(p);f.material.rolling_resistance=0;return f;};
}
Vec3 angular_momentum(const ContactVehicleParameters& p,const ContactVehicleState& s,
    const ContactVehicleEvaluation& e) {
    const auto com=e.center_of_mass_world_m,velocity=e.center_of_mass_velocity_world_mps;
    Vec3 result=add(cross(sub(s.position_world_m,com),mul(sub(s.velocity_world_mps,velocity),p.sprung_mass_kg)),
        rotate(s.body_to_world,{p.sprung_inertia_kg_m2.x*s.omega_body_radps.x,
            p.sprung_inertia_kg_m2.y*s.omega_body_radps.y,p.sprung_inertia_kg_m2.z*s.omega_body_radps.z}));
    for(std::size_t i=0;i<4;++i)result=add(result,cross(sub(e.wheel[i].center_world_m,com),
        mul(sub(e.wheel[i].velocity_world_mps,velocity),p.corner[i].unsprung_mass_kg)));
    return result;
}
struct CrestResult { double airborne{0},clearance{0},peak_load{0}; unsigned min_contacts{4}; };
CrestResult crest(const ContactVehicleModel& model,double speed,double dt) {
    const auto road=lossless(crest_surface(2,35));auto s=model.flat_equilibrium(speed);s.position_world_m.x=-45;
    CrestResult result;
    for(double t=0;t<140/speed;t+=dt) {
        const auto e=model.evaluate(s,{},road);unsigned contacts=0;
        double support=0;
        for(const auto& w:e.wheel){contacts+=w.contacting?1u:0u;support+=w.normal_force_n;
            require(w.normal_force_n>=0&&finite(w.force_world_n),"invalid crest force");
            if(!w.contacting)near(norm(w.force_world_n),0,0,"airborne wheel has force");}
        result.min_contacts=std::min(result.min_contacts,contacts);
        if(contacts==0)result.airborne+=dt;
        result.clearance=std::max(result.clearance,s.position_world_m.z-road(s.position_world_m).position.z-.45);
        result.peak_load=std::max(result.peak_load,support);
        s=model.step(s,{},road,dt);
    }
    return result;
}
}
int main() {
 try {
    const auto legacy=load_vehicle_parameters(std::filesystem::path(APEXLAB_SOURCE_DIR)/"configs/vehicles/mclaren-p1-sprung-approx.json");
    auto p=estimated_contact_parameters(legacy);p.legacy.aero={};
    const ContactVehicleModel model(p);const auto flat=lossless(plane_surface());
    auto s=model.flat_equilibrium();const auto e=model.evaluate(s,{},flat);
    near(norm(e.derivative.acceleration_world_mps2),0,1e-10,"static body balance");
    near(norm(e.derivative.angular_acceleration_body_radps2),0,1e-10,"static moment balance");
    double fz=0;for(std::size_t i=0;i<4;++i){fz+=e.wheel[i].normal_force_n;
        near(e.derivative.extension_acceleration_mps2[i],0,1e-9,"static unsprung balance");}
    near(fz,legacy.mass.value()*9.80665,1e-8,"static weight");
    for(int k=0;k<500;++k)s=model.step(s,{},flat,.002);
    near(s.position_world_m.z,.45,1e-9,"equilibrium drift");
    // Exact total-COM ballistic solution despite suspension extending in mid-air.
    s=model.flat_equilibrium(12);s.position_world_m.z+=8;s.velocity_world_mps.z=2;
    s.omega_body_radps={.3,-.2,.4};
    const auto initial=model.evaluate(s,{},flat);constexpr double duration=.5;
    for(int k=0;k<500;++k) {
        const auto a=model.evaluate(s,{si::radians(.3),1,1},flat);
        for(const auto& w:a.wheel){require(!w.contacting,"unexpected airborne contact");near(norm(w.force_world_n),0,0,"airborne pedals generate force");}
        near(a.external_force_world_n.z,-legacy.mass.value()*9.80665,1e-8,"airborne external gravity");
        require(a.max_equation_residual_n<1e-8,"mass matrix residual");
        s=model.step(s,{si::radians(.3),1,1},flat,.001);
    }
    const auto flight=model.evaluate(s,{},flat);
    near(norm(sub(angular_momentum(p,s,flight),angular_momentum(p,
        [&] { auto start=model.flat_equilibrium(12);start.position_world_m.z+=8;start.velocity_world_mps.z=2;
            start.omega_body_radps={.3,-.2,.4};return start; }(),initial))),0,1e-4,"airborne angular momentum");
    require(flight.kinetic_energy_j+flight.potential_energy_j<=initial.kinetic_energy_j+initial.potential_energy_j+1e-4,
        "passive suspension generates airborne energy");
    const auto expected=add(initial.center_of_mass_world_m,add(mul(initial.center_of_mass_velocity_world_mps,duration),{0,0,-.5*9.80665*duration*duration}));
    near(norm(sub(flight.center_of_mass_world_m,expected)),0,2e-6,"ballistic COM trajectory");
    near(s.body_to_world.w*s.body_to_world.w+s.body_to_world.x*s.body_to_world.x+s.body_to_world.y*s.body_to_world.y+s.body_to_world.z*s.body_to_world.z,1,1e-12,"quaternion normalization");
    // Release during tire unloading: Kelvin-Voigt must not create tensile force.
    s=model.flat_equilibrium();s.velocity_world_mps.z=100;
    for(const auto& w:model.evaluate(s,{},flat).wheel)near(w.normal_force_n,0,0,"road pulls departing tire");
    // Compliance landing: finite loads, suspension stroke, settling without reset.
    s=model.flat_equilibrium();s.position_world_m.z+=.5;
    double landing_peak=0,stroke=0,max_height_step=0;bool touched=false;
    for(int k=0;k<2500;++k) {
        const auto a=model.evaluate(s,{},flat);
        for(std::size_t i=0;i<4;++i){touched|=a.wheel[i].contacting;
            landing_peak=std::max(landing_peak,a.wheel[i].normal_force_n);
            stroke=std::max(stroke,a.wheel[i].suspension_compression_m-e.wheel[i].suspension_compression_m);}
        const auto next=model.step(s,{},flat,.002);
        max_height_step=std::max(max_height_step,std::abs(next.position_world_m.z-s.position_world_m.z));s=next;
    }
    require(touched&&landing_peak>5000&&landing_peak<100000,"invalid landing response");
    require(stroke>.015&&max_height_step<.02,"landing teleports or has no suspension motion");
    near(s.position_world_m.z,.45,.002,"landing does not settle");
    std::cout<<"landing: peak wheel N="<<landing_peak<<" additional compression m="<<stroke<<'\n';
    // Existing nonlinear tire law is used exactly with the measured contact load.
    s=model.flat_equilibrium(20);s.velocity_world_mps.y=1;
    const auto tires=model.evaluate(s,{si::radians(.05),.4,.1},flat);
    for(std::size_t i=0;i<4;++i) {
        const auto& w=tires.wheel[i];
        const NonlinearTireModel tire((i<2?legacy.planar->front_cornering_stiffness_n_per_rad:legacy.planar->rear_cornering_stiffness_n_per_rad)/2,
            legacy.nonlinear_planar->tire_mu_reference,legacy.nonlinear_planar->tire_reference_load,legacy.nonlinear_planar->tire_load_sensitivity_exponent);
        const auto expected_tire=tire.evaluate({si::newtons(w.normal_force_n),si::radians(w.slip_angle_rad),w.tire.requested_longitudinal_force});
        near(w.tire.lateral_force.value(),expected_tire.lateral_force.value(),1e-10,"legacy tire mismatch");
        require(w.tire.friction_utilization<=1+1e-12,"friction circle violation");
    }
    // Mirrored rear slip forces (road on vehicle), even with zero rear steering.
    s=model.flat_equilibrium(20);s.velocity_world_mps.y=-1;
    const auto left=model.evaluate(s,{},flat);s.velocity_world_mps.y=1;
    const auto right=model.evaluate(s,{},flat);
    require(left.wheel[2].force_world_n.y>0&&right.wheel[2].force_world_n.y<0,"rear lateral sign");
    near(left.wheel[2].force_world_n.y,-right.wheel[2].force_world_n.y,1e-8,"mirrored lateral symmetry");
    // Four independent surfaces: a left-side step must not move the right tires.
    s=model.flat_equilibrium();const auto split=[](Vec3 p){return SurfacePoint{{p.x,p.y,p.y>0?.03:0},{0,0,1},SurfaceKind::asphalt,{}};};
    const auto split_forces=model.evaluate(s,{},split);
    require(split_forces.wheel[0].normal_force_n>e.wheel[0].normal_force_n+6000,"left bump missing");
    near(split_forces.wheel[1].normal_force_n,e.wheel[1].normal_force_n,1e-8,"right tire shares left plane");
    // Curbs / runoff / rough grass query, including heightfield normal and seam.
    const PeriodicTrack track(make_circle_track(100));RoadSurfaceParameters surface_parameters;
    surface_parameters.curbs={{10,30,1}};const TrackRoadSurface surface(track,surface_parameters);
    near(surface.at(20,6.45).position.z,.045,1e-10,"physical curb height");
    require(surface.at(20,6.45).kind==SurfaceKind::curb,"curb classification");
    require(surface.at(40,7).kind==SurfaceKind::runoff,"runoff classification");
    require(surface.at(40,12).kind==SurfaceKind::grass,"grass classification");
    require(surface.at(40,12).material.friction_scale<surface.at(40,0).material.friction_scale,"offtrack grip");
    const auto c=surface.at(20,6.3);near(norm(c.normal),1,1e-12,"surface normal unit");
    const auto world=surface(c.position);near(norm(sub(world.position,c.position)),0,.002,"road world/track alignment");
    near(norm(sub(surface.at(0,0).position,surface.at(track.length_m(),0).position)),0,1e-8,"surface seam");
    // Imported Spa road queries use each wheel's own elevation, including grade.
    const PeriodicTrack spa(load_track_definition(std::filesystem::path(APEXLAB_SOURCE_DIR)/"configs/tracks/spa-francorchamps.json"));
    const TrackRoadSurface spa_surface(spa);
    for(int sample=0;sample<16;++sample) {
        const double progress=spa.length_m()*sample/16;const auto f=spa.frame_at_s(progress);
        s=model.flat_equilibrium(20);
        s.body_to_world=product(from_rotation_vector({0,0,f.heading_rad}),from_rotation_vector({0,-std::atan(f.grade),0}));
        s.position_world_m=add(spa.position_3d_at_s(progress),rotate(s.body_to_world,{0,0,.45}));
        s.velocity_world_mps=rotate(s.body_to_world,{20,0,0});
        const auto evaluation=model.evaluate(s,{},[&spa_surface](Vec3 point){return spa_surface(point);});
        for(const auto& w:evaluation.wheel) {
            require(finite(w.force_world_n)&&w.normal_force_n>=0,"Spa contact query invalid");
            near(norm(w.surface.normal),1,1e-10,"Spa normal unit");
        }
    }
    // Crest transition emerges from equations; no jump branch or attached constraint.
    const auto low=crest(model,12,.002);const auto fast=crest(model,40,.002);const auto refined=crest(model,40,.001);
    std::cout<<"crest 12m/s: contacts="<<low.min_contacts<<" airborne="<<low.airborne<<"; 40m/s: contacts="<<fast.min_contacts<<" airborne="<<fast.airborne<<" clearance="<<fast.clearance<<" peak N="<<fast.peak_load<<'\n';
    require(low.min_contacts==4,"low-speed crest loses contact");
    require(fast.airborne>.05&&fast.clearance>.05,"high-speed crest stays glued");
    near(fast.airborne,refined.airborne,.006,"crest airborne timestep convergence");
    near(fast.clearance,refined.clearance,.003,"crest clearance timestep convergence");
    near(fast.peak_load,refined.peak_load,fast.peak_load*.025,"landing load timestep convergence");
    bool rejected=false;try{auto invalid=p;invalid.corner[0].unsprung_mass_kg=-1;ContactVehicleModel bad(invalid);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"invalid mass accepted");
    rejected=false;try{s=model.flat_equilibrium();s.position_world_m.x=std::numeric_limits<double>::quiet_NaN();(void)model.evaluate(s,{},flat);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"nonfinite state accepted");
    std::cout<<"3D contact analytical, landing, tire, surface and crest tests passed\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}

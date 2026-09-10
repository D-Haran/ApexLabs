#include "apexlab/dynamic_vehicle.hpp"
#include "input_adapter.hpp"
#include "nlohmann/json.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <numbers>
#include <string>
using namespace apexlab;
using namespace apexlab::rigid;
using json=nlohmann::json;
namespace {
json vec(Vec3 v){return {v.x,v.y,v.z};}
json render_context(const char* course){
 auto path=std::filesystem::path(course);auto stem=path.stem().string();const auto pos=stem.find("-dynamic");if(pos!=std::string::npos)stem.erase(pos);
 std::ifstream file(path.parent_path()/(stem+".render.json"));return file?json::parse(file):json();
}
RoadSurfaceParameters curb_parameters(const json& context,double length){
 RoadSurfaceParameters p;if(!context.is_null())for(const auto& c:context.value("curbs",json::array())){
  const double begin=c.at("start_m"),end=std::min(length,c.at("end_m").get<double>());if(begin<end)p.curbs.push_back({begin,end,c.at("side")});
 }return p;
}
struct Runtime {
 DynamicVehicleModel model;PeriodicTrack track;json context;TrackRoadSurface surface;
 DynamicState state;DynamicControl control,target;
 double time=0,progress=0,lateral=0,start_s=0,lap_time=0,steering_rate=1.4,saturation=.65;
 bool profiling=false;double simulation_cost=0,projection_cost=0,objective_cost=0;std::size_t simulation_steps=0;
 static double seconds(std::chrono::steady_clock::time_point start){return std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();}
 KeyboardInput keyboard;KeyboardParameters keyboard_parameters;bool keyboard_friendly=false;double steer_key=0,delta_dot=0;
 double physics_dt=.002;bool assisted=false;int lap=0;std::string output,error;
 Runtime(const char* vehicle,const char* course):model(load_dynamic_vehicle(vehicle)),track(load_track_definition(course)),context(render_context(course)),surface(track,curb_parameters(context,track.length_m())){reset(0,0);}
 RoadSurfaceQuery road(){return [this](Vec3 p){const auto start=profiling?std::chrono::steady_clock::now():std::chrono::steady_clock::time_point{};const auto c=track.project({p.x,p.y},progress,12);auto result=surface.at(c.s_m,c.lateral_offset_m);if(profiling)projection_cost+=seconds(start);return result;};}
 void reset(double start,double speed){
   keyboard={};delta_dot=0;last_steer=0;time=0;progress=track.wrap_s(start);start_s=progress;lateral=0;lap=0;lap_time=0;control={};target={};
   state=model.initial(speed);const auto f=track.frame_at_s(progress);
   const auto q=product(from_rotation_vector({0,0,f.heading_rad}),from_rotation_vector({0,-std::atan(f.grade),0}));
   state.contact.position_world_m=add({f.position.x,f.position.y,f.elevation_m},rotate(q,state.contact.position_world_m));
   state.contact.body_to_world=q;state.contact.velocity_world_mps=rotate(q,{speed,0,0});
 }
 void advance(double dt){
   if(!std::isfinite(dt)||dt<=0||dt>.1)throw std::invalid_argument("advance must be in (0,.1]");
   const int count=static_cast<int>(std::ceil(dt/physics_dt));const double h=dt/count;
   for(int k=0;k<count;++k){
     double requested=std::clamp(target.pedals.steering_angle.value(),-saturation,saturation);
     if(assisted){const double v=norm(state.contact.velocity_world_mps);requested=std::clamp(requested,-.65/(1+v*v/250),.65/(1+v*v/250));}
     control=target;
     if(keyboard_friendly){requested=keyboard.step(steer_key,target.pedals.throttle,target.pedals.brake,norm(state.contact.velocity_world_mps),model.parameters().contact.legacy.planar->wheelbase.value(),h,keyboard_parameters);control.pedals.throttle=keyboard.throttle;control.pedals.brake=keyboard.brake;}
     const double previous_steer=last_steer;
     control.pedals.steering_angle=si::radians(last_steer+std::clamp(requested-last_steer,-steering_rate*h,steering_rate*h));last_steer=control.pedals.steering_angle.value();delta_dot=(last_steer-previous_steer)/h;
     const auto step_start=profiling?std::chrono::steady_clock::now():std::chrono::steady_clock::time_point{};const double projection_before=projection_cost;state=model.step(state,control,road(),h);time+=h;++simulation_steps;if(profiling)simulation_cost+=seconds(step_start)-(projection_cost-projection_before);
     const auto projection_start=profiling?std::chrono::steady_clock::now():std::chrono::steady_clock::time_point{};
     const auto c=track.project({state.contact.position_world_m.x,state.contact.position_world_m.y},progress,8);if(profiling)projection_cost+=seconds(projection_start);
     double ds=c.s_m-track.wrap_s(progress);if(ds>track.length_m()/2)ds-=track.length_m();if(ds<-track.length_m()/2)ds+=track.length_m();
     progress+=ds;lateral=c.lateral_offset_m;
     const int next=static_cast<int>(std::floor((progress-start_s)/track.length_m()));
     if(next>lap){lap=next;lap_time=time;model.begin_lap(state);}
     if(!finite(state.contact.position_world_m)||norm(state.contact.velocity_world_mps)>180||std::abs(state.contact.position_world_m.z-track.frame_at_s(progress).elevation_m)>25)throw std::runtime_error("outside dynamic operating envelope; reset required");
   }
 }
 double last_steer=0;
 json snapshot(){
  auto e=model.evaluate(state,control,road());const auto& c=state.contact;const auto& q=c.body_to_world;
  const auto a_body=inverse_rotate(q,e.contact.derivative.acceleration_world_mps2);
  json j={{"schema",1},{"model","dynamic_contact"},{"time",time},{"s",progress},{"lateral",lateral},{"lap",lap},{"lapElapsed",time-lap_time},
    {"position",vec(c.position_world_m)},{"quaternion",{q.w,q.x,q.y,q.z}},{"velocity",vec(c.velocity_world_mps)},{"acceleration",vec(e.contact.derivative.acceleration_world_mps2)},
    {"bodyAcceleration",vec(a_body)},{"omega",vec(c.omega_body_radps)},{"gear",state.gear},{"rpm",e.rpm},{"steering",control.pedals.steering_angle.value()},
    {"throttle",control.pedals.throttle},{"brake",control.pedals.brake},{"fuel",state.fuel_mass_kg},{"battery",state.battery_energy_j},
    {"deployed",state.deployed_lap_j},{"recovered",state.recovered_lap_j},{"fuelFlow",e.fuel_flow_kg_s},{"electricPower",e.battery_discharge_w-e.battery_charge_w},
    {"drag",e.drag_n},{"downforce",e.downforce_n},{"frontAeroFraction",e.front_aero_fraction},{"cda",e.cda_m2},{"cla",e.cla_m2},
    {"frontHeight",e.front_height_m},{"rearHeight",e.rear_height_m},{"drs",state.drs_position},{"aeroState",state.aero_position},{"aeroForce",vec(e.contact.aero_force_world_n)},
    {"steeringRateActual",delta_dot},{"steerCommand",keyboard_friendly?keyboard.steer:target.pedals.steering_angle.value()/saturation},{"steeringRequest",keyboard_friendly?keyboard.steer*KeyboardInput::angle_limit(norm(c.velocity_world_mps),model.parameters().contact.legacy.planar->wheelbase.value(),keyboard_parameters):target.pedals.steering_angle.value()},{"keyboardFriendly",keyboard_friendly},{"assisted",assisted},{"wheels",json::array()}};
  for(std::size_t i=0;i<4;++i){const auto& w=e.contact.wheel[i];j["wheels"].push_back({{"center",vec(w.center_world_m)},{"patch",vec(w.patch_world_m)},{"force",vec(w.force_world_n)},
    {"normal",vec(w.surface.normal)},{"forward",vec(w.forward_world)},{"left",vec(w.left_world)},{"contact",w.contacting},{"fz",w.normal_force_n},{"fx",w.tire.longitudinal_force.value()},
    {"fy",w.tire.lateral_force.value()},{"slip",w.slip_angle_rad},{"utilization",w.tire.friction_utilization},{"requestedUtilization",w.tire.requested_friction_utilization},
    {"compression",w.suspension_compression_m},{"tireCompression",w.compression_m},{"damperVelocity",w.suspension_rate_mps},{"damperForce",w.damper_force_n},
    {"material",static_cast<int>(w.surface.kind)},{"tread",state.tread_c[i]},{"carcass",state.carcass_c[i]},{"wear",state.wear[i]}});}
  return j;
 }
 json metadata(){
   const auto& p=model.parameters();const auto& n=*p.contact.legacy.nonlinear_planar;const auto& b=*p.contact.legacy.planar;
   json j={{"model","dynamic_contact"},{"name",p.name},{"family",p.family},{"track",track.definition().name},{"length",track.length_m()},
   {"sectors",track.definition().sector_boundaries_fraction},{"physicsHz",500},{"values",p.values},
   {"parameters",{{"cg_height_m",n.cg_height.value()},{"body_axle_midpoint_m",(b.cg_to_front_axle.value()-b.cg_to_rear_axle.value())/2},{"total_mass_kg",p.contact.legacy.mass.value()},
   {"corner",json::array()}}},{"geometry",json::array()}};
   for(const auto& c:p.contact.corner)j["parameters"]["corner"].push_back({{"radius_m",c.radius_m}});
   const int count=static_cast<int>(std::ceil(track.length_m()/2));
   for(int i=0;i<=count;++i){const double s=track.length_m()*i/count;const auto f=track.frame_at_s(s);j["geometry"].push_back({{"s_m",s},{"x_m",f.position.x},{"y_m",f.position.y},{"elevation_m",f.elevation_m},{"grade",f.grade},{"heading_rad",f.heading_rad},{"curvature_1_m",f.curvature_1_m},{"left_width_m",f.left_width_m},{"right_width_m",f.right_width_m},{"bank_angle_rad",0}});}
   j["renderContext"]=context;j["physicalKerbs"]=json::array();
   for(const auto& span:surface.parameters().curbs){json strip={{"columns",5},{"positions",json::array()},{"colors",json::array()}};
    const int curb_count=static_cast<int>(std::ceil((span.end_s_m-span.start_s_m)/.75));
    for(int i=0;i<=curb_count;++i){const double s=span.start_s_m+(span.end_s_m-span.start_s_m)*i/curb_count;const auto f=track.frame_at_s(s);const double edge=span.side>0?f.left_width_m:f.right_width_m;
     for(int k=0;k<5;++k){const auto point=surface.at(s,span.side*(edge+surface.parameters().curb_width_m*k/4));strip["positions"].push_back(vec(point.position));strip["colors"].push_back(static_cast<int>(s/2)%2);}}
    j["physicalKerbs"].push_back(strip);
   }
   return j;
 }

 double profile(const std::vector<double>& v,double s) const {
   if(v.empty())return 0;
   const double x=track.wrap_s(s)/track.length_m()*static_cast<double>(v.size());const auto i=static_cast<long>(std::floor(x));const double t=x-i;
   auto at=[&](long k){const long n=static_cast<long>(v.size());return v[static_cast<std::size_t>((k%n+n)%n)];};
   return .5*((2*at(i))+(-at(i-1)+at(i+1))*t+(2*at(i-1)-5*at(i)+4*at(i+1)-at(i+2))*t*t+(-at(i-1)+3*at(i)-3*at(i+1)+at(i+2))*t*t*t);
 }
 json physical_state() const {
   const auto& s=state;const auto& c=s.contact;const auto& q=c.body_to_world;
   return {{"position",vec(c.position_world_m)},{"quaternion",{q.w,q.x,q.y,q.z}},{"velocity",vec(c.velocity_world_mps)},{"omega",vec(c.omega_body_radps)},
     {"extension",c.extension_m},{"extensionRate",c.extension_rate_mps},{"gear",s.gear},{"shift",s.shift_remaining_s},{"aero",s.aero_position},{"drs",s.drs_position},
     {"fuel",s.fuel_mass_kg},{"battery",s.battery_energy_j},{"deployed",s.deployed_lap_j},{"recovered",s.recovered_lap_j},{"tread",s.tread_c},{"carcass",s.carcass_c},{"wear",s.wear},
     {"s",progress},{"steering",last_steer}};
 }
 void restore(const json& j){
   auto v=[](const json& a){return Vec3{a[0],a[1],a[2]};};auto& c=state.contact;
   c.position_world_m=v(j.at("position"));c.velocity_world_mps=v(j.at("velocity"));c.omega_body_radps=v(j.at("omega"));
   const auto q=j.at("quaternion");c.body_to_world={q[0],q[1],q[2],q[3]};
   c.extension_m=j.at("extension").get<std::array<double,4>>();c.extension_rate_mps=j.at("extensionRate").get<std::array<double,4>>();
   state.gear=j.at("gear");state.shift_remaining_s=j.at("shift");state.aero_position=j.at("aero");state.drs_position=j.at("drs");
   state.fuel_mass_kg=j.at("fuel");state.battery_energy_j=j.at("battery");state.deployed_lap_j=j.at("deployed");state.recovered_lap_j=j.at("recovered");
   state.tread_c=j.at("tread").get<std::array<double,4>>();state.carcass_c=j.at("carcass").get<std::array<double,4>>();state.wear=j.at("wear").get<std::array<double,4>>();
   progress=j.at("s");last_steer=j.at("steering");time=0;lap=0;start_s=progress;lap_time=0;
 }
 json rollout(const json& j){
   const auto start_clock=std::chrono::steady_clock::now();profiling=j.value("profileTiming",false);simulation_cost=projection_cost=objective_cost=0;simulation_steps=0;
   const auto offsets=j.at("offsets").get<std::vector<double>>(),scales=j.at("scales").get<std::vector<double>>();
   if(offsets.size()<4||offsets.size()!=scales.size())throw std::invalid_argument("profile needs matching periodic nodes >=4");
   for(double v:offsets)if(!std::isfinite(v)||std::abs(v)>8)throw std::invalid_argument("offset outside profile bounds");
   for(double v:scales)if(!std::isfinite(v)||v<.3||v>2)throw std::invalid_argument("speed scale outside profile bounds");
   physics_dt=j.value("dt",.002);if(physics_dt<.0005||physics_dt>.002)throw std::invalid_argument("invalid rollout dt");
   const bool record=j.value("record",false);const int warmup=j.value("warmup",1);const auto& p=model.parameters();
   const int nodes=static_cast<int>(std::ceil(track.length_m()/3));const double ds=track.length_m()/nodes;
   std::vector<double> speed(static_cast<std::size_t>(nodes));
   // Feasible-guess speed envelope only. Solver changes speed and path profiles;
   // the full native dynamic rollout, not this envelope, determines feasibility.
   for(int i=0;i<nodes;++i){const auto f=track.frame_at_s(i*ds);speed[static_cast<std::size_t>(i)]=std::min(p.family=="p1"?70.:85.,std::sqrt(5.5/std::max(.0001,std::abs(f.curvature_1_m))));}
   for(int cycle=0;cycle<3;++cycle){for(int i=nodes-1;i>=0;--i){auto& v=speed[static_cast<std::size_t>(i)];const double next=speed[static_cast<std::size_t>((i+1)%nodes)];v=std::min(v,std::sqrt(next*next+2*5.*ds));}
     for(int i=0;i<nodes;++i){auto& v=speed[static_cast<std::size_t>((i+1)%nodes)];const double old=speed[static_cast<std::size_t>(i)];v=std::min(v,std::sqrt(old*old+2*4.*ds));}}
   reset(0,speed[0]*scales[0]);last_steer=0;assisted=false;keyboard_friendly=false;steering_rate=1.4;saturation=.65;state.aero_position=1;
   if(j.contains("initial"))restore(j["initial"]);
   const double origin=progress;const double finish=origin+track.length_m()*(warmup+1);const double begin=origin+track.length_m()*warmup;
   double measurement_start=0,start_fuel=state.fuel_mass_kg,path=0,clearance=1e9,balance=0,stability=0,wear=0,max_tire=0,max_request=0,max_defect=0,airborne=0;
   std::array<double,4> start_wear=state.wear;Vec3 last=state.contact.position_world_m;json samples=json::array(),commands=json::array(),initial;
   bool measuring=warmup==0,complete=false;if(measuring)initial=physical_state();
   const double maximum_time=j.value("maximum_time",800.);
   while(time<maximum_time){
     const auto f=track.frame_at_s(progress);const double ey=profile(offsets,progress),eps=1.;
     const auto point=[&](double s){return track.position_at_s(s,profile(offsets,s));};const auto a=point(progress-eps),b=point(progress),c=point(progress+eps);
     const double dx=(c.x-a.x)/(2*eps),dy=(c.y-a.y)/(2*eps),ddx=(c.x-2*b.x+a.x)/(eps*eps),ddy=(c.y-2*b.y+a.y)/(eps*eps);
     const double heading=std::atan2(dy,dx),curvature=(dx*ddy-dy*ddx)/std::pow(dx*dx+dy*dy,1.5);
     const auto forward=rotate(state.contact.body_to_world,{1,0,0});const double yaw=std::atan2(forward.y,forward.x);
     const double heading_error=std::remainder(yaw-heading,2*std::numbers::pi);
     const double vx=dot(state.contact.velocity_world_mps,forward);
     const double steering=std::atan(p.contact.legacy.planar->wheelbase.value()*curvature)-.045*(lateral-ey)-.9*heading_error;
     const double vref=std::max(4.,profile(speed,progress)*profile(scales,progress));
     const double demand=.22*(vref-vx);
     target.pedals={si::radians(std::clamp(steering,-.6,.6)),std::clamp(demand,0.,1.),std::clamp(-demand,0.,1.)};
     target.aero_mode=1;target.deployment=j.value("deployment",1.);target.drs=false;
     if(measuring&&record)commands.push_back({{"time",time-measurement_start},{"steering",target.pedals.steering_angle.value()},{"throttle",target.pedals.throttle},{"brake",target.pedals.brake},{"deployment",target.deployment},{"aeroMode",1}});
     advance(.02);
     if(!measuring&&progress>=begin){measuring=true;measurement_start=time;start_fuel=state.fuel_mass_kg;start_wear=state.wear;initial=physical_state();last=state.contact.position_world_m;}
     if(measuring){
       const auto objective_start=profiling?std::chrono::steady_clock::now():std::chrono::steady_clock::time_point{};const double projection_before=projection_cost;
       const auto e=model.evaluate(state,control,road());const auto frame=track.frame_at_s(progress);
       const double margin=p.get("vehicle_half_width_m")+p.get("boundary_margin_m");
       clearance=std::min(clearance,std::min(frame.left_width_m-lateral,frame.right_width_m+lateral)-margin);
       path+=norm(sub(state.contact.position_world_m,last));last=state.contact.position_world_m;
       double front=0,rear=0;int contacts=0;
       for(std::size_t i=0;i<4;++i){const auto& w=e.contact.wheel[i];(i<2?front:rear)+=w.tire.friction_utilization/2;
         max_tire=std::max(max_tire,w.tire.friction_utilization);max_request=std::max(max_request,w.tire.requested_friction_utilization);if(w.contacting)++contacts;}
       if(contacts==0)airborne+=.02;
       balance+=std::pow(front-rear,2)*.02;const auto up=rotate(state.contact.body_to_world,{0,0,1});
       stability+=(up.x*up.x+up.y*up.y+norm(state.contact.omega_body_radps)*norm(state.contact.omega_body_radps)*.05)*.02;
       max_defect=std::max(max_defect,e.contact.max_equation_residual_n);
       if(record){auto row=snapshot();row["time"]=time-measurement_start;row["s"]=progress-begin;row["lapElapsed"]=time-measurement_start;samples.push_back(row);}
       if(profiling)objective_cost+=seconds(objective_start)-(projection_cost-projection_before);
     }
     if(progress>=finish){complete=true;break;}
     if(std::abs(lateral)>std::max(f.left_width_m,f.right_width_m)+5||vx<1){break;}
   }
   const double duration=time-measurement_start;for(std::size_t i=0;i<4;++i)wear+=state.wear[i]-start_wear[i];
   json result={{"model","dynamic_contact"},{"complete",complete},{"lap_time_s",duration},{"path_m",path},{"minimum_clearance_m",clearance},
   {"balance_cost",balance/std::max(.02,duration)},{"stability_cost",stability/std::max(.02,duration)},{"fuel_kg",start_fuel-state.fuel_mass_kg},{"wear",wear},
   {"maximum_tire_utilization",max_tire},{"maximum_requested_tire_utilization",max_request},{"maximum_mass_equation_residual",max_defect},{"all_airborne_s",airborne},
   {"progress_m",progress-begin},{"dt_s",physics_dt},{"wall_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-start_clock).count()},{"final",physical_state()},{"initial",initial}};
   if(!initial.is_null()) {
     const auto source=initial.at("velocity");const Vec3 vi{source[0],source[1],source[2]};
     const auto oq=initial.at("quaternion");const Quaternion qi{oq[0],oq[1],oq[2],oq[3]};
     const auto velocity_delta=sub(inverse_rotate(state.contact.body_to_world,state.contact.velocity_world_mps),inverse_rotate(qi,vi));
     const auto initial_forward=rotate(qi,{1,0,0}),final_forward=rotate(state.contact.body_to_world,{1,0,0});
     const double initial_heading=std::atan2(initial_forward.y,initial_forward.x)-track.frame_at_s(initial.at("s").get<double>()).heading_rad;
     const double final_heading=std::atan2(final_forward.y,final_forward.x)-track.frame_at_s(progress).heading_rad;
     double travel=0,rate=0;for(std::size_t i=0;i<4;++i)travel=std::max(travel,std::abs(state.contact.extension_m[i]-initial.at("extension")[i].get<double>()));
     const auto wi=initial.at("omega");rate=norm(sub(state.contact.omega_body_radps,{wi[0],wi[1],wi[2]}));
     const double yaw=std::abs(std::remainder(final_heading-initial_heading,2*std::numbers::pi));
     const double v=norm(velocity_delta);const double periodic=std::max({v/.5,yaw/.03,rate/.05,travel/.01});
     result["periodicity"]={{"body_velocity_mps",v},{"heading_rad",yaw},{"angular_rate_radps",rate},{"suspension_m",travel},{"normalized_max",periodic}};
   }
   result["simulation_steps"]=simulation_steps;
   if(profiling)result["timing"]={{"simulation_s",simulation_cost},{"track_projection_s",projection_cost},{"objective_and_diagnostics_s",objective_cost},{"other_native_s",std::max(0.,seconds(start_clock)-simulation_cost-projection_cost-objective_cost)}};
   if(record){result["samples"]=samples;result["commands"]=commands;}
   return result;
 }
 json replay(const json& j){
   reset(0,0);restore(j.at("initial"));physics_dt=j.value("replay_dt",.001);if(physics_dt<.0005||physics_dt>.002)throw std::invalid_argument("invalid replay timestep");
   const double origin=progress;json samples=json::array();
   for(const auto& c:j.at("commands")){command(c);advance(.02);auto row=snapshot();row["s"]=progress-origin;samples.push_back(row);}
   return {{"samples",samples},{"final",physical_state()},{"lap_time_s",time},{"progress_m",progress-origin},{"dt_s",physics_dt}};
 }
 void command(const json& j){
   if(j.value("reset",false)){reset(j.value("start",0.),j.value("speed",0.));last_steer=0;}
   auto bound=[&](const char* key,double old,double lo,double hi){double v=j.value(key,old);if(!std::isfinite(v)||v<lo||v>hi)throw std::invalid_argument(std::string("invalid ")+key);return v;};
   target.pedals.throttle=bound("throttle",target.pedals.throttle,0,1);target.pedals.brake=bound("brake",target.pedals.brake,0,1);
   target.pedals.steering_angle=si::radians(bound("steering",target.pedals.steering_angle.value(),-.65,.65));
   steering_rate=bound("steeringRate",steering_rate,.1,5);saturation=bound("saturation",saturation,.05,.65);
   target.deployment=bound("deployment",target.deployment,0,1);target.drs=j.value("drs",target.drs);target.aero_mode=j.value("aeroMode",target.aero_mode);
   assisted=j.value("assisted",assisted);
   const bool friendly=j.value("keyboardFriendly",keyboard_friendly);
   if(friendly!=keyboard_friendly)keyboard={};keyboard_friendly=friendly;
   steer_key=bound("steerKey",steer_key,-1,1);
   keyboard_parameters.rise=bound("inputRise",keyboard_parameters.rise,.1,10);
   keyboard_parameters.release=bound("inputReturn",keyboard_parameters.release,.1,10);
   keyboard_parameters.throttle_rise=bound("throttleRise",keyboard_parameters.throttle_rise,.1,10);
   keyboard_parameters.throttle_release=bound("throttleRelease",keyboard_parameters.throttle_release,.1,10);
   keyboard_parameters.brake_rise=bound("brakeRise",keyboard_parameters.brake_rise,.1,10);
   keyboard_parameters.brake_release=bound("brakeRelease",keyboard_parameters.brake_release,.1,10);
   keyboard_parameters.low_speed_angle=saturation;
 }
};
thread_local std::string global_error;
}
extern "C" {
void* apex_create(const char* vehicle,const char* track){try{return new Runtime(vehicle,track);}catch(const std::exception& e){global_error=e.what();return nullptr;}}
const char* apex_rollout(void* ptr,const char* request,int replay){auto& r=*static_cast<Runtime*>(ptr);try{auto j=json::parse(request);r.output=(replay?r.replay(j):r.rollout(j)).dump();return r.output.c_str();}catch(const std::exception& e){r.error=e.what();return nullptr;}}
void apex_destroy(void* ptr){delete static_cast<Runtime*>(ptr);}
const char* apex_error(void* ptr){return ptr?static_cast<Runtime*>(ptr)->error.c_str():global_error.c_str();}
int apex_command(void* ptr,const char* command){auto& r=*static_cast<Runtime*>(ptr);try{r.command(json::parse(command));return 1;}catch(const std::exception& e){r.error=e.what();return 0;}}
int apex_advance(void* ptr,double dt){auto& r=*static_cast<Runtime*>(ptr);try{r.advance(dt);return 1;}catch(const std::exception& e){r.error=e.what();return 0;}}
const char* apex_snapshot(void* ptr,int metadata){auto& r=*static_cast<Runtime*>(ptr);try{r.output=(metadata?r.metadata():r.snapshot()).dump();return r.output.c_str();}catch(const std::exception& e){r.error=e.what();return nullptr;}}
}

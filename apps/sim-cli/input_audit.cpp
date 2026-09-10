#include "apexlab/dynamic_vehicle.hpp"
#include "../sim-server/input_adapter.hpp"
#include "nlohmann/json.hpp"
#include <iostream>
using namespace apexlab;using namespace apexlab::rigid;using json=nlohmann::json;
int main(int argc,char** argv){if(argc!=2)return 1;DynamicVehicleModel model(load_dynamic_vehicle(argv[1]));json cases=json::array();
 for(double speed:{15.,30.,50.})for(const std::string mode:{"old-keyboard","friendly-keyboard","physical-step"}){
 auto state=model.initial(speed);state.resources_enabled=false;DynamicControl u;u.aero_mode=1;state.aero_position=1;KeyboardInput input;KeyboardParameters params;double delta=0;json rows=json::array();
 double peak_yaw=0,peak_beta=0,peak_slip=0,peak_util=0,peak_g=0,saturation_time=-1;
 for(int i=0;i<2000;i++){double t=i*.002;double key=t>=1&&t<1.2?1.:0.;if(mode=="physical-step")key=t>=1?1.:0.;
 const auto v=inverse_rotate(state.contact.body_to_world,state.contact.velocity_world_mps);
 double request=mode=="old-keyboard"?key*.65:mode=="physical-step"?key*.02:input.step(key,0,0,norm(state.contact.velocity_world_mps),model.parameters().contact.legacy.planar->wheelbase.value(),.002,params);
 const double previous=delta;delta=KeyboardInput::approach(delta,request,1.4,.002);double demand=.3*(speed-v.x);
 u.pedals={si::radians(delta),std::clamp(demand,0.,1.),std::clamp(-demand,0.,1.)};state=model.step(state,u,plane_surface(),.002);
 const auto e=model.evaluate(state,u,plane_surface());const auto a=inverse_rotate(state.contact.body_to_world,e.contact.derivative.acceleration_world_mps2);double beta=std::atan2(v.y,v.x);json slips=json::array(),utils=json::array();
 for(const auto& w:e.contact.wheel){slips.push_back(w.slip_angle_rad);utils.push_back(w.tire.friction_utilization);peak_slip=std::max(peak_slip,std::abs(w.slip_angle_rad));peak_util=std::max(peak_util,w.tire.friction_utilization);if(saturation_time<0&&w.tire.friction_utilization>=.98)saturation_time=t;}
 peak_yaw=std::max(peak_yaw,std::abs(state.contact.omega_body_radps.z));peak_beta=std::max(peak_beta,std::abs(beta));peak_g=std::max(peak_g,std::abs(a.y)/9.80665);
 if(i%10==0)rows.push_back({{"time",t},{"speed",norm(state.contact.velocity_world_mps)},{"key",key},{"steerCommand",input.steer},{"request",request},{"delta",delta},{"delta_dot",(delta-previous)/.002},{"steering_wheel_equivalent_rad",delta*15},{"yaw",state.contact.omega_body_radps.z},{"beta",beta},{"slip",slips},{"utilization",utils},{"lateral_g",a.y/9.80665}});
 }
 cases.push_back({{"mode",mode},{"entry_speed_mps",speed},{"peak_yaw_radps",peak_yaw},{"peak_beta_rad",peak_beta},{"peak_slip_rad",peak_slip},{"peak_utilization",peak_util},{"peak_lateral_g",peak_g},{"saturation_time_s",saturation_time},{"final_yaw_gain_per_s",state.contact.omega_body_radps.z/.02},{"samples",rows}});
 }
 std::cout<<json({{"cases",cases},{"protocol","flat asphalt; resources frozen; speed feedback; 200 ms key press at t=1; physical-step holds 0.02 rad; 15:1 illustrative steering-wheel ratio, not a measured vehicle parameter"}}).dump(2)<<'\n';}

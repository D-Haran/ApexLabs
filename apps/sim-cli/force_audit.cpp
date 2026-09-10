#include "apexlab/dynamic_vehicle.hpp"
#include "nlohmann/json.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace apexlab;using namespace apexlab::rigid;using json=nlohmann::json;
int main(int argc,char** argv){try{
 if(argc!=2)throw std::invalid_argument("usage: apexlab-force-audit VEHICLE");
 const DynamicVehicleModel model(load_dynamic_vehicle(argv[1]));const auto road=plane_surface();json rows=json::array();
 std::array<double,2> lateral{},yaw{},radius{};
 for(std::size_t side=0;side<2;++side){const double sign=side==0?1.:-1.;auto state=model.initial(15);state.resources_enabled=false;DynamicControl u;u.aero_mode=1;
 double previous_yaw=0,max_transform=0;DynamicTelemetry e;
 for(int i=0;i<15000;++i){const auto body_velocity=inverse_rotate(state.contact.body_to_world,state.contact.velocity_world_mps);
  const double demand=.3*(15-body_velocity.x);u.pedals={si::radians(sign*.035),std::clamp(demand,0.,1.),std::clamp(-demand,0.,1.)};
  if(i==14900)previous_yaw=state.contact.omega_body_radps.z;
  state=model.step(state,u,road,.002);
 }
 e=model.evaluate(state,u,road);const auto left=unit(cross({0,0,1},state.contact.velocity_world_mps));Vec3 tire_force{};
 json tires=json::array();std::size_t wheel_index=0;for(const auto& w:e.contact.wheel){
  const auto reconstructed=add(mul(w.forward_world,w.tire.longitudinal_force.value()),add(mul(w.left_world,w.tire.lateral_force.value()),mul(w.surface.normal,w.normal_force_n)));
  max_transform=std::max(max_transform,norm(sub(reconstructed,w.force_world_n)));tire_force=add(tire_force,w.force_world_n);
  if(sign*w.tire.lateral_force.value()<=0)throw std::runtime_error("tire Fy points away from turn center");
  const auto body_force=inverse_rotate(state.contact.body_to_world,w.force_world_n);
  tires.push_back({{"steering_rad",wheel_index++<2?u.pedals.steering_angle.value():0.},{"body_force_n",{body_force.x,body_force.y,body_force.z}},{"world_force_n",{w.force_world_n.x,w.force_world_n.y,w.force_world_n.z}},{"fx_n",w.tire.longitudinal_force.value()},{"fy_n",w.tire.lateral_force.value()},{"fz_n",w.normal_force_n},{"slip_rad",w.slip_angle_rad}});
 }
 lateral[side]=dot(tire_force,left);yaw[side]=state.contact.omega_body_radps.z;
 radius[side]=norm(state.contact.velocity_world_mps)/std::abs(yaw[side]);
 if(sign*lateral[side]<=0||sign*yaw[side]<=0||std::abs(yaw[side]-previous_yaw)>.002||max_transform>1e-8)throw std::runtime_error("steady-turn force audit failed");
 const double mass=model.parameters().contact.legacy.mass.value();
 const double expected=mass*norm(state.contact.velocity_world_mps)*std::abs(yaw[side]);
 const double centripetal_error=std::abs(std::abs(lateral[side])-expected)/expected;
 if(centripetal_error>.02)throw std::runtime_error("centripetal balance failed");
 rows.push_back({{"centripetal_force_n",expected},{"centripetal_relative_error",centripetal_error},{"turn",side==0?"left":"right"},{"yaw_rate_radps",yaw[side]},{"radius_m",radius[side]},{"world_inward_force_n",sign*lateral[side]},
 {"max_world_transform_error_n",max_transform},{"steady_yaw_rate_change_radps",std::abs(yaw[side]-previous_yaw)},{"tires",tires}});
 }
 if(std::abs(lateral[0]+lateral[1])>1e-5||std::abs(yaw[0]+yaw[1])>1e-8)throw std::runtime_error("mirrored force symmetry failed");
 std::cout<<json({{"passed",true},{"semantics","road force on vehicle at actual contact patch"},{"frame","tire basis projected into local road tangent plane; world normal load"},{"tests",rows}}).dump(2)<<'\n';
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}

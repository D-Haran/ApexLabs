#include "apexlab/dynamic_vehicle.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace apexlab;
void require(bool c,const char* m){if(!c)throw std::runtime_error(m);}
int main(){try{
 const auto root=std::filesystem::path(APEXLAB_SOURCE_DIR)/"configs/vehicles";
 const DynamicVehicleModel m(load_dynamic_vehicle(root/"mcl36-dynamic.json"));
 const auto road=plane_surface();DynamicControl u;u.pedals.throttle=1;
 auto s=m.initial(40);const double fuel=s.fuel_mass_kg,energy=s.battery_energy_j;
 double spent=0,consumed=0;
 for(int k=0;k<2000;++k){const auto e=m.evaluate(s,u,road);
   require(e.fuel_flow_kg_s<=std::min(100.,.009*e.rpm+5.5)/3600.+1e-12,"F1 fuel envelope");
   require(e.hybrid_available_w<=120000,"MGU-K power");
   spent+=e.battery_discharge_w*.002;consumed+=e.fuel_flow_kg_s*.002;s=m.step(s,u,road,.002);
 }
 require(std::abs(fuel-s.fuel_mass_kg-consumed)<1e-10,"fuel conservation");
 require(std::abs(energy-s.battery_energy_j-spent)<1e-5,"energy conservation");
 require(spent>0&&consumed>0,"resources do not affect moving vehicle");
 auto depleted=s;depleted.battery_energy_j=0;depleted.fuel_mass_kg=0;
 require(m.evaluate(depleted,u,road).drive_force_n==0,"empty resources still provide power");
 depleted=s;depleted.deployed_lap_j=4e6;
 require(m.evaluate(depleted,u,road).hybrid_available_w==0,"deployment lap cap");
 m.begin_lap(depleted);require(depleted.deployed_lap_j==0&&depleted.battery_energy_j==s.battery_energy_j,"lap reset loses SOC");
 s.battery_energy_j=2e6;u.pedals.throttle=0;u.pedals.brake=.5;
 const double before=s.battery_energy_j;double recovered=0;
 for(int k=0;k<100;++k){auto e=m.evaluate(s,u,road);require(e.battery_charge_w<=120000,"recovery power cap");recovered+=e.battery_charge_w*.002;s=m.step(s,u,road,.002);}
 require(recovered>0&&std::abs(s.battery_energy_j-before-recovered)<1e-6,"recovery conservation");
 s.recovered_lap_j=2e6;require(m.evaluate(s,u,road).battery_charge_w==0,"recovery lap cap");
 s=m.initial(30);u={};u.pedals.steering_angle=si::radians(.08);const auto e=m.evaluate(s,u,road);
 require(e.friction_heat_w[0]>0,"slip work does not heat tread");
 auto worn=s;worn.wear.fill(1);auto cold=s;cold.tread_c.fill(0);
 require(m.evaluate(worn,u,road).operating.friction_scale[0]<e.operating.friction_scale[0],"wear does not reduce grip");
 require(m.evaluate(cold,u,road).operating.friction_scale[0]<e.operating.friction_scale[0],"cold tire does not reduce grip");
 s=m.initial();s.contact.position_world_m.z+=10;const auto initial_t=s.tread_c[0];
 for(int k=0;k<50;++k)s=m.step(s,{},road,.002);
 require(s.tread_c[0]<initial_t&&s.wear[0]==0,"airborne cooling/wear");
 s=m.initial(20);auto t=s;
 for(int k=0;k<500;++k)s=m.step(s,u,road,.002);
 for(int k=0;k<1000;++k)t=m.step(t,u,road,.001);
 require(rigid::norm(rigid::sub(s.contact.position_world_m,t.contact.position_world_m))<.03,"resource timestep convergence");
 require(std::abs(s.tread_c[0]-t.tread_c[0])<.02,"thermal timestep convergence");
 require(s.wear[0]>0&&s.wear[0]<=1,"wear bounds");
 std::cout<<"Resource conservation, limits, thermal feedback and timestep tests passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}

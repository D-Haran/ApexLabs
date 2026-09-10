#include "apexlab/dynamic_vehicle.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace apexlab;
void require(bool c,const char* message){if(!c)throw std::runtime_error(message);}
int main(){try{
    const auto root=std::filesystem::path(APEXLAB_SOURCE_DIR)/"configs/vehicles";
    const DynamicVehicleModel p1(load_dynamic_vehicle(root/"mclaren-p1-dynamic.json")),f1(load_dynamic_vehicle(root/"mcl36-dynamic.json"));
    require(p1.parameters().gears.size()==7&&f1.parameters().gears.size()==8,"vehicle-specific gearboxes");
    require(p1.parameters().contact.legacy.mass.value()>f1.parameters().contact.legacy.mass.value(),"vehicle-specific mass");
    require(f1.parameters().get("speed_limiter_mps")==0,"F1 must not have a fixed speed cap");
    const auto road=plane_surface();DynamicControl u;u.pedals.throttle=1;
    auto a=p1.initial(70),b=f1.initial(70);a.aero_position=b.aero_position=1;a.resources_enabled=b.resources_enabled=false;
    const auto pa=p1.evaluate(a,u,road),fb=f1.evaluate(b,u,road);
    require(fb.downforce_n>pa.downforce_n*2,"F1 aero must differ materially");
    b.drs_position=1;const auto drs=f1.evaluate(b,u,road);
    require(drs.downforce_n<fb.downforce_n&&drs.drag_n<fb.drag_n,"DRS must alter aero forces");
    require(drs.front_aero_fraction>fb.front_aero_fraction,"rear DRS balance migration");
    b.contact.position_world_m.z+=.15;require(f1.evaluate(b,u,road).downforce_n<drs.downforce_n,"floor ride-height response");
    b=f1.initial(70);b.aero_position=1;b.contact.body_to_world=rigid::from_rotation_vector({0,.01,0});
    require(std::abs(f1.evaluate(b,u,road).front_aero_fraction-fb.front_aero_fraction)>1e-4,"pitch balance migration");
    a=p1.initial(10);a.contact.position_world_m.z+=5;
    for(const auto& w:p1.evaluate(a,u,road).contact.wheel)require(w.normal_force_n==0&&rigid::norm(w.force_world_n)==0,"powered airborne tire force");
    b=f1.initial(25);u.drs=true;
    for(int i=0;i<500;++i)b=f1.step(b,u,road,.002);
    require(b.drs_position>.9,"DRS state does not respond");
    u.pedals.brake=1;u.pedals.throttle=0;
    for(int i=0;i<500;++i)b=f1.step(b,u,road,.002);
    require(b.drs_position<.01,"braking does not close DRS");
    require(rigid::finite(b.contact.position_world_m),"nonfinite driven model");
    std::cout<<"Vehicle-specific powertrain/aero/contact tests passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}

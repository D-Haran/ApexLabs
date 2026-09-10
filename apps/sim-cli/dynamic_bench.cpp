#include "apexlab/dynamic_vehicle.hpp"
#include "nlohmann/json.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <cmath>
using namespace apexlab;
using namespace apexlab::rigid;
int main(int argc,char** argv) {
 try {
    if(argc<2||argc>3)throw std::invalid_argument("usage: apexlab-dynamic-bench VEHICLE_JSON [DT]");
    const double dt=argc==3?std::stod(argv[2]):.002;
    const DynamicVehicleModel model(load_dynamic_vehicle(argv[1]));const auto road=plane_surface();
    const auto start=std::chrono::steady_clock::now();
    DynamicControl input;input.aero_mode=0;input.pedals.throttle=1;
    auto s=model.initial();s.resources_enabled=false;double t100=-1,t200=-1,maximum=0;
    // Top-speed run is independent of the acceleration crossing interpolation.
    for(int k=0;k<static_cast<int>(90/dt);++k) {
        const auto before=s;s=model.step(s,input,road,dt);
        const double v=s.contact.velocity_world_mps.x,old=before.contact.velocity_world_mps.x;
        if(t100<0&&v>=100/3.6)t100=k*dt+dt*(100/3.6-old)/(v-old);
        if(t200<0&&v>=200/3.6)t200=k*dt+dt*(200/3.6-old)/(v-old);
        maximum=std::max(maximum,v);
        if(!finite(s.contact.position_world_m)||std::abs(s.contact.position_world_m.z)>3)
            throw std::runtime_error("acceleration left physical operating envelope");
    }
    nlohmann::json result={{"name",model.parameters().name},{"dt_s",dt},{"zero_to_100_s",t100},
        {"zero_to_200_s",t200},{"top_speed_kph",maximum*3.6},{"final_gear",s.gear}};
    for(const double speed:{100.,200.}) {
        s=model.initial(speed/3.6);s.resources_enabled=false;input.pedals.throttle=0;input.pedals.brake=0;
        // Settle the aero/suspension at constant initial speed on a rolling bench;
        // only the subsequent unrestrained braking distance is measured.
        for(int k=0;k<static_cast<int>(2/dt);++k){s=model.step(s,input,road,dt);s.contact.velocity_world_mps.x=speed/3.6;s.contact.position_world_m.x=0;}
        input.pedals.brake=1;double distance=-1;
        for(int k=0;k<static_cast<int>(15/dt);++k) {
            const auto before=s;s=model.step(s,input,road,dt);
            if(s.contact.velocity_world_mps.x<.05){distance=s.contact.position_world_m.x;break;}
            if(k==static_cast<int>(15/dt)-1)throw std::runtime_error("braking failed to stop");
            (void)before;
        }
        result[speed==100?"brake_100_m":"brake_200_m"]=distance;
    }
    s=model.initial(70);s.resources_enabled=false;input={};s.aero_position=1;
    auto a=model.evaluate(s,input,road);s.drs_position=1;auto b=model.evaluate(s,input,road);
    s.contact.position_world_m.z+=.15;auto high=model.evaluate(s,input,road);
    result["aero_70mps"]={{"downforce_n",a.downforce_n},{"drag_n",a.drag_n},{"front_fraction",a.front_aero_fraction},
        {"drs_downforce_n",b.downforce_n},{"drs_drag_n",b.drag_n},{"drs_front_fraction",b.front_aero_fraction},{"raised_body_downforce_n",high.downforce_n}};
    result["wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    std::cout<<result.dump(2)<<'\n';
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}

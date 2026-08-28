// Deterministic physical validation intervals. They are not completed circuit laps.
#include "apexlab/lap_simulation.hpp"
#include "replay_export.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace apexlab;
int main(int argc,char** argv) {
 try {
    if(argc!=3)throw std::invalid_argument("usage: apexlab-chassis-scenes VEHICLE OUTPUT_DIRECTORY");
    auto p=load_vehicle_parameters(argv[1]);
    for(const std::string name:{"flat","uphill","downhill","left","right","braking","acceleration"}) {
        const double grade=name=="uphill"?std::tan(10*std::acos(-1)/180):name=="downhill"?-std::tan(10*std::acos(-1)/180):0;
        const double k=name=="left"?1./70:name=="right"?-1./70:0;
        auto parameters=p;
        if(name=="flat"||name=="uphill"||name=="downhill") {parameters.aero={};parameters.tires.rolling_resistance_coefficient=0;}
        RoadVehicleModel model(parameters);
        const auto frame=[&](const PlanarState& state){
            TrackFrame f;f.s_m=k==0?state.position_x.value():std::atan2(k*state.position_x.value(),1-k*state.position_y.value())/k;
            f.position={k==0?f.s_m:std::sin(k*f.s_m)/k,k==0?0:(1-std::cos(k*f.s_m))/k};f.heading_rad=k*f.s_m;f.grade=grade;f.elevation_m=f.s_m*grade;f.curvature_1_m=k;f.left_width_m=f.right_width_m=6;
            f.tangent={std::cos(f.heading_rad),std::sin(f.heading_rad)};f.left_normal={-f.tangent.y,f.tangent.x};return f;
        };
        const auto control=[&](const PlanarState& state,double t){
            const auto f=frame(state);const double error=-(state.position_x.value()-f.position.x)*std::sin(f.heading_rad)+(state.position_y.value()-f.position.y)*std::cos(f.heading_rad);
            const double ramp=std::min(1.,t),demand=ramp*ramp*(3-2*ramp);
            const double speed_command=k!=0?.2*(20-state.velocity_x.value()):0;
            return PlanarControl{si::radians(std::atan(p.planar->wheelbase.value()*k)-.045*error-.85*(state.yaw.value()-f.heading_rad)),
                name=="acceleration"?.3*demand:std::clamp(speed_command,0.,1.),name=="braking"?.18*demand:std::clamp(-speed_command,0.,1.)};
        };
        RoadVehicleState state;state.planar.velocity_x=si::meters_per_second(name=="braking"?28:20);
        LapSimulationResult result;constexpr double dt=.005,duration=6,length=300;
        for(std::uint64_t step=0;step<=1200;++step) {
            const double t=static_cast<double>(step)*dt;const auto f=frame(state.planar);const auto input=control(state.planar,t);const auto force=model.forces(state,input,f);
            LapTelemetrySample sample;sample.time=si::seconds(t);sample.step=step;sample.state=state.planar;sample.input=input;sample.forces=force.tires;sample.speed_mps=std::hypot(state.planar.velocity_x.value(),state.planar.velocity_y.value());sample.longitudinal_acceleration_mps2=force.tires.body_longitudinal_force.value()/p.mass.value();sample.lateral_acceleration_mps2=force.tires.body_lateral_force.value()/p.mass.value();sample.track_s_m=sample.unwrapped_track_s_m=f.s_m;sample.track_progress_fraction=f.s_m/length;sample.heading_error_rad=state.planar.yaw.value()-f.heading_rad;sample.lateral_error_m=-(state.planar.position_x.value()-f.position.x)*std::sin(f.heading_rad)+(state.planar.position_y.value()-f.position.y)*std::cos(f.heading_rad);sample.reference_curvature_1_m=k;sample.target_speed_mps=20;sample.speed_error_mps=20-sample.speed_mps;sample.lap_elapsed_time_s=t;sample.on_track=std::abs(sample.lateral_error_m)<=6;sample.chassis=state.body;sample.suspension_compression_m=force.compression_m;sample.gravity_body=force.gravity_body;result.samples.push_back(sample);
            const auto derivative=[&](const RoadVehicleState& s,Time time){return model.derivative(s,control(s.planar,time.value()),frame(s.planar));};
            state=integrate_rk4(state,si::seconds(t),si::seconds(dt),derivative);
        }
        // Reuse the recorded-interval container/export contract; metadata marks this as a scene,
        // and UI labels elapsed duration rather than claiming a physically completed lap.
        result.completed=true;result.completed_lap_times_s={duration};result.completed_sector_times_s={duration};
        const auto output=std::filesystem::path(argv[2])/name;std::filesystem::create_directories(output);
        write_lap_telemetry_csv(output/"telemetry.csv",result.samples);
        const PeriodicTrack extent(make_straight_test_path(length));LapSimulationOptions options;options.warmup_laps=0;options.timed_laps=1;
        write_replay_data(output,result,extent,options);
        std::ofstream geometry(output/"geometry.csv");geometry<<std::setprecision(17)<<"s_m,x_m,y_m,heading_rad,curvature_1_m,left_width_m,right_width_m,left_x_m,left_y_m,right_x_m,right_y_m,elevation_m,grade\n";
        for(int i=0;i<=600;++i){const double s=static_cast<double>(i)*.5,x=k==0?s:std::sin(k*s)/k,y=k==0?0:(1-std::cos(k*s))/k,h=k*s;
            geometry<<s<<','<<x<<','<<y<<','<<h<<','<<k<<",6,6,"<<x-6*std::sin(h)<<','<<y+6*std::cos(h)<<','<<x+6*std::sin(h)<<','<<y-6*std::cos(h)<<','<<s*grade<<','<<grade<<'\n';}
        std::cout<<name<<" completed physical interval; "<<result.samples.size()<<" samples\n";
    }
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}

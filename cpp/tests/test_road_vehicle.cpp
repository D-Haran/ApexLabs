#include "apexlab/road_vehicle_model.hpp"
#include "apexlab/integrator.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
using namespace apexlab;
void require(bool condition,const char* message) {if (!condition) throw std::runtime_error(message);}
void near(double a,double b,double tol,const char* message) {require(std::abs(a-b)<tol,message);}
int main(int argc,char** argv) {
 try {
    auto p=load_vehicle_parameters(std::filesystem::path(APEXLAB_SOURCE_DIR)/"configs/vehicles/mclaren-p1-sprung-approx.json");
    RoadVehicleModel model(p);RoadVehicleState rest;TrackFrame flat;flat.tangent={1,0};
    const auto equilibrium=model.forces(rest,{},flat);
    near(equilibrium.body_derivative.heave_rate,0,1e-10,"static heave");
    near(equilibrium.body_derivative.roll_rate,0,1e-10,"static roll");
    near(equilibrium.body_derivative.pitch_rate,0,1e-10,"static pitch");
    double support=0;for(const auto& w:equilibrium.tires.wheel)support+=w.normal_load.value();
    near(support,p.mass.value()*9.80665,1e-8,"static normal loads");
    // No drivetrain/resistance: grade acceleration must equal the analytical gravity component.
    auto coast=p;coast.aero={};coast.tires.rolling_resistance_coefficient=0;coast.powertrain.maximum_drive_force=si::newtons(0);
    RoadVehicleModel grade_model(coast);
    for(const double deg:{-10.,-5.,0.,5.,10.}) {
        TrackFrame road=flat;road.grade=std::tan(deg*std::acos(-1)/180);road.heading_rad=.7;
        rest.planar.yaw=si::radians(.7);rest.planar.velocity_x=si::meters_per_second(20);
        const auto d=grade_model.derivative(rest,{},road);const auto f=road_frame_3d(road);
        near(d.planar.velocity_x_rate.value(),-9.80665*std::sin(deg*std::acos(-1)/180),1e-10,"grade acceleration");
        near(f.tangent.x*f.normal.x+f.tangent.y*f.normal.y+f.tangent.z*f.normal.z,0,1e-12,"orthogonal T/N");
        near(f.tangent.x*f.tangent.x+f.tangent.y*f.tangent.y+f.tangent.z*f.tangent.z,1,1e-12,"unit tangent");
    }
    // Force-controlled bench isolates suspension dynamics; actual lap mode couples production tires.
    const auto destination=argc>1?std::filesystem::path(argv[1]):std::filesystem::temp_directory_path()/"apexlab-body-validation.csv";
    if(destination.has_parent_path())std::filesystem::create_directories(destination.parent_path());
    std::ofstream out(destination);out<<std::setprecision(17)<<"scenario,dt,time,heave,roll,pitch,heave_rate,roll_rate,pitch_rate,fl,fr,rl,rr,fx,fy,down,gn\n";
    auto symmetric=p;
    symmetric.planar->cg_to_front_axle=si::meters(p.planar->wheelbase.value()/2);
    symmetric.planar->cg_to_rear_axle=symmetric.planar->cg_to_front_axle;
    for(auto& c:symmetric.sprung_body->corners){c.spring_rate_n_m=60000;c.compression_damping_ns_m=2000;c.rebound_damping_ns_m=2000;}
    RoadVehicleModel symmetric_model(symmetric);
    for(const double dt:{.01,.005,.0025,.00125}) {
        for(int scenario=0;scenario<9;++scenario) {
            const auto& bench = scenario==8?symmetric_model:model;
            RoadVehicleState state; if(scenario==5 || scenario==8)state.body.heave=.01;
            const double fx=scenario==1?-6000:scenario==2?6000:0;
            const double fy=scenario==3?6000:scenario==4?-6000:0;
            const double down=scenario==6?2000:0;
            const double gn=scenario==7?9.80665*std::cos(10*std::acos(-1)/180):9.80665;
            for(int step=0;step<=static_cast<int>(10/dt);++step) {
                const double t=step*dt;const auto loads=bench.suspension_loads(state.body);
                out<<scenario<<','<<dt<<','<<t<<','<<state.body.heave<<','<<state.body.roll<<','<<state.body.pitch<<','<<state.body.heave_rate<<','<<state.body.roll_rate<<','<<state.body.pitch_rate;
                for(const auto f:loads.wheel)out<<','<<f.value();
                out<<','<<fx<<','<<fy<<','<<down<<','<<gn<<'\n';
                const auto derivative=[&](const RoadVehicleState& s,Time time) {
                    const double ramp=std::min(1.0,time.value()/1.0);const double demand=ramp*ramp*(3-2*ramp);
                    return RoadVehicleDerivative{{},bench.body_derivative(s.body,fx*demand,fy*demand,down*demand,gn)};
                };
                state=integrate_rk4(state,si::seconds(t),si::seconds(dt),derivative);
            }
            if(scenario==1)require(state.body.pitch<0,"braking must lower nose");
            if(scenario==2)require(state.body.pitch>0,"acceleration must raise nose");
            if(scenario==3)require(state.body.roll>0,"left turn must load right side");
            if(scenario==4)require(state.body.roll<0,"right turn must load left side");
            if(scenario==5)near(state.body.heave,0,1e-8,"damped heave decay");
        }
    }
    // Invalid configurations/contact loads must fail, never be silently clipped.
    bool rejected=false;try{auto s=SprungState{};s.heave=1;(void)model.suspension_loads(s);}catch(const std::domain_error&){rejected=true;}
    require(rejected,"contact envelope failure");
    std::cout<<"Road/sprung body analytical tests passed; CSV="<<destination<<'\n';
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}

// Reproducible contact experiments, deliberately separate from completed-lap data.
#include "apexlab/contact_vehicle_model.hpp"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string>
using namespace apexlab;
using namespace apexlab::rigid;
namespace {
constexpr std::array<const char*,4> names{"fl","fr","rl","rr"};
void vec(std::ostream& out,Vec3 p){out<<','<<p.x<<','<<p.y<<','<<p.z;}
void header(std::ostream& out) {
    out<<"time_s,x_m,y_m,z_m,qw,qx,qy,qz,vx_mps,vy_mps,vz_mps,wx_radps,wy_radps,wz_radps,ax_mps2,ay_mps2,az_mps2,com_x_m,com_y_m,com_z_m,com_vx_mps,com_vy_mps,com_vz_mps,kinetic_j,potential_j,equation_residual,road_height_m,steering_rad,throttle,brake";
    for(const auto name:names)for(const auto field:{"center_x","center_y","center_z","patch_x","patch_y","patch_z","force_x","force_y","force_z","extension","extension_rate","tire_compression","suspension_compression","damper_force","fz","fx","fy","slip","utilization","contact","material"})out<<','<<name<<'_'<<field;
    out<<'\n';
}
void row(std::ostream& out,double time,const ContactVehicleState& s,const ContactVehicleEvaluation& e,
    const RoadSurfaceQuery& road,PlanarControl input) {
    out<<time;vec(out,s.position_world_m);
    out<<','<<s.body_to_world.w<<','<<s.body_to_world.x<<','<<s.body_to_world.y<<','<<s.body_to_world.z;
    vec(out,s.velocity_world_mps);vec(out,s.omega_body_radps);vec(out,e.derivative.acceleration_world_mps2);
    vec(out,e.center_of_mass_world_m);vec(out,e.center_of_mass_velocity_world_mps);
    out<<','<<e.kinetic_energy_j<<','<<e.potential_energy_j<<','<<e.max_equation_residual_n<<','<<road(s.position_world_m).position.z
       <<','<<input.steering_angle.value()<<','<<input.throttle<<','<<input.brake;
    for(std::size_t i=0;i<4;++i) {
        const auto& w=e.wheel[i];vec(out,w.center_world_m);vec(out,w.patch_world_m);vec(out,w.force_world_n);
        out<<','<<s.extension_m[i]<<','<<s.extension_rate_mps[i]<<','<<w.compression_m<<','<<w.suspension_compression_m
           <<','<<w.damper_force_n<<','<<w.normal_force_n<<','<<w.tire.longitudinal_force.value()<<','<<w.tire.lateral_force.value()
           <<','<<w.slip_angle_rad<<','<<w.tire.friction_utilization<<','<<w.contacting<<','<<static_cast<int>(w.surface.kind);
    }
    out<<'\n';
}
void surface_mesh(const std::filesystem::path& path,const RoadSurfaceQuery& road,double start,double end) {
    std::ofstream out(path);out<<std::setprecision(12)<<"ix,iy,x_m,y_m,z_m,nx,ny,nz,material\n";
    std::vector<double> xs,ys;
    for(double x=start;x<=end+.001;x+=.5)xs.push_back(x);
    for(int i=0;i<=80;++i)ys.push_back(-10+i*.25);
    // Resolve the synthetic curb profile and end ramps to sub-millimetre mesh error.
    for(int i=0;i<=18;++i){ys.push_back(6+i*.05);ys.push_back(-6-i*.05);}
    for(int i=0;i<=20;++i)for(double x:{20+i*.1,78+i*.1})if(x>start&&x<end)xs.push_back(x);
    for(auto* values:{&xs,&ys}) {
        std::sort(values->begin(),values->end());
        values->erase(std::unique(values->begin(),values->end(),[](double a,double b){return std::abs(a-b)<1e-8;}),values->end());
    }
    std::vector<Vec3> positions;
    for(std::size_t ix=0;ix<xs.size();++ix)for(std::size_t iy=0;iy<ys.size();++iy) {
        const auto p=road({xs[ix],ys[iy],0});positions.push_back(p.position);
        out<<ix<<','<<iy;vec(out,p.position);vec(out,p.normal);out<<','<<static_cast<int>(p.kind)<<'\n';
    }
    double max_error=0;
    for(std::size_t x=0;x+1<xs.size();++x)for(std::size_t y=0;y+1<ys.size();++y) {
        const auto i=x*ys.size()+y;
        const auto a=positions[i],b=positions[i+ys.size()],c=positions[i+1],d=positions[i+ys.size()+1];
        for(const auto center:{mul(add(add(a,b),c),1./3),mul(add(add(b,c),d),1./3)})
            max_error=std::max(max_error,std::abs(road(center).position.z-center.z));
    }
    std::ofstream accuracy(path.parent_path()/"surface-accuracy.json");
    accuracy<<std::setprecision(12)<<"{\"maximum_triangle_centroid_height_error_m\":"<<max_error<<",\"vertex_count\":"<<positions.size()<<",\"triangle_count\":"<<(xs.size()-1)*(ys.size()-1)*2<<"}\n";
    if(max_error>.001)throw std::runtime_error("surface render interpolation exceeds 1 mm");
}
}
int main(int argc,char** argv) {
 try {
    if(argc<3||argc>4)throw std::invalid_argument("usage: apexlab-contact-scenes LEGACY_SPRUNG_VEHICLE OUTPUT_DIRECTORY [dt_s]");
    const double dt=argc==4?std::stod(argv[3]):.002;
    if(!std::isfinite(dt)||dt<=0||dt>.002)throw std::invalid_argument("experiment dt must be in (0,.002]");
    const auto output=std::filesystem::path(argv[2]);std::filesystem::create_directories(output);
    auto p=estimated_contact_parameters(load_vehicle_parameters(argv[1]));p.legacy.aero={};
    const ContactVehicleModel model(p);
    std::ofstream config(output/"parameters.json");config<<std::setprecision(12);
    config<<"{\"schema_version\":1,\"model\":\"contact_3d\",\"provenance\":\"Engineering estimates from the preserved P1 sprung-body fixture; no vehicle calibration claimed. Aero and rolling resistance disabled for mechanical benches.\",\"sprung_mass_kg\":"<<p.sprung_mass_kg<<",\"total_mass_kg\":"<<p.legacy.mass.value()<<",\"cg_height_m\":"<<p.legacy.nonlinear_planar->cg_height.value()<<",\"corner\":[";
    for(std::size_t i=0;i<4;++i) {
        const auto& c=p.corner[i];if(i)config<<',';
        config<<"{\"name\":\""<<names[i]<<"\",\"parameter_class\":\"estimated\",\"unsprung_mass_kg\":"<<c.unsprung_mass_kg
              <<",\"radius_m\":"<<c.radius_m<<",\"tire_stiffness_n_m\":"<<c.tire_stiffness_n_m<<",\"tire_damping_ns_m\":"<<c.tire_damping_ns_m
              <<",\"spring_n_m\":"<<c.spring_n_m<<",\"compression_damping_ns_m\":"<<c.compression_damping_ns_m<<",\"rebound_damping_ns_m\":"<<c.rebound_damping_ns_m
              <<",\"rest_extension_m\":"<<c.rest_extension_m<<",\"min_extension_m\":"<<c.min_extension_m<<",\"max_extension_m\":"<<c.max_extension_m
              <<",\"stop_stiffness_n_m\":"<<c.stop_stiffness_n_m<<",\"stop_damping_ns_m\":"<<c.stop_damping_ns_m<<'}';
    }
    config<<"]}\n";
    std::ofstream runs(output/"runs.csv");runs<<"scene,initial_speed_mps,dt_s,steps,wall_s,crest_radius_m,nominal_crest_demand_mps2\n"<<std::setprecision(12);
    const PeriodicTrack straight(make_straight_test_path(160));
    RoadSurfaceParameters sp;sp.curbs={{20,80,1}};const TrackRoadSurface strip(straight,sp);
    for(const std::string name:{"flat","drop","curb","grass","crest-12","crest-20","crest-28","crest-36","crest-40","crest-44"}) {
        const bool is_crest=name.starts_with("crest-");
        const double speed=is_crest?std::stod(name.substr(6)):name=="drop"?0:12;
        RoadSurfaceQuery surface=is_crest?crest_surface(2,35):name=="curb"||name=="grass"?RoadSurfaceQuery([&strip](Vec3 point){return strip(point);}):plane_surface();
        RoadSurfaceQuery road=[surface,name](Vec3 point){auto r=surface(point);if(name!="grass"&&name!="curb")r.material.rolling_resistance=0;return r;};
        auto s=model.flat_equilibrium(speed);s.position_world_m.x=is_crest?-45:0;
        if(name=="drop")s.position_world_m.z+=.5;
        if(name=="curb")s.position_world_m.y=5.6;
        if(name=="grass")s.position_world_m.y=10.5;
        const double duration=is_crest?140/speed:6;
        const int count=static_cast<int>(std::ceil(duration/dt));
        const auto dir=output/name;std::filesystem::create_directories(dir);
        std::ofstream out(dir/"contact.csv");out<<std::setprecision(12);header(out);
        const auto started=std::chrono::steady_clock::now();
        for(int k=0;k<=count;++k) {
            const PlanarControl input{};const auto e=model.evaluate(s,input,road);
            row(out,k*dt,s,e,road,input);
            if(k<count)s=model.step(s,input,road,dt);
        }
        const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
        constexpr double crest_radius=35.*35./(2*std::numbers::pi*std::numbers::pi);
        runs<<name<<','<<speed<<','<<dt<<','<<count<<','<<seconds<<','<<(is_crest?crest_radius:0)<<','<<(is_crest?speed*speed/crest_radius:0)<<'\n';
        surface_mesh(dir/"surface.csv",road,is_crest?-50:0,is_crest?105:100);
        std::cout<<name<<": "<<count<<" steps, "<<seconds<<" s wall\n";
    }
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}

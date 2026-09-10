#include "apexlab/dynamic_vehicle.hpp"
#include "nlohmann/json.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <numbers>
#include <set>
#include <stdexcept>
namespace apexlab {
using namespace rigid;
DynamicVehicleParameters load_dynamic_vehicle(const std::filesystem::path& path) {
    std::ifstream file(path);if(!file)throw std::runtime_error("cannot open dynamic vehicle: "+path.string());
    const auto j=nlohmann::json::parse(file);
    if(j.at("schema_version")!=1||j.at("model")!="dynamic_contact")throw std::invalid_argument("unsupported dynamic vehicle schema");
    DynamicVehicleParameters p;p.name=j.at("name");p.family=j.at("family");
    if(p.family!="p1"&&p.family!="f1_2022")throw std::invalid_argument("unsupported aero/powertrain family");
    const std::filesystem::path base=j.at("base_vehicle").get<std::string>();
    if(base.is_absolute()||base.has_parent_path())throw std::invalid_argument("base vehicle must be sibling filename");
    auto legacy=load_vehicle_parameters(path.parent_path()/base);
    p.values=j.at("values").get<std::map<std::string,double>>();p.gears=j.at("gears").get<std::vector<double>>();
    const std::set<std::string> classes{"published","derived","calibrated","estimated","unknown"};
    for(const auto& [key,value]:p.values) {
        if(!std::isfinite(value)||!classes.contains(j.at("parameter_classes").at(key).get<std::string>()))
            throw std::invalid_argument("invalid dynamic parameter/provenance: "+key);
    }
    if(!classes.contains(j.at("parameter_classes").at("gears").get<std::string>()))throw std::invalid_argument("gear provenance missing");
    if(p.gears.size()!=(p.family=="p1"?7u:8u))throw std::invalid_argument("wrong gear count");
    for(std::size_t i=0;i<p.gears.size();++i)if(!std::isfinite(p.gears[i])||p.gears[i]<=0||(i&&p.gears[i]>=p.gears[i-1]))throw std::invalid_argument("invalid gear ratios");
    for(const auto key:{"ice_power_w","hybrid_power_w","peak_torque_nm","final_drive","efficiency","max_rpm","shift_s","brake_force_n","tire_mu","road_cda_m2","race_cda_m2","road_cla_m2","race_cla_m2","front_aero_fraction","nominal_floor_height_m","floor_cla_m2","floor_optimal_height_m","floor_width_m","drs_drag_reduction","drs_rear_downforce_reduction","aero_response_s","speed_limiter_mps"}) {
        const auto v=p.get(key);if(v<0)throw std::invalid_argument(std::string("negative parameter: ")+key);
    }
    if(p.get("efficiency")<=0||p.get("efficiency")>1||p.get("max_rpm")<=1000||p.get("final_drive")<=0||
       p.get("aero_response_s")<=0||p.get("floor_width_m")<=0||p.get("tire_mu")<=0||
       p.get("front_aero_fraction")<=0||p.get("front_aero_fraction")>=1||
       p.get("drs_drag_reduction")>=1||p.get("drs_rear_downforce_reduction")>=1)
        throw std::invalid_argument("dynamic parameter outside physical domain");
    for(const auto key:{"fuel_lhv_j_kg","engine_thermal_efficiency","battery_capacity_j","motor_efficiency","tire_temperature_width_c","tread_heat_capacity_j_k","carcass_heat_capacity_j_k","wear_energy_j"})
        if(p.get(key)<=0)throw std::invalid_argument(std::string("resource parameter must be positive: ")+key);
    for(const auto key:{"initial_fuel_kg","reference_fuel_kg","idle_fuel_kg_s","initial_battery_j","recovery_efficiency","recovery_power_w","deploy_lap_limit_j","recover_lap_limit_j","tread_carcass_conductance_w_k","air_conductance_w_k","air_speed_conductance_w_k_mps","road_conductance_w_k","slip_heat_fraction","worn_grip_loss"})
        if(p.get(key)<0)throw std::invalid_argument(std::string("negative resource parameter: ")+key);
    for(const auto key:{"engine_thermal_efficiency","motor_efficiency","recovery_efficiency","slip_heat_fraction","worn_grip_loss"})
        if(p.get(key)>1)throw std::invalid_argument("resource fraction exceeds one");
    if(p.get("initial_battery_j")>p.get("battery_capacity_j"))throw std::invalid_argument("battery over capacity");
    legacy.nonlinear_planar->tire_mu_reference=p.get("tire_mu");
    legacy.brakes.maximum_brake_force=si::newtons(p.get("brake_force_n"));
    p.contact=estimated_contact_parameters(legacy);return p;
}
DynamicVehicleModel::DynamicVehicleModel(DynamicVehicleParameters p):parameters_(std::move(p)),contact_(parameters_.contact) {}
DynamicState DynamicVehicleModel::initial(double speed) const {
    DynamicState s;s.contact=contact_.flat_equilibrium(speed);
    const auto& p=parameters_;
    const double wheel_rpm=std::abs(speed)/p.contact.corner[2].radius_m*60/(2*std::numbers::pi);
    while(s.gear<static_cast<int>(p.gears.size())&&wheel_rpm*p.gears[static_cast<std::size_t>(s.gear-1)]*p.get("final_drive")>p.get("max_rpm")*.92)++s.gear;
    s.fuel_mass_kg=p.get("initial_fuel_kg");s.battery_energy_j=p.get("initial_battery_j");
    s.tread_c.fill(p.get("initial_tire_temperature_c"));s.carcass_c=s.tread_c;
    return s;
}
DynamicTelemetry DynamicVehicleModel::operating(const DynamicState& s,DynamicControl u,const RoadSurfaceQuery& road) const {
    const auto& p=parameters_;const auto& chassis=p.contact.legacy;
    if(s.gear<1||s.gear>static_cast<int>(p.gears.size()))throw std::invalid_argument("invalid selected gear");
    DynamicTelemetry out;
    const auto forward=rotate(s.contact.body_to_world,{1,0,0}),up=rotate(s.contact.body_to_world,{0,0,1});
    const double speed=std::max(0.,dot(s.contact.velocity_world_mps,forward));
    const double ratio=p.gears[static_cast<std::size_t>(s.gear-1)]*p.get("final_drive");
    out.rpm=std::max(p.family=="p1"?900.:4000.,speed/p.contact.corner[2].radius_m*ratio*60/(2*std::numbers::pi));
    out.ice_available_w=p.get("ice_power_w");
    out.hybrid_available_w=p.get("hybrid_power_w")*std::clamp(u.deployment,0.,1.);
    if(s.resources_enabled) {
        const double fuel_limit=p.family=="f1_2022"?std::min(100.,.009*out.rpm+5.5)/3600.:1.;
        out.ice_available_w=std::min(out.ice_available_w,std::min(fuel_limit,std::max(0.,s.fuel_mass_kg)/.002)*p.get("fuel_lhv_j_kg")*p.get("engine_thermal_efficiency"));
        const double available=std::min(std::max(0.,s.battery_energy_j),std::max(0.,p.get("deploy_lap_limit_j")-s.deployed_lap_j));
        out.hybrid_available_w=std::min(out.hybrid_available_w,available/.002*p.get("motor_efficiency"));
    }
    const double power=out.ice_available_w+out.hybrid_available_w;
    const double torque_force=p.get("peak_torque_nm")*std::min(1.,power/(p.get("ice_power_w")+p.get("hybrid_power_w")))*ratio*p.get("efficiency")/p.contact.corner[2].radius_m;
    const double power_force=power*p.get("efficiency")/std::max(speed,1.);
    const double rev_cut=std::clamp((p.get("max_rpm")+200-out.rpm)/200,0.,1.);
    double limiter=1;
    if(p.get("speed_limiter_mps")>0)limiter=std::clamp((p.get("speed_limiter_mps")+.5-speed)/.5,0.,1.);
    out.drive_force_n=std::min(torque_force,power_force)*std::clamp(u.pedals.throttle,0.,1.)*rev_cut*limiter*(s.shift_remaining_s>0?0.:1.);
    out.wheel_power_w=out.drive_force_n*speed;
    const double aero=std::clamp(s.aero_position,0.,1.),drs=std::clamp(s.drs_position,0.,1.);
    out.cda_m2=p.get("road_cda_m2")+(p.get("race_cda_m2")-p.get("road_cda_m2"))*aero;
    out.cla_m2=p.get("road_cla_m2")+(p.get("race_cla_m2")-p.get("road_cla_m2"))*aero;
    out.front_aero_fraction=p.get("front_aero_fraction");
    const auto height=[&](double x){
        const auto point=add(s.contact.position_world_m,rotate(s.contact.body_to_world,{x,0,-chassis.nonlinear_planar->cg_height.value()+p.get("nominal_floor_height_m")}));
        const auto ground=road(point);return dot(sub(point,ground.position),ground.normal);
    };
    out.front_height_m=height(chassis.planar->cg_to_front_axle.value());
    out.rear_height_m=height(-chassis.planar->cg_to_rear_axle.value());
    if(p.family=="f1_2022") {
        const double h=.5*(out.front_height_m+out.rear_height_m);
        const double delta=(h-p.get("floor_optimal_height_m"))/p.get("floor_width_m");
        const double stall=1/(1+std::exp(std::clamp(-(h-.015)/.007,-60.,60.)));
        const double floor=p.get("floor_cla_m2")*std::exp(-delta*delta)*stall;
        const double floor_front=std::clamp(.45+2*(out.rear_height_m-out.front_height_m),.3,.6);
        const double front=out.cla_m2*out.front_aero_fraction+floor*floor_front;
        const double rear=out.cla_m2*(1-out.front_aero_fraction)*(1-drs*p.get("drs_rear_downforce_reduction"))+floor*(1-floor_front);
        const double total=out.cla_m2*(1-(1-out.front_aero_fraction)*drs*p.get("drs_rear_downforce_reduction"))+floor;
        out.front_aero_fraction=front/(front+rear);out.cla_m2=total;
        out.cda_m2*=1-drs*p.get("drs_drag_reduction");
    } else {
        // Estimated braking airbrake surrogate, not OEM active-aero logic.
        out.cda_m2+=.12*std::clamp(u.pedals.brake,0.,1.);
        out.cla_m2+=.2*std::clamp(u.pedals.brake,0.,1.);
    }
    const double pressure=.5*chassis.aero.air_density_kgpm3*speed*speed;
    out.drag_n=pressure*out.cda_m2;out.downforce_n=pressure*out.cla_m2;
    if(p.family=="p1")out.downforce_n=std::min(out.downforce_n,600*9.80665);
    auto& operating=out.operating;operating.override_forces=true;
    operating.drive_force_n=out.drive_force_n;operating.brake_force_n=std::clamp(u.pedals.brake,0.,1.)*p.get("brake_force_n");
    operating.aero_force_world_n=add(mul(forward,-out.drag_n),mul(up,-out.downforce_n));
    const double aero_x=out.front_aero_fraction*chassis.planar->cg_to_front_axle.value()-(1-out.front_aero_fraction)*chassis.planar->cg_to_rear_axle.value();
    operating.aero_moment_body_nm={0,aero_x*out.downforce_n,0};
    if(s.resources_enabled) {
        operating.extra_sprung_mass_kg=s.fuel_mass_kg-p.get("reference_fuel_kg");
        for(std::size_t i=0;i<4;++i) {
            const double d=(s.tread_c[i]-p.get("tire_optimal_temperature_c"))/p.get("tire_temperature_width_c");
            operating.friction_scale[i]=(.65+.35*std::exp(-d*d))*(1-p.get("worn_grip_loss")*std::clamp(s.wear[i],0.,1.));
        }
    }
    return out;
}
DynamicTelemetry DynamicVehicleModel::evaluate(const DynamicState& s,DynamicControl u,const RoadSurfaceQuery& road) const {
    auto out=operating(s,u,road);out.contact=contact_.evaluate(s.contact,u.pedals,road,out.operating);
    if(!s.resources_enabled)return out;
    const auto& p=parameters_;
    double delivered=0,braking=0;
    for(std::size_t i=0;i<4;++i) {
        const auto& w=out.contact.wheel[i];
        const double vx=dot(w.patch_velocity_world_mps,w.forward_world),vy=dot(w.patch_velocity_world_mps,w.left_world);
        delivered+=std::max(0.,w.tire.longitudinal_force.value()*vx);
        if(i>=2)braking+=std::max(0.,-w.tire.longitudinal_force.value()*vx);
        out.friction_heat_w[i]=std::max(0.,-w.tire.lateral_force.value()*vy)*p.get("slip_heat_fraction");
        out.deformation_heat_w[i]=w.surface.material.rolling_resistance*w.normal_force_n*std::abs(vx);
    }
    const double shaft=std::min(out.wheel_power_w,delivered)/p.get("efficiency");
    const double available=out.ice_available_w+out.hybrid_available_w;
    const double hybrid=available>0?shaft*out.hybrid_available_w/available:0.;
    out.battery_discharge_w=hybrid/p.get("motor_efficiency");
    const double fuel_limit=p.family=="f1_2022"?std::min(100.,.009*out.rpm+5.5)/3600.:1.;
    out.fuel_flow_kg_s=std::min({fuel_limit,std::max(0.,s.fuel_mass_kg)/.002,
        (shaft-hybrid)/(p.get("fuel_lhv_j_kg")*p.get("engine_thermal_efficiency"))+p.get("idle_fuel_kg_s")});
    if(u.pedals.brake>0&&out.battery_discharge_w==0)out.battery_charge_w=std::min({
        braking*p.get("recovery_efficiency"),p.get("recovery_power_w"),
        std::max(0.,p.get("battery_capacity_j")-s.battery_energy_j)/.002,
        std::max(0.,p.get("recover_lap_limit_j")-s.recovered_lap_j)/.002});
    return out;
}
DynamicState DynamicVehicleModel::step(const DynamicState& initial,DynamicControl u,const RoadSurfaceQuery& road,double dt) const {
    if(!std::isfinite(dt)||dt<=0||dt>.05)throw std::invalid_argument("dynamic step outside (0,.05]");
    auto s=initial;const auto& p=parameters_;
    const int count=static_cast<int>(std::ceil(dt/.002));const double h=dt/count;
    for(int k=0;k<count;++k) {
        const auto before=evaluate(s,u,road);
        if(s.shift_remaining_s<=0) {
            if(before.rpm>p.get("max_rpm")*.92&&s.gear<static_cast<int>(p.gears.size())){++s.gear;s.shift_remaining_s=p.get("shift_s");}
            else if(before.rpm<p.get("max_rpm")*.48&&s.gear>1){--s.gear;s.shift_remaining_s=p.get("shift_s");}
        }
        s.contact=contact_.step(s.contact,u.pedals,road,h,[&](const ContactVehicleState& state){auto candidate=s;candidate.contact=state;return operating(candidate,u,road).operating;});
        if(s.resources_enabled) {
            s.fuel_mass_kg=std::max(0.,s.fuel_mass_kg-h*before.fuel_flow_kg_s);
            s.battery_energy_j+=h*(before.battery_charge_w-before.battery_discharge_w);
            s.deployed_lap_j+=h*before.battery_discharge_w;s.recovered_lap_j+=h*before.battery_charge_w;
            const double speed=norm(s.contact.velocity_world_mps);
            for(std::size_t i=0;i<4;++i) {
                const double conduction=p.get("tread_carcass_conductance_w_k")*(s.tread_c[i]-s.carcass_c[i]);
                const double air=(p.get("air_conductance_w_k")+p.get("air_speed_conductance_w_k_mps")*speed)*(s.tread_c[i]-p.get("ambient_temperature_c"));
                const double road_heat=before.contact.wheel[i].contacting?p.get("road_conductance_w_k")*(s.tread_c[i]-p.get("road_temperature_c")):0.;
                s.tread_c[i]+=h*(before.friction_heat_w[i]+.5*before.deformation_heat_w[i]-conduction-air-road_heat)/p.get("tread_heat_capacity_j_k");
                s.carcass_c[i]+=h*(conduction+.5*before.deformation_heat_w[i]-.25*p.get("air_conductance_w_k")*(s.carcass_c[i]-p.get("ambient_temperature_c")))/p.get("carcass_heat_capacity_j_k");
                const double hot=1+std::pow(std::max(0.,s.tread_c[i]-p.get("tire_optimal_temperature_c"))/30.,2);
                s.wear[i]=std::min(1.,s.wear[i]+h*(before.friction_heat_w[i]+before.deformation_heat_w[i])*hot/p.get("wear_energy_j"));
            }
        }
        s.shift_remaining_s=std::max(0.,s.shift_remaining_s-h);
        const double blend=1-std::exp(-h/p.get("aero_response_s"));
        s.aero_position+=((u.aero_mode==1?1.:0.)-s.aero_position)*blend;
        s.drs_position+=((p.family=="f1_2022"&&u.drs&&u.pedals.brake<.01?1.:0.)-s.drs_position)*blend;
    }
    return s;
}
} // namespace apexlab

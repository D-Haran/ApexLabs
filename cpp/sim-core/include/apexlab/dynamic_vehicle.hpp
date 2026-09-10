#pragma once
#include "apexlab/contact_vehicle_model.hpp"
#include <map>
#include <string>
#include <vector>
namespace apexlab {
struct DynamicVehicleParameters {
    std::string name, family;
    ContactVehicleParameters contact;
    std::map<std::string,double> values;
    std::vector<double> gears;
    double get(const std::string& key) const { return values.at(key); }
};
DynamicVehicleParameters load_dynamic_vehicle(const std::filesystem::path& path);
struct DynamicControl {
    PlanarControl pedals;
    bool drs{false};
    int aero_mode{1}; // 0 road, 1 race; braking augments the selected mode
    double deployment{1};
};
struct DynamicState {
    ContactVehicleState contact;
    int gear{1};
    double shift_remaining_s{0}, aero_position{0}, drs_position{0};
    bool resources_enabled{true}; // false only for documented fixed-condition calibration benches
    double fuel_mass_kg{0}, battery_energy_j{0}, deployed_lap_j{0}, recovered_lap_j{0};
    std::array<double,4> tread_c{},carcass_c{},wear{};
};
struct DynamicTelemetry {
    ContactVehicleEvaluation contact;
    double rpm{0}, drive_force_n{0}, wheel_power_w{0};
    double drag_n{0}, downforce_n{0}, front_aero_fraction{0};
    double cda_m2{0}, cla_m2{0}, front_height_m{0}, rear_height_m{0};
    ContactOperatingPoint operating;
    double ice_available_w{0}, hybrid_available_w{0}, fuel_flow_kg_s{0};
    double battery_discharge_w{0}, battery_charge_w{0};
    std::array<double,4> friction_heat_w{},deformation_heat_w{};
};
class DynamicVehicleModel {
  public:
    explicit DynamicVehicleModel(DynamicVehicleParameters p);
    const DynamicVehicleParameters& parameters() const { return parameters_; }
    DynamicState initial(double speed=0) const;
    DynamicTelemetry evaluate(const DynamicState&,DynamicControl,const RoadSurfaceQuery&) const;
    DynamicState step(const DynamicState&,DynamicControl,const RoadSurfaceQuery&,double dt) const;
    // Call only on an actual forward finish-line crossing; SOC/fuel/tires persist.
    void begin_lap(DynamicState& state) const { state.deployed_lap_j=0;state.recovered_lap_j=0; }
  private:
    DynamicTelemetry operating(const DynamicState&,DynamicControl,const RoadSurfaceQuery&) const;
    DynamicVehicleParameters parameters_;
    ContactVehicleModel contact_;
};
} // namespace apexlab

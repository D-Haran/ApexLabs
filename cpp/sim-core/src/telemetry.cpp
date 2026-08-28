#include "apexlab/telemetry.hpp"

#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace apexlab {

void write_telemetry_csv(const std::filesystem::path& path,
                         std::span<const TelemetrySample> samples) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream stream(path);
    if (!stream) {
        throw std::runtime_error("cannot write telemetry CSV: " + path.string());
    }

    stream << "schema_version,time_s,step,position_m,velocity_mps,acceleration_mps2,"
              "throttle,brake,drive_force_n,brake_force_n,drag_force_n,"
              "rolling_resistance_n,downforce_n,traction_limit_n\n";
    stream << std::setprecision(17);
    for (const auto& sample : samples) {
        stream << sample.schema_version << ',' << sample.time.value() << ',' << sample.step << ','
               << sample.state.position.value() << ',' << sample.state.velocity.value() << ','
               << sample.acceleration.value() << ',' << sample.input.throttle << ','
               << sample.input.brake << ',' << sample.forces.drive.value() << ','
               << sample.forces.brake.value() << ',' << sample.forces.aerodynamic_drag.value()
               << ',' << sample.forces.rolling_resistance.value() << ','
               << sample.forces.aerodynamic_downforce.value() << ','
               << sample.forces.traction_limit.value() << '\n';
    }
    if (!stream) {
        throw std::runtime_error("failed while writing telemetry CSV: " + path.string());
    }
}

void write_planar_telemetry_csv(const std::filesystem::path& path,
                                std::span<const PlanarTelemetrySample> samples) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream stream(path);
    if (!stream) {
        throw std::runtime_error("cannot write planar telemetry CSV: " + path.string());
    }

    stream << "schema_version,time_s,step,position_x_m,position_y_m,yaw_rad,"
              "yaw_rate_rad_s,vx_m_s,vy_m_s,speed_m_s,steering_angle_rad,throttle,brake,"
              "front_slip_angle_rad,rear_slip_angle_rad,front_lateral_force_n,"
              "rear_lateral_force_n,longitudinal_accel_m_s2,lateral_accel_m_s2,"
              "yaw_accel_rad_s2,body_longitudinal_force_n,body_lateral_force_n,yaw_moment_nm,"
              "front_normal_load_n,rear_normal_load_n,drive_force_n,brake_force_n,"
              "drag_force_n,rolling_resistance_n,downforce_n,traction_limit_n\n";
    stream << std::setprecision(17);
    for (const auto& sample : samples) {
        const PlanarState& state = sample.state;
        const PlanarForces& forces = sample.forces;
        stream << sample.schema_version << ',' << sample.time.value() << ',' << sample.step << ','
               << state.position_x.value() << ',' << state.position_y.value() << ','
               << state.yaw.value() << ',' << state.yaw_rate.value() << ','
               << state.velocity_x.value() << ',' << state.velocity_y.value() << ','
               << sample.speed_mps << ',' << sample.input.steering_angle.value() << ','
               << sample.input.throttle << ',' << sample.input.brake << ','
               << forces.front_slip_angle.value() << ',' << forces.rear_slip_angle.value() << ','
               << forces.front_lateral_force.value() << ',' << forces.rear_lateral_force.value()
               << ',' << sample.longitudinal_acceleration.value() << ','
               << sample.lateral_acceleration.value() << ',' << sample.yaw_acceleration.value()
               << ',' << forces.body_longitudinal_force.value() << ','
               << forces.body_lateral_force.value() << ',' << forces.yaw_moment_nm << ','
               << forces.front_normal_load.value() << ',' << forces.rear_normal_load.value() << ','
               << forces.longitudinal.drive.value() << ',' << forces.longitudinal.brake.value()
               << ',' << forces.longitudinal.aerodynamic_drag.value() << ','
               << forces.longitudinal.rolling_resistance.value() << ','
               << forces.longitudinal.aerodynamic_downforce.value() << ','
               << forces.longitudinal.traction_limit.value() << '\n';
    }
    if (!stream) {
        throw std::runtime_error("failed while writing planar telemetry CSV: " + path.string());
    }
}

void write_nonlinear_planar_telemetry_csv(
    const std::filesystem::path& path,
    std::span<const NonlinearPlanarTelemetrySample> samples) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream stream(path);
    if (!stream) {
        throw std::runtime_error("cannot write nonlinear planar telemetry CSV: " + path.string());
    }
    stream << "schema_version,time_s,step,position_x_m,position_y_m,yaw_rad,yaw_rate_rad_s,"
              "vx_m_s,vy_m_s,speed_m_s,steering_angle_rad,throttle,brake,"
              "longitudinal_accel_m_s2,lateral_accel_m_s2,yaw_accel_rad_s2,"
              "body_longitudinal_force_n,body_lateral_force_n,yaw_moment_nm,"
              "longitudinal_load_transfer_n,front_lateral_load_transfer_n,"
              "rear_lateral_load_transfer_n,solver_iterations,solver_residual_m_s2,"
              "solver_converged";
    for (const char* suffix : {"fl", "fr", "rl", "rr"}) {
        stream << ",fz_" << suffix << "_n,slip_angle_" << suffix << "_rad,requested_fx_"
               << suffix << "_n,requested_fy_" << suffix << "_n,fx_" << suffix << "_n,fy_"
               << suffix << "_n,force_capacity_" << suffix << "_n,effective_mu_" << suffix
               << ",requested_friction_utilization_" << suffix << ",friction_utilization_"
               << suffix << ",tire_saturated_" << suffix;
    }
    stream << "\n" << std::setprecision(17);
    for (const auto& sample : samples) {
        const auto& state = sample.state;
        const auto& forces = sample.forces;
        stream << sample.schema_version << ',' << sample.time.value() << ',' << sample.step << ','
               << state.position_x.value() << ',' << state.position_y.value() << ','
               << state.yaw.value() << ',' << state.yaw_rate.value() << ','
               << state.velocity_x.value() << ',' << state.velocity_y.value() << ','
               << sample.speed_mps << ',' << sample.input.steering_angle.value() << ','
               << sample.input.throttle << ',' << sample.input.brake << ','
               << sample.longitudinal_acceleration.value() << ','
               << sample.lateral_acceleration.value() << ',' << sample.yaw_acceleration.value()
               << ',' << forces.body_longitudinal_force.value() << ','
               << forces.body_lateral_force.value() << ',' << forces.yaw_moment_nm << ','
               << forces.longitudinal_load_transfer.value() << ','
               << forces.front_lateral_load_transfer.value() << ','
               << forces.rear_lateral_load_transfer.value() << ',' << forces.solver_iterations
               << ',' << forces.solver_residual_mps2 << ',' << (forces.solver_converged ? 1 : 0);
        for (const auto& wheel : forces.wheel) {
            stream << ',' << wheel.normal_load.value() << ',' << wheel.slip_angle.value() << ','
                   << wheel.requested_longitudinal_force.value() << ','
                   << wheel.requested_lateral_force.value() << ','
                   << wheel.longitudinal_force.value() << ',' << wheel.lateral_force.value() << ','
                   << wheel.force_capacity.value() << ',' << wheel.effective_friction_coefficient
                   << ',' << wheel.requested_friction_utilization << ','
                   << wheel.friction_utilization << ',' << (wheel.saturated ? 1 : 0);
        }
        stream << '\n';
    }
    if (!stream) {
        throw std::runtime_error("failed while writing nonlinear telemetry CSV: " + path.string());
    }
}

} // namespace apexlab

#include "replay_export.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace {
std::ofstream output(const std::filesystem::path& path) {
    std::ofstream stream(path);
    if (!stream) throw std::runtime_error("cannot write replay artifact: " + path.string());
    stream.exceptions(std::ios::badbit | std::ios::failbit);
    stream << std::setprecision(17);
    return stream;
}
}
void write_replay_data(const std::filesystem::path& directory,
                       const apexlab::LapSimulationResult& result,
                       const apexlab::PeriodicTrack& track,
                       const apexlab::LapSimulationOptions& options) {
    if (!result.completed) throw std::runtime_error("replay export requires completed laps");
    std::filesystem::create_directories(directory);
    auto wheels = output(directory / "wheels.csv");
    wheels << "schema_version,time_s,step";
    for (const char* w : {"fl", "fr", "rl", "rr"}) {
        wheels << ",fz_" << w << "_n,fx_" << w << "_n,fy_" << w
               << "_n,slip_angle_" << w << "_rad,force_capacity_" << w
               << "_n,tire_saturated_" << w;
    }
    wheels << '\n';
    for (const auto& s : result.samples) {
        wheels << "1," << s.time.value() << ',' << s.step;
        for (const auto& w : s.forces.wheel) {
            wheels << ',' << w.normal_load.value() << ',' << w.longitudinal_force.value()
                   << ',' << w.lateral_force.value() << ',' << w.slip_angle.value()
                   << ',' << w.force_capacity.value() << ',' << (w.saturated ? 1 : 0);
        }
        wheels << '\n';
    }
    if (!result.samples.empty() && result.samples.front().chassis) {
        auto chassis = output(directory / "chassis.csv");
        chassis << "schema_version,time_s,step,heave_m,roll_rad,pitch_rad,heave_rate_m_s,roll_rate_rad_s,pitch_rate_rad_s,compression_fl_m,compression_fr_m,compression_rl_m,compression_rr_m,gravity_x_m_s2,gravity_y_m_s2,gravity_z_m_s2\n";
        for (const auto& row : result.samples) {
            const auto& b = *row.chassis;
            chassis << "1," << row.time.value() << ',' << row.step << ',' << b.heave << ',' << b.roll << ',' << b.pitch << ',' << b.heave_rate << ',' << b.roll_rate << ',' << b.pitch_rate;
            for (const double compression : row.suspension_compression_m) chassis << ',' << compression;
            chassis << ',' << row.gravity_body.x << ',' << row.gravity_body.y << ',' << row.gravity_body.z << '\n';
        }
    }
    auto spatial = output(directory / "spatial.csv");
    spatial << "schema_version,lap_number,s_m,time_s,speed_m_s,throttle,brake,steering_angle_rad,lateral_accel_m_s2,maximum_tire_utilization,lateral_error_m\n";
    for (int lap = options.warmup_laps; lap < options.warmup_laps + options.timed_laps; ++lap) {
        std::vector<apexlab::LapTelemetrySample> clean;
        // The simulator labels the crossing sample with the lap it finishes. Exclude that
        // wrapped endpoint before invoking the preserved M4 spatial resampler.
        for (const auto& s : result.samples) {
            if (s.lap_number == lap && s.unwrapped_track_s_m < (lap + 1) * track.length_m())
                clean.push_back(s);
        }
        for (const auto& s : apexlab::resample_lap_spatially(clean, lap, track.length_m(), 1.0)) {
            const auto hi = std::lower_bound(clean.begin(), clean.end(), s.s_m,
                [](const auto& row, double value) { return row.track_s_m < value; });
            if (hi == clean.begin() || hi == clean.end()) continue;
            const auto& lo = *std::prev(hi);
            const double f = (s.s_m - lo.track_s_m) / (hi->track_s_m - lo.track_s_m);
            const double t = lo.time.value() + f * (hi->time.value() - lo.time.value());
            spatial << "1," << lap << ',' << s.s_m << ',' << t << ',' << s.speed_mps << ','
                    << s.throttle << ',' << s.brake << ',' << s.steering_rad << ','
                    << s.lateral_acceleration_mps2 << ',' << s.maximum_tire_utilization << ','
                    << s.lateral_error_m << '\n';
        }
    }
    auto results = output(directory / "results.json");
    results << "{\"track_length_m\":" << track.length_m() << ",\"lap_times_s\":[";
    for (std::size_t i = 0; i < result.completed_lap_times_s.size(); ++i) {
        if (i) results << ',';
        results << result.completed_lap_times_s[i];
    }
    results << "],\"sector_times_s\":[";
    for (std::size_t i = 0; i < result.completed_sector_times_s.size(); ++i) {
        if (i) results << ',';
        results << result.completed_sector_times_s[i];
    }
    results << "]}\n";
}

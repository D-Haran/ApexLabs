#pragma once

#include <cstdint>
#include <filesystem>
#include <span>

#include "apexlab/longitudinal_model.hpp"
#include "apexlab/nonlinear_planar_model.hpp"
#include "apexlab/planar_model.hpp"

namespace apexlab {

inline constexpr int telemetry_schema_version = 1;
inline constexpr int planar_telemetry_schema_version = 2;
inline constexpr int nonlinear_planar_telemetry_schema_version = 3;

struct TelemetrySample {
    int schema_version{telemetry_schema_version};
    Time time{si::seconds(0.0)};
    std::uint64_t step{0};
    LongitudinalState state;
    Acceleration acceleration{si::meters_per_second_squared(0.0)};
    DriverInput input;
    LongitudinalForces forces;
};

struct PlanarTelemetrySample {
    int schema_version{planar_telemetry_schema_version};
    Time time{si::seconds(0.0)};
    std::uint64_t step{0};
    PlanarState state;
    double speed_mps{0.0};
    PlanarControl input;
    PlanarForces forces;
    Acceleration longitudinal_acceleration{si::meters_per_second_squared(0.0)};
    Acceleration lateral_acceleration{si::meters_per_second_squared(0.0)};
    AngularAcceleration yaw_acceleration{si::radians_per_second_squared(0.0)};
};

struct NonlinearPlanarTelemetrySample {
    int schema_version{nonlinear_planar_telemetry_schema_version};
    Time time{si::seconds(0.0)};
    std::uint64_t step{0};
    PlanarState state;
    double speed_mps{0.0};
    PlanarControl input;
    NonlinearPlanarForces forces;
    Acceleration longitudinal_acceleration{si::meters_per_second_squared(0.0)};
    Acceleration lateral_acceleration{si::meters_per_second_squared(0.0)};
    AngularAcceleration yaw_acceleration{si::radians_per_second_squared(0.0)};
};

void write_telemetry_csv(const std::filesystem::path& path,
                         std::span<const TelemetrySample> samples);
void write_planar_telemetry_csv(const std::filesystem::path& path,
                                std::span<const PlanarTelemetrySample> samples);
void write_nonlinear_planar_telemetry_csv(
    const std::filesystem::path& path,
    std::span<const NonlinearPlanarTelemetrySample> samples);

} // namespace apexlab

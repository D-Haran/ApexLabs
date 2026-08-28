#pragma once

#include <filesystem>
#include <string>

#include "apexlab/integrator.hpp"
#include "apexlab/planar_model.hpp"

namespace apexlab {

inline constexpr int scenario_schema_version = 1;

enum class ProfileKind { constant, step, ramp, sine };

struct ScalarProfile {
    ProfileKind kind{ProfileKind::constant};
    double constant_value{0.0};
    double initial_value{0.0};
    double final_value{0.0};
    Time start_time{si::seconds(0.0)};
    Time ramp_duration{si::seconds(0.0)};
    double offset{0.0};
    double amplitude{0.0};
    double frequency_hz{0.0};
    Angle phase{si::radians(0.0)};

    [[nodiscard]] double value_at(Time time) const;
};

struct PlanarScenario {
    int schema_version{scenario_schema_version};
    std::string name;
    Time duration{si::seconds(0.0)};
    Time timestep{si::seconds(0.0)};
    IntegratorKind integrator{IntegratorKind::rk4};
    PlanarState initial_state;
    ScalarProfile steering_profile;
    ScalarProfile throttle_profile;
    ScalarProfile brake_profile;

    [[nodiscard]] PlanarControl control_at(Time time) const;
};

void validate(const PlanarScenario& scenario);
[[nodiscard]] PlanarScenario load_planar_scenario(const std::filesystem::path& path);

} // namespace apexlab

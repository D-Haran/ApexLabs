#include "apexlab/scenario.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <numbers>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace apexlab {
namespace {

[[nodiscard]] std::string read_file(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream) {
        throw std::runtime_error("cannot open scenario configuration: " + path.string());
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    return buffer.str();
}

[[nodiscard]] std::string object_value(const std::string& document, const std::string& key) {
    const std::string quoted_key = '"' + key + '"';
    const std::size_t key_position = document.find(quoted_key);
    if (key_position == std::string::npos) {
        throw std::runtime_error("missing scenario object: " + key);
    }
    const std::size_t open = document.find('{', key_position + quoted_key.size());
    if (open == std::string::npos) {
        throw std::runtime_error("invalid scenario object: " + key);
    }
    int depth = 0;
    bool in_string = false;
    bool escaped = false;
    for (std::size_t index = open; index < document.size(); ++index) {
        const char character = document[index];
        if (in_string) {
            if (escaped) {
                escaped = false;
            } else if (character == '\\') {
                escaped = true;
            } else if (character == '"') {
                in_string = false;
            }
            continue;
        }
        if (character == '"') {
            in_string = true;
        } else if (character == '{') {
            ++depth;
        } else if (character == '}') {
            --depth;
            if (depth == 0) {
                return document.substr(open, index - open + 1U);
            }
        }
    }
    throw std::runtime_error("unterminated scenario object: " + key);
}

[[nodiscard]] std::optional<double> optional_number(const std::string& document,
                                                    const std::string& key) {
    const std::regex expression{
        "\"" + key + "\"\\s*:\\s*(-?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][+-]?[0-9]+)?)"};
    std::smatch match;
    if (!std::regex_search(document, match, expression)) {
        return std::nullopt;
    }
    return std::stod(match[1].str());
}

[[nodiscard]] double required_number(const std::string& document, const std::string& key) {
    const std::optional<double> value = optional_number(document, key);
    if (!value) {
        throw std::runtime_error("missing or invalid numeric scenario field: " + key);
    }
    return *value;
}

[[nodiscard]] std::string required_string(const std::string& document, const std::string& key) {
    const std::regex expression{"\"" + key + "\"\\s*:\\s*\"([^\"]*)\""};
    std::smatch match;
    if (!std::regex_search(document, match, expression)) {
        throw std::runtime_error("missing or invalid string scenario field: " + key);
    }
    return match[1].str();
}

[[nodiscard]] ScalarProfile parse_profile(const std::string& document) {
    ScalarProfile profile;
    const std::string type = required_string(document, "type");
    if (type == "constant") {
        profile.kind = ProfileKind::constant;
        profile.constant_value = required_number(document, "value");
    } else if (type == "step") {
        profile.kind = ProfileKind::step;
        profile.initial_value = required_number(document, "initial_value");
        profile.final_value = required_number(document, "final_value");
        profile.start_time = si::seconds(required_number(document, "start_time_s"));
    } else if (type == "ramp") {
        profile.kind = ProfileKind::ramp;
        profile.initial_value = required_number(document, "initial_value");
        profile.final_value = required_number(document, "final_value");
        profile.start_time = si::seconds(required_number(document, "start_time_s"));
        profile.ramp_duration = si::seconds(required_number(document, "duration_s"));
    } else if (type == "sine") {
        profile.kind = ProfileKind::sine;
        profile.offset = required_number(document, "offset");
        profile.amplitude = required_number(document, "amplitude");
        profile.frequency_hz = required_number(document, "frequency_hz");
        profile.phase = si::radians(optional_number(document, "phase_rad").value_or(0.0));
        profile.start_time = si::seconds(optional_number(document, "start_time_s").value_or(0.0));
    } else {
        throw std::runtime_error("scenario profile type must be constant, step, ramp, or sine");
    }
    return profile;
}

[[nodiscard]] std::pair<double, double> profile_range(const ScalarProfile& profile) {
    switch (profile.kind) {
    case ProfileKind::constant:
        return {profile.constant_value, profile.constant_value};
    case ProfileKind::step:
    case ProfileKind::ramp:
        return {std::min(profile.initial_value, profile.final_value),
                std::max(profile.initial_value, profile.final_value)};
    case ProfileKind::sine:
        return {profile.offset - std::abs(profile.amplitude),
                profile.offset + std::abs(profile.amplitude)};
    }
    throw std::logic_error("unknown profile kind");
}

void validate_profile(const ScalarProfile& profile, const char* name) {
    const auto [minimum, maximum] = profile_range(profile);
    if (!std::isfinite(minimum) || !std::isfinite(maximum) ||
        !std::isfinite(profile.start_time.value())) {
        throw std::invalid_argument(std::string{name} + " profile values must be finite");
    }
    if (profile.start_time.value() < 0.0) {
        throw std::invalid_argument(std::string{name} + " profile start time must be nonnegative");
    }
    if (profile.kind == ProfileKind::ramp &&
        (!std::isfinite(profile.ramp_duration.value()) || profile.ramp_duration.value() <= 0.0)) {
        throw std::invalid_argument(std::string{name} + " ramp duration must be positive");
    }
    if (profile.kind == ProfileKind::sine &&
        (!std::isfinite(profile.frequency_hz) || profile.frequency_hz < 0.0 ||
         !std::isfinite(profile.phase.value()))) {
        throw std::invalid_argument(std::string{name} + " sine parameters are invalid");
    }
}

} // namespace

double ScalarProfile::value_at(Time time) const {
    switch (kind) {
    case ProfileKind::constant:
        return constant_value;
    case ProfileKind::step:
        return time.value() < start_time.value() ? initial_value : final_value;
    case ProfileKind::ramp: {
        if (time.value() <= start_time.value()) {
            return initial_value;
        }
        const double fraction =
            std::clamp((time.value() - start_time.value()) / ramp_duration.value(), 0.0, 1.0);
        return initial_value + fraction * (final_value - initial_value);
    }
    case ProfileKind::sine:
        if (time.value() < start_time.value()) {
            return offset;
        }
        return offset + amplitude * std::sin(2.0 * std::numbers::pi * frequency_hz *
                                                 (time.value() - start_time.value()) +
                                             phase.value());
    }
    throw std::logic_error("unknown profile kind");
}

PlanarControl PlanarScenario::control_at(Time time) const {
    return {
        si::radians(steering_profile.value_at(time)),
        throttle_profile.value_at(time),
        brake_profile.value_at(time),
    };
}

void validate(const PlanarScenario& scenario) {
    if (scenario.schema_version != scenario_schema_version) {
        throw std::invalid_argument("unsupported scenario schema version");
    }
    if (scenario.name.empty()) {
        throw std::invalid_argument("scenario name must not be empty");
    }
    if (!std::isfinite(scenario.duration.value()) || scenario.duration.value() < 0.0) {
        throw std::invalid_argument("scenario duration_s must be finite and nonnegative");
    }
    if (!std::isfinite(scenario.timestep.value()) || scenario.timestep.value() <= 0.0) {
        throw std::invalid_argument("scenario timestep_s must be finite and positive");
    }
    const PlanarState& state = scenario.initial_state;
    const double initial_values[] = {
        state.position_x.value(), state.position_y.value(), state.yaw.value(),
        state.velocity_x.value(), state.velocity_y.value(), state.yaw_rate.value(),
    };
    if (!std::all_of(std::begin(initial_values), std::end(initial_values),
                     [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("scenario initial state values must be finite");
    }
    if (state.velocity_x.value() < 0.0) {
        throw std::invalid_argument("planar scenarios do not support negative initial vx_m_s");
    }

    validate_profile(scenario.steering_profile, "steering");
    validate_profile(scenario.throttle_profile, "throttle");
    validate_profile(scenario.brake_profile, "brake");
    const auto [steering_minimum, steering_maximum] = profile_range(scenario.steering_profile);
    constexpr double steering_envelope_rad = 0.35;
    if (steering_minimum < -steering_envelope_rad || steering_maximum > steering_envelope_rad) {
        throw std::invalid_argument("steering profile exceeds the documented +/-0.35 rad envelope");
    }
    for (const auto* profile : {&scenario.throttle_profile, &scenario.brake_profile}) {
        const auto [minimum, maximum] = profile_range(*profile);
        if (minimum < 0.0 || maximum > 1.0) {
            throw std::invalid_argument("throttle and brake profiles must remain in [0,1]");
        }
    }
}

PlanarScenario load_planar_scenario(const std::filesystem::path& path) {
    const std::string document = read_file(path);
    PlanarScenario scenario;
    const double version = required_number(document, "schema_version");
    if (std::floor(version) != version) {
        throw std::runtime_error("scenario schema_version must be an integer");
    }
    scenario.schema_version = static_cast<int>(version);
    scenario.name = required_string(document, "name");
    scenario.duration = si::seconds(required_number(document, "duration_s"));
    scenario.timestep = si::seconds(required_number(document, "timestep_s"));
    const std::string integrator = required_string(document, "integrator");
    if (integrator == "euler") {
        scenario.integrator = IntegratorKind::euler;
    } else if (integrator == "rk4") {
        scenario.integrator = IntegratorKind::rk4;
    } else {
        throw std::runtime_error("scenario integrator must be euler or rk4");
    }

    const std::string initial = object_value(document, "initial_state");
    scenario.initial_state = {
        si::meters(required_number(initial, "position_x_m")),
        si::meters(required_number(initial, "position_y_m")),
        si::radians(required_number(initial, "yaw_rad")),
        si::meters_per_second(required_number(initial, "vx_m_s")),
        si::meters_per_second(required_number(initial, "vy_m_s")),
        si::radians_per_second(required_number(initial, "yaw_rate_rad_s")),
    };
    scenario.steering_profile = parse_profile(object_value(document, "steering_profile"));
    scenario.throttle_profile = parse_profile(object_value(document, "throttle_profile"));
    scenario.brake_profile = parse_profile(object_value(document, "brake_profile"));
    validate(scenario);
    return scenario;
}

} // namespace apexlab

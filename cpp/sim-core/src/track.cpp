#include "apexlab/track.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numbers>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace apexlab {
namespace {

[[nodiscard]] Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
[[nodiscard]] Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
[[nodiscard]] Vec2 operator*(Vec2 a, double scale) { return {a.x * scale, a.y * scale}; }
[[nodiscard]] double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
[[nodiscard]] double norm(Vec2 a) { return std::hypot(a.x, a.y); }
[[nodiscard]] double squared_distance(Vec2 a, Vec2 b) {
    const Vec2 delta = a - b;
    return dot(delta, delta);
}

[[nodiscard]] std::string read_file(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream) {
        throw std::runtime_error("cannot open track configuration: " + path.string());
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    return buffer.str();
}

[[nodiscard]] std::string required_string(const std::string& text, const std::string& key) {
    const std::regex expression{"\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]+)\\\""};
    std::smatch match;
    if (!std::regex_search(text, match, expression)) {
        throw std::runtime_error("missing track string: " + key);
    }
    return match[1].str();
}

[[nodiscard]] double required_number(const std::string& text, const std::string& key) {
    const std::regex expression{"\\\"" + key + "\\\"\\s*:\\s*([-+0-9.eE]+)"};
    std::smatch match;
    if (!std::regex_search(text, match, expression)) {
        throw std::runtime_error("missing track number: " + key);
    }
    return std::stod(match[1].str());
}

[[nodiscard]] double optional_number(const std::string& text, const std::string& key,
                                     double fallback) {
    const std::regex expression{"\\\"" + key + "\\\"\\s*:\\s*([-+0-9.eE]+)"};
    std::smatch match;
    return std::regex_search(text, match, expression) ? std::stod(match[1].str()) : fallback;
}

[[nodiscard]] TrackDefinition make_track(std::string name, std::vector<Vec2> points,
                                         double width = 7.0) {
    TrackDefinition result;
    result.name = std::move(name);
    result.provenance = "ApexLab generated validation geometry";
    for (Vec2 point : points) {
        result.control_points.push_back({point, 0.0, width, width});
    }
    result.sector_boundaries_fraction = {1.0 / 3.0, 2.0 / 3.0, 1.0};
    return result;
}

} // namespace

PeriodicTrack::PeriodicTrack(TrackDefinition definition, std::size_t samples_per_segment)
    : definition_(std::move(definition)) {
    if (definition_.schema_version != legacy_track_schema_version &&
        definition_.schema_version != track_schema_version) {
        throw std::invalid_argument("track requires schema version 1 or 2");
    }
    const std::size_t minimum_points = definition_.closed ? 4U : 2U;
    if (definition_.control_points.size() < minimum_points || samples_per_segment < 8U) {
        throw std::invalid_argument("track has too few control points or arc samples");
    }
    for (const auto& point : definition_.control_points) {
        if (!std::isfinite(point.position.x) || !std::isfinite(point.position.y) ||
            !std::isfinite(point.elevation_m) ||
            !std::isfinite(point.left_width_m) || !std::isfinite(point.right_width_m) ||
            point.left_width_m <= 0.0 || point.right_width_m <= 0.0) {
            throw std::invalid_argument("track points and widths must be finite; widths must be positive");
        }
    }
    const std::size_t segment_count = definition_.closed ? definition_.control_points.size()
                                                         : definition_.control_points.size() - 1U;
    const std::size_t count = segment_count * samples_per_segment;
    arc_table_.reserve(count + 1U);
    Vec2 previous = evaluate(0.0);
    double previous_elevation = evaluate_elevation(0.0);
    arc_table_.push_back({0.0, 0.0, previous});
    double cumulative = 0.0;
    for (std::size_t index = 1; index <= count; ++index) {
        const double parameter = static_cast<double>(segment_count) *
                                 static_cast<double>(index) / static_cast<double>(count);
        const Vec2 current = evaluate(parameter);
        const double elevation = evaluate_elevation(parameter);
        cumulative += std::hypot(norm(current - previous), elevation - previous_elevation);
        arc_table_.push_back({parameter, cumulative, current});
        previous = current;
        previous_elevation = elevation;
    }
    length_m_ = cumulative;
    if (!(length_m_ > 0.0)) {
        throw std::invalid_argument("track length must be positive");
    }
}

Vec2 PeriodicTrack::evaluate(double parameter) const {
    const auto count = static_cast<long>(definition_.control_points.size());
    if (definition_.closed) {
        parameter = std::fmod(parameter, static_cast<double>(count));
        if (parameter < 0.0) parameter += static_cast<double>(count);
    } else {
        parameter = std::clamp(parameter, 0.0, static_cast<double>(count - 1L));
    }
    const long segment = definition_.closed ? static_cast<long>(std::floor(parameter))
                                            : std::min(count - 2L, static_cast<long>(std::floor(parameter)));
    const double t = parameter - static_cast<double>(segment);
    const auto point = [&](long index) -> Vec2 {
        index = definition_.closed ? (index % count + count) % count
                                   : std::clamp(index, 0L, count - 1L);
        return definition_.control_points[static_cast<std::size_t>(index)].position;
    };
    const Vec2 p0 = point(segment - 1), p1 = point(segment), p2 = point(segment + 1),
               p3 = point(segment + 2);
    return (p1 * 2.0 + (p2 - p0) * t +
            (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * (t * t) +
            (p0 * -1.0 + p1 * 3.0 - p2 * 3.0 + p3) * (t * t * t)) * 0.5;
}

Vec2 PeriodicTrack::derivative(double parameter) const {
    const auto count = static_cast<long>(definition_.control_points.size());
    if (definition_.closed) {
        parameter = std::fmod(parameter, static_cast<double>(count));
        if (parameter < 0.0) parameter += static_cast<double>(count);
    } else parameter = std::clamp(parameter, 0.0, static_cast<double>(count - 1L));
    const long segment = definition_.closed ? static_cast<long>(std::floor(parameter))
                                            : std::min(count - 2L, static_cast<long>(std::floor(parameter)));
    const double t = parameter - static_cast<double>(segment);
    const auto point = [&](long index) -> Vec2 {
        index = definition_.closed ? (index % count + count) % count
                                   : std::clamp(index, 0L, count - 1L);
        return definition_.control_points[static_cast<std::size_t>(index)].position;
    };
    const Vec2 p0 = point(segment - 1), p1 = point(segment), p2 = point(segment + 1),
               p3 = point(segment + 2);
    return ((p2 - p0) + (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * (2.0 * t) +
            (p0 * -1.0 + p1 * 3.0 - p2 * 3.0 + p3) * (3.0 * t * t)) * 0.5;
}

Vec2 PeriodicTrack::second_derivative(double parameter) const {
    const auto count = static_cast<long>(definition_.control_points.size());
    if (definition_.closed) {
        parameter = std::fmod(parameter, static_cast<double>(count));
        if (parameter < 0.0) parameter += static_cast<double>(count);
    } else parameter = std::clamp(parameter, 0.0, static_cast<double>(count - 1L));
    const long segment = definition_.closed ? static_cast<long>(std::floor(parameter))
                                            : std::min(count - 2L, static_cast<long>(std::floor(parameter)));
    const double t = parameter - static_cast<double>(segment);
    const auto point = [&](long index) -> Vec2 {
        index = definition_.closed ? (index % count + count) % count
                                   : std::clamp(index, 0L, count - 1L);
        return definition_.control_points[static_cast<std::size_t>(index)].position;
    };
    const Vec2 p0 = point(segment - 1), p1 = point(segment), p2 = point(segment + 1),
               p3 = point(segment + 2);
    return (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) +
           (p0 * -1.0 + p1 * 3.0 - p2 * 3.0 + p3) * (3.0 * t);
}

double PeriodicTrack::evaluate_elevation(double parameter) const {
    const auto count = static_cast<long>(definition_.control_points.size());
    if (definition_.closed) {
        parameter = std::fmod(parameter, static_cast<double>(count));
        if (parameter < 0.0) parameter += static_cast<double>(count);
    } else {
        parameter = std::clamp(parameter, 0.0, static_cast<double>(count - 1L));
    }
    const long segment = definition_.closed ? static_cast<long>(std::floor(parameter))
                                            : std::min(count - 2L, static_cast<long>(std::floor(parameter)));
    const double t = parameter - static_cast<double>(segment);
    const auto elevation = [&](long index) {
        index = definition_.closed ? (index % count + count) % count
                                   : std::clamp(index, 0L, count - 1L);
        return definition_.control_points[static_cast<std::size_t>(index)].elevation_m;
    };
    const double p0 = elevation(segment - 1), p1 = elevation(segment),
                 p2 = elevation(segment + 1), p3 = elevation(segment + 2);
    return 0.5 * (2.0 * p1 + (p2 - p0) * t +
                  (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t * t +
                  (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t * t * t);
}

double PeriodicTrack::elevation_derivative(double parameter) const {
    const auto count = static_cast<long>(definition_.control_points.size());
    if (definition_.closed) {
        parameter = std::fmod(parameter, static_cast<double>(count));
        if (parameter < 0.0) parameter += static_cast<double>(count);
    } else {
        parameter = std::clamp(parameter, 0.0, static_cast<double>(count - 1L));
    }
    const long segment = definition_.closed ? static_cast<long>(std::floor(parameter))
                                            : std::min(count - 2L, static_cast<long>(std::floor(parameter)));
    const double t = parameter - static_cast<double>(segment);
    const auto elevation = [&](long index) {
        index = definition_.closed ? (index % count + count) % count
                                   : std::clamp(index, 0L, count - 1L);
        return definition_.control_points[static_cast<std::size_t>(index)].elevation_m;
    };
    const double p0 = elevation(segment - 1), p1 = elevation(segment),
                 p2 = elevation(segment + 1), p3 = elevation(segment + 2);
    return 0.5 * ((p2 - p0) + 2.0 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t +
                  3.0 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t * t);
}

double PeriodicTrack::wrap_s(double s_m) const {
    if (!definition_.closed) return std::clamp(s_m, 0.0, length_m_);
    double wrapped = std::fmod(s_m, length_m_);
    if (wrapped < 0.0) wrapped += length_m_;
    return wrapped;
}

double PeriodicTrack::parameter_at_s(double s_m) const {
    const double wrapped = wrap_s(s_m);
    const auto upper = std::lower_bound(arc_table_.begin(), arc_table_.end(), wrapped,
                                       [](const ArcSample& sample, double value) {
                                           return sample.s_m < value;
                                       });
    if (upper == arc_table_.begin()) return upper->parameter;
    const auto lower = std::prev(upper);
    const double fraction = (wrapped - lower->s_m) / (upper->s_m - lower->s_m);
    return lower->parameter + fraction * (upper->parameter - lower->parameter);
}

TrackFrame PeriodicTrack::frame_at_s(double s_m) const {
    const double parameter = parameter_at_s(s_m);
    const Vec2 position = evaluate(parameter);
    const Vec2 first = derivative(parameter);
    const Vec2 second = second_derivative(parameter);
    const double magnitude = norm(first);
    if (magnitude < 1.0e-12) throw std::runtime_error("degenerate track tangent");
    const Vec2 tangent{first.x / magnitude, first.y / magnitude};
    const double curvature = (first.x * second.y - first.y * second.x) /
                             (magnitude * magnitude * magnitude);
    const double point_parameter = std::floor(parameter);
    const std::size_t index = std::min(static_cast<std::size_t>(point_parameter),
                                       definition_.control_points.size() - 1U);
    const std::size_t next = definition_.closed ? (index + 1U) % definition_.control_points.size()
                                                : std::min(index + 1U, definition_.control_points.size() - 1U);
    const double t = parameter - point_parameter;
    const auto& a = definition_.control_points[index];
    const auto& b = definition_.control_points[next];
    const double elevation = evaluate_elevation(parameter);
    const double grade = elevation_derivative(parameter) / magnitude;
    return {wrap_s(s_m), position, tangent, {-tangent.y, tangent.x}, std::atan2(tangent.y, tangent.x),
            curvature, a.left_width_m + t * (b.left_width_m - a.left_width_m),
            a.right_width_m + t * (b.right_width_m - a.right_width_m), elevation, grade, 0.0};
}

Vec2 PeriodicTrack::position_at_s(double s_m, double lateral_offset_m) const {
    const TrackFrame frame = frame_at_s(s_m);
    return frame.position + frame.left_normal * lateral_offset_m;
}

Vec3 PeriodicTrack::position_3d_at_s(double s_m, double lateral_offset_m) const {
    const TrackFrame frame = frame_at_s(s_m);
    const Vec2 horizontal = frame.position + frame.left_normal * lateral_offset_m;
    return {horizontal.x, horizontal.y, frame.elevation_m};
}

bool PeriodicTrack::contains(double s_m, double lateral_offset_m) const {
    const TrackFrame frame = frame_at_s(s_m);
    return lateral_offset_m >= -frame.right_width_m && lateral_offset_m <= frame.left_width_m;
}

TrackCoordinate PeriodicTrack::refine_projection(Vec2 point, double parameter) const {
    const double parameter_count = static_cast<double>(definition_.closed ? definition_.control_points.size()
                                                                          : definition_.control_points.size() - 1U);
    for (int iteration = 0; iteration < 10; ++iteration) {
        const Vec2 position = evaluate(parameter);
        const Vec2 first = derivative(parameter);
        const Vec2 second = second_derivative(parameter);
        const Vec2 delta = position - point;
        const double denominator = dot(first, first) + dot(delta, second);
        if (std::abs(denominator) < 1.0e-12) break;
        parameter -= std::clamp(dot(delta, first) / denominator, -0.5, 0.5);
        if (definition_.closed) {
            parameter = std::fmod(parameter, parameter_count);
            if (parameter < 0.0) parameter += parameter_count;
        } else parameter = std::clamp(parameter, 0.0, parameter_count);
    }
    const Vec2 position = evaluate(parameter);
    const Vec2 first = derivative(parameter);
    const double first_norm = norm(first);
    const Vec2 left{-first.y / first_norm, first.x / first_norm};
    const Vec2 delta = point - position;
    const auto upper = std::lower_bound(arc_table_.begin(), arc_table_.end(), parameter,
                                       [](const ArcSample& sample, double value) {
                                           return sample.parameter < value;
                                       });
    double s = 0.0;
    if (upper == arc_table_.begin()) {
        s = upper->s_m;
    } else if (upper == arc_table_.end()) {
        s = length_m_;
    } else {
        const auto lower = std::prev(upper);
        const double f = (parameter - lower->parameter) / (upper->parameter - lower->parameter);
        s = lower->s_m + f * (upper->s_m - lower->s_m);
    }
    return {wrap_s(s), dot(delta, left), norm(delta)};
}

TrackCoordinate PeriodicTrack::project(Vec2 point, std::optional<double> previous_s_m,
                                       double search_window_m) const {
    std::size_t best_index = 0U;
    double best_distance = std::numeric_limits<double>::infinity();
    const auto consider = [&](std::size_t index) {
        const double candidate = squared_distance(point, arc_table_[index].position);
        if (candidate < best_distance) {
            best_distance = candidate;
            best_index = index;
        }
    };
    const std::size_t unique_samples = arc_table_.size() - 1U;
    if (previous_s_m) {
        const double center_s = wrap_s(*previous_s_m);
        auto center_iterator = std::lower_bound(arc_table_.begin(), arc_table_.end(), center_s,
                                                [](const ArcSample& sample, double value) {
                                                    return sample.s_m < value;
                                                });
        std::size_t center = static_cast<std::size_t>(std::distance(arc_table_.begin(), center_iterator));
        if (center >= unique_samples) center = unique_samples - 1U;
        const double average_spacing = length_m_ / static_cast<double>(unique_samples);
        const auto radius = static_cast<long>(std::ceil(search_window_m / average_spacing)) + 2L;
        for (long offset = -radius; offset <= radius; ++offset) {
            long candidate = static_cast<long>(center) + offset;
            if (definition_.closed) {
                const long count = static_cast<long>(unique_samples);
                candidate = (candidate % count + count) % count;
                consider(static_cast<std::size_t>(candidate));
            } else if (candidate >= 0L && candidate < static_cast<long>(unique_samples)) {
                consider(static_cast<std::size_t>(candidate));
            }
        }
    } else {
        for (std::size_t index = 0; index < unique_samples; ++index) consider(index);
    }
    return refine_projection(point, arc_table_[best_index].parameter);
}

TrackDefinition load_track_definition(const std::filesystem::path& path) {
    const std::string text = read_file(path);
    TrackDefinition result;
    result.schema_version = static_cast<int>(required_number(text, "schema_version"));
    result.name = required_string(text, "name");
    result.provenance = required_string(text, "provenance");
    const std::size_t closed_key = text.find("\"closed\"");
    if (closed_key == std::string::npos) throw std::runtime_error("missing track closed flag");
    result.closed = text.find("true", closed_key) < text.find("false", closed_key);
    const std::size_t points_key = text.find("\"control_points\"");
    const std::size_t points_begin = text.find('[', points_key);
    const std::size_t points_end = text.find(']', points_begin);
    if (points_key == std::string::npos || points_begin == std::string::npos || points_end == std::string::npos)
        throw std::runtime_error("missing track control_points array");
    std::size_t cursor = points_begin;
    while ((cursor = text.find('{', cursor)) != std::string::npos && cursor < points_end) {
        const std::size_t object_end = text.find('}', cursor);
        if (object_end == std::string::npos || object_end > points_end) break;
        const std::string object = text.substr(cursor, object_end - cursor + 1U);
        result.control_points.push_back({{required_number(object, "x_m"), required_number(object, "y_m")},
                                         optional_number(object, "elevation_m", 0.0),
                                         required_number(object, "left_width_m"),
                                         required_number(object, "right_width_m")});
        cursor = object_end + 1U;
    }
    const std::size_t sector_key = text.find("\"sector_boundaries_fraction\"");
    if (sector_key != std::string::npos) {
        const std::size_t begin = text.find('[', sector_key);
        const std::size_t end = text.find(']', begin);
        std::stringstream values(text.substr(begin + 1U, end - begin - 1U));
        std::string token;
        while (std::getline(values, token, ',')) result.sector_boundaries_fraction.push_back(std::stod(token));
    }
    return result;
}

TrackDefinition make_circle_track(double radius_m, double half_width_m, std::size_t count) {
    if (radius_m <= 0.0 || count < 8U) throw std::invalid_argument("circle dimensions invalid");
    std::vector<Vec2> points;
    for (std::size_t index = 0; index < count; ++index) {
        const double angle = 2.0 * std::numbers::pi * static_cast<double>(index) / static_cast<double>(count);
        points.push_back({radius_m * std::cos(angle), radius_m * std::sin(angle)});
    }
    return make_track("Generated circle", std::move(points), half_width_m);
}

TrackDefinition make_straight_test_path(double length_m, double half_width_m) {
    if (length_m <= 0.0 || half_width_m <= 0.0) {
        throw std::invalid_argument("straight dimensions invalid");
    }
    TrackDefinition result = make_track(
        "Generated open straight", {{0.0, 0.0}, {length_m / 3.0, 0.0},
                                     {2.0 * length_m / 3.0, 0.0}, {length_m, 0.0}},
        half_width_m);
    result.closed = false;
    result.sector_boundaries_fraction.clear();
    return result;
}

TrackDefinition make_oval_track() {
    return make_track("Generated oval", {{-120,-45},{-60,-60},{60,-60},{120,-45},{140,0},{120,45},{60,60},{-60,60},{-120,45},{-140,0}});
}

TrackDefinition make_hairpin_track() {
    return make_track("Generated hairpin projection test", {{0,0},{80,0},{120,10},{135,35},{120,60},{80,70},{20,70},{-10,55},{-20,30},{-10,5}});
}

TrackDefinition make_s_curve_track() {
    return make_track("Generated S technical loop", {{0,0},{70,-5},{125,25},{105,70},{45,85},{0,55},{-45,25},{-95,45},{-125,5},{-90,-40},{-25,-50}});
}

TrackDefinition make_technical_track() {
    return make_track("ApexLab Technical Test Circuit",
                      {{0,0},{120,0},{210,15},{270,65},{260,125},{210,155},{145,145},{105,105},
                       {80,65},{30,55},{-25,85},{-80,130},{-135,115},{-155,65},{-130,20},{-75,-10}}, 8.0);
}

} // namespace apexlab

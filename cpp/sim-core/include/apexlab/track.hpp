#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace apexlab {

inline constexpr int legacy_track_schema_version = 1;
inline constexpr int track_schema_version = 2;

struct Vec2 {
    double x{0.0};
    double y{0.0};
};

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct TrackControlPoint {
    Vec2 position;
    double elevation_m{0.0};
    double left_width_m{6.0};
    double right_width_m{6.0};
};

struct TrackDefinition {
    int schema_version{track_schema_version};
    std::string name;
    std::string provenance;
    bool closed{true};
    std::vector<TrackControlPoint> control_points;
    std::vector<double> sector_boundaries_fraction;
};

struct TrackFrame {
    double s_m{0.0};
    Vec2 position;
    Vec2 tangent;
    Vec2 left_normal;
    double heading_rad{0.0};
    double curvature_1_m{0.0};
    double left_width_m{0.0};
    double right_width_m{0.0};
    double elevation_m{0.0};
    double grade{0.0};
    double bank_angle_rad{0.0};
};

struct TrackCoordinate {
    double s_m{0.0};
    double lateral_offset_m{0.0}; // positive is left of the reference direction
    double distance_m{0.0};
};

class PeriodicTrack {
  public:
    explicit PeriodicTrack(TrackDefinition definition, std::size_t samples_per_segment = 256);

    [[nodiscard]] const TrackDefinition& definition() const { return definition_; }
    [[nodiscard]] double length_m() const { return length_m_; }
    [[nodiscard]] double wrap_s(double s_m) const;
    [[nodiscard]] TrackFrame frame_at_s(double s_m) const;
    [[nodiscard]] Vec2 position_at_s(double s_m, double lateral_offset_m = 0.0) const;
    [[nodiscard]] Vec3 position_3d_at_s(double s_m, double lateral_offset_m = 0.0) const;
    [[nodiscard]] bool contains(double s_m, double lateral_offset_m) const;
    [[nodiscard]] TrackCoordinate project(Vec2 point,
                                          std::optional<double> previous_s_m = std::nullopt,
                                          double search_window_m = 100.0) const;

  private:
    struct ArcSample {
        double parameter{0.0};
        double s_m{0.0};
        Vec2 position;
    };

    [[nodiscard]] Vec2 evaluate(double parameter) const;
    [[nodiscard]] Vec2 derivative(double parameter) const;
    [[nodiscard]] Vec2 second_derivative(double parameter) const;
    [[nodiscard]] double evaluate_elevation(double parameter) const;
    [[nodiscard]] double elevation_derivative(double parameter) const;
    [[nodiscard]] double parameter_at_s(double s_m) const;
    [[nodiscard]] TrackCoordinate refine_projection(Vec2 point, double parameter) const;

    TrackDefinition definition_;
    std::vector<ArcSample> arc_table_;
    double length_m_{0.0};
};

[[nodiscard]] TrackDefinition load_track_definition(const std::filesystem::path& path);
[[nodiscard]] TrackDefinition make_straight_test_path(double length_m = 200.0,
                                                      double half_width_m = 6.0);
[[nodiscard]] TrackDefinition make_circle_track(double radius_m, double half_width_m = 6.0,
                                                std::size_t control_point_count = 32);
[[nodiscard]] TrackDefinition make_oval_track();
[[nodiscard]] TrackDefinition make_hairpin_track();
[[nodiscard]] TrackDefinition make_s_curve_track();
[[nodiscard]] TrackDefinition make_technical_track();

} // namespace apexlab

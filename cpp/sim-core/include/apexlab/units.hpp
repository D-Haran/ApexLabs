#pragma once

#include <cmath>

namespace apexlab {

template <typename Tag> class Quantity {
  public:
    constexpr Quantity() = default;
    explicit constexpr Quantity(double value) : value_(value) {}

    [[nodiscard]] constexpr double value() const { return value_; }

    constexpr Quantity& operator+=(Quantity rhs) {
        value_ += rhs.value_;
        return *this;
    }

    constexpr Quantity& operator-=(Quantity rhs) {
        value_ -= rhs.value_;
        return *this;
    }

  private:
    double value_{0.0};
};

template <typename Tag>
[[nodiscard]] constexpr Quantity<Tag> operator+(Quantity<Tag> lhs, Quantity<Tag> rhs) {
    lhs += rhs;
    return lhs;
}

template <typename Tag>
[[nodiscard]] constexpr Quantity<Tag> operator-(Quantity<Tag> lhs, Quantity<Tag> rhs) {
    lhs -= rhs;
    return lhs;
}

template <typename Tag> [[nodiscard]] constexpr Quantity<Tag> operator-(Quantity<Tag> value) {
    return Quantity<Tag>{-value.value()};
}

template <typename Tag>
[[nodiscard]] constexpr Quantity<Tag> operator*(Quantity<Tag> quantity, double scalar) {
    return Quantity<Tag>{quantity.value() * scalar};
}

template <typename Tag>
[[nodiscard]] constexpr Quantity<Tag> operator*(double scalar, Quantity<Tag> quantity) {
    return quantity * scalar;
}

template <typename Tag>
[[nodiscard]] constexpr Quantity<Tag> operator/(Quantity<Tag> quantity, double scalar) {
    return Quantity<Tag>{quantity.value() / scalar};
}

template <typename Tag>
[[nodiscard]] constexpr bool operator==(Quantity<Tag> lhs, Quantity<Tag> rhs) {
    return lhs.value() == rhs.value();
}

struct TimeTag {};
struct DistanceTag {};
struct VelocityTag {};
struct AccelerationTag {};
struct MassTag {};
struct ForceTag {};
struct AngleTag {};
struct AngularVelocityTag {};
struct AngularAccelerationTag {};
struct MomentOfInertiaTag {};

using Time = Quantity<TimeTag>;
using Distance = Quantity<DistanceTag>;
using Velocity = Quantity<VelocityTag>;
using Acceleration = Quantity<AccelerationTag>;
using Mass = Quantity<MassTag>;
using Force = Quantity<ForceTag>;
using Angle = Quantity<AngleTag>;
using AngularVelocity = Quantity<AngularVelocityTag>;
using AngularAcceleration = Quantity<AngularAccelerationTag>;
using MomentOfInertia = Quantity<MomentOfInertiaTag>;

namespace si {

[[nodiscard]] constexpr Time seconds(double value) { return Time{value}; }
[[nodiscard]] constexpr Distance meters(double value) { return Distance{value}; }
[[nodiscard]] constexpr Velocity meters_per_second(double value) { return Velocity{value}; }
[[nodiscard]] constexpr Acceleration meters_per_second_squared(double value) {
    return Acceleration{value};
}
[[nodiscard]] constexpr Mass kilograms(double value) { return Mass{value}; }
[[nodiscard]] constexpr Force newtons(double value) { return Force{value}; }
[[nodiscard]] constexpr Angle radians(double value) { return Angle{value}; }
[[nodiscard]] constexpr AngularVelocity radians_per_second(double value) {
    return AngularVelocity{value};
}
[[nodiscard]] constexpr AngularAcceleration radians_per_second_squared(double value) {
    return AngularAcceleration{value};
}
[[nodiscard]] constexpr MomentOfInertia kilogram_square_meters(double value) {
    return MomentOfInertia{value};
}

} // namespace si

inline constexpr double standard_gravity_mps2 = 9.80665;

} // namespace apexlab

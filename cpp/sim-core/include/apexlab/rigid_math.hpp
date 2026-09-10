#pragma once

#include "apexlab/track.hpp"
#include <cmath>
#include <stdexcept>

namespace apexlab::rigid {
inline Vec3 add(Vec3 a, Vec3 b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
inline Vec3 sub(Vec3 a, Vec3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
inline Vec3 mul(Vec3 a, double s) { return {a.x*s, a.y*s, a.z*s}; }
inline double dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
inline double norm(Vec3 a) { return std::sqrt(dot(a,a)); }
inline bool finite(Vec3 a) { return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z); }
inline Vec3 unit(Vec3 a) {
    const double n=norm(a);
    if (!std::isfinite(n)||n<1e-12) throw std::domain_error("degenerate 3D direction");
    return mul(a,1/n);
}
// Hamilton quaternion; maps body (+x forward, +y left, +z up) to world (z up).
struct Quaternion { double w{1}, x{0}, y{0}, z{0}; };
inline Quaternion normalized(Quaternion q) {
    const double n=std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
    if (!std::isfinite(n)||n<1e-12) throw std::domain_error("invalid orientation quaternion");
    return {q.w/n,q.x/n,q.y/n,q.z/n};
}
inline Quaternion product(Quaternion a, Quaternion b) {
    return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
        a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
        a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
        a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
inline Vec3 rotate(Quaternion q, Vec3 v) {
    q=normalized(q);
    const Vec3 u{q.x,q.y,q.z};
    return add(v,add(mul(cross(u,v),2*q.w),mul(cross(u,cross(u,v)),2)));
}
inline Vec3 inverse_rotate(Quaternion q, Vec3 v) { return rotate({q.w,-q.x,-q.y,-q.z},v); }
inline Quaternion from_rotation_vector(Vec3 v) {
    const double angle=norm(v), scale=angle<1e-10?.5:std::sin(angle/2)/angle;
    return normalized({std::cos(angle/2),v.x*scale,v.y*scale,v.z*scale});
}
} // namespace apexlab::rigid

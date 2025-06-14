// math2d.hpp
#pragma once
#include <cmath>
#include <float.h>

struct vec2d {
    float v[2];

    vec2d() : v{0, 0} {}
    vec2d(float x, float y) : v{x, y} {}

    vec2d operator+(const vec2d& rhs) const {
        return vec2d(v[0] + rhs.v[0], v[1] + rhs.v[1]);
    }
    vec2d operator-(const vec2d& rhs) const {
        return vec2d(v[0] - rhs.v[0], v[1] - rhs.v[1]);
    }
    vec2d operator*(float scalar) const {
        return vec2d(v[0] * scalar, v[1] * scalar);
    }
    vec2d operator/(float scalar) const {
        if (scalar != 0)
            return vec2d(v[0] / scalar, v[1] / scalar);
        return vec2d((v[0] < 0 ? -FLT_MAX : FLT_MAX), (v[1] < 0 ? -FLT_MAX : FLT_MAX));
    }
    vec2d& operator+=(const vec2d& rhs) {
        v[0] += rhs.v[0]; v[1] += rhs.v[1]; return *this;
    }
    vec2d& operator-=(const vec2d& rhs) {
        v[0] -= rhs.v[0]; v[1] -= rhs.v[1]; return *this;
    }
    vec2d& operator*=(float scalar) {
        v[0] *= scalar; v[1] *= scalar; return *this;
    }
    vec2d& operator/=(float scalar) {
        if (scalar != 0) { v[0] /= scalar; v[1] /= scalar; }
        else { v[0] = (v[0] < 0 ? -FLT_MAX : FLT_MAX); v[1] = (v[1] < 0 ? -FLT_MAX : FLT_MAX); }
        return *this;
    }
    vec2d operator-() const {
        return vec2d(-v[0], -v[1]);
    }
    bool operator==(const vec2d& rhs) const {
        return v[0] == rhs.v[0] && v[1] == rhs.v[1];
    }
    bool operator!=(const vec2d& rhs) const {
        return !(*this == rhs);
    }
    float length() const {
        return std::sqrt(v[0] * v[0] + v[1] * v[1]);
    }
    vec2d normalized() const {
        float len = length();
        return len != 0 ? *this / len : vec2d(0, 0);
    }
    float dot(const vec2d& rhs) const {
        return v[0] * rhs.v[0] + v[1] * rhs.v[1];
    }
    float cross(const vec2d& rhs) const {
        return v[0] * rhs.v[1] - v[1] * rhs.v[0];
    }
};

struct mat2d {
    float m[2][2];

    mat2d() : m{{0, 0}, {0, 0}} {}
    mat2d(float a, float b, float c, float d) : m{{a, b}, {c, d}} {}

    static mat2d from_rvec(const vec2d& a, const vec2d& b) {
        return mat2d(a.v[0], a.v[1], b.v[0], b.v[1]);
    }

    static mat2d from_cvec(const vec2d& a, const vec2d& b) {
        return mat2d(a.v[0], b.v[0], a.v[1], b.v[1]);
    }

    mat2d operator*(const mat2d& rhs) const {
        return mat2d(
            m[0][0] * rhs.m[0][0] + m[0][1] * rhs.m[1][0],
            m[0][0] * rhs.m[0][1] + m[0][1] * rhs.m[1][1],
            m[1][0] * rhs.m[0][0] + m[1][1] * rhs.m[1][0],
            m[1][0] * rhs.m[0][1] + m[1][1] * rhs.m[1][1]
        );
    }
    vec2d operator*(const vec2d& v) const {
        return vec2d(
            m[0][0] * v.v[0] + m[0][1] * v.v[1],
            m[1][0] * v.v[0] + m[1][1] * v.v[1]
        );
    }
    mat2d operator*(float scalar) const {
        return mat2d(
            m[0][0] * scalar, m[0][1] * scalar,
            m[1][0] * scalar, m[1][1] * scalar
        );
    }
    mat2d operator/(float scalar) const {
        if (scalar != 0) {
            return mat2d(
                m[0][0] / scalar, m[0][1] / scalar,
                m[1][0] / scalar, m[1][1] / scalar
            );
        } else {
            return mat2d(
                (m[0][0] < 0 ? -FLT_MAX : FLT_MAX), (m[0][1] < 0 ? -FLT_MAX : FLT_MAX),
                (m[1][0] < 0 ? -FLT_MAX : FLT_MAX), (m[1][1] < 0 ? -FLT_MAX : FLT_MAX)
            );
        }
    }
    mat2d operator+(const mat2d& rhs) const {
        return mat2d(
            m[0][0] + rhs.m[0][0], m[0][1] + rhs.m[0][1],
            m[1][0] + rhs.m[1][0], m[1][1] + rhs.m[1][1]
        );
    }
    mat2d operator-(const mat2d& rhs) const {
        return mat2d(
            m[0][0] - rhs.m[0][0], m[0][1] - rhs.m[0][1],
            m[1][0] - rhs.m[1][0], m[1][1] - rhs.m[1][1]
        );
    }
    mat2d& operator+=(const mat2d& rhs) {
        m[0][0] += rhs.m[0][0]; m[0][1] += rhs.m[0][1];
        m[1][0] += rhs.m[1][0]; m[1][1] += rhs.m[1][1];
        return *this;
    }
    mat2d& operator-=(const mat2d& rhs) {
        m[0][0] -= rhs.m[0][0]; m[0][1] -= rhs.m[0][1];
        m[1][0] -= rhs.m[1][0]; m[1][1] -= rhs.m[1][1];
        return *this;
    }
    mat2d operator-() const {
        return mat2d(-m[0][0], -m[0][1], -m[1][0], -m[1][1]);
    }
    bool operator==(const mat2d& rhs) const {
        return m[0][0] == rhs.m[0][0] && m[0][1] == rhs.m[0][1] &&
               m[1][0] == rhs.m[1][0] && m[1][1] == rhs.m[1][1];
    }
    bool operator!=(const mat2d& rhs) const {
        return !(*this == rhs);
    }
    float determinant() const {
        return m[0][0] * m[1][1] - m[0][1] * m[1][0];
    }
    mat2d inverse() const {
        float d = determinant();
        if (d == 0)
            return mat2d();
        return mat2d(
            m[1][1] / d, -m[0][1] / d,
            -m[1][0] / d, m[0][0] / d
        );
    }
    mat2d transposed() const {
        return mat2d(m[0][0], m[1][0], m[0][1], m[1][1]);
    }
};

inline vec2d solve(const mat2d& m, const vec2d& s) {
    float d = m.determinant();
    if (d == 0) return vec2d(0, 0);
    return vec2d(
        (m.m[1][1] * s.v[0] - m.m[0][1] * s.v[1]) / d,
        (m.m[0][0] * s.v[1] - m.m[1][0] * s.v[0]) / d
    );
}

inline vec2d rotate_ccw_90(const vec2d& v) {
    return vec2d(-v.v[1], v.v[0]);
}

inline vec2d rotate_cw_90(const vec2d& v) {
    return vec2d(v.v[1], -v.v[0]);
}

inline vec2d rotate_ccw(const vec2d& v, float angle) {
    float cos_a = std::cos(angle);
    float sin_a = std::sin(angle);
    return vec2d(
        v.v[0] * cos_a - v.v[1] * sin_a,
        v.v[0] * sin_a + v.v[1] * cos_a
    );
}

inline vec2d rotate_cw(const vec2d& v, float angle) {
    float cos_a = std::cos(angle);
    float sin_a = std::sin(angle);
    return vec2d(
        v.v[0] * cos_a + v.v[1] * sin_a,
        -v.v[0] * sin_a + v.v[1] * cos_a
    );
}

inline mat2d rotation_matrix_ccw(float angle) {
    float cos_a = std::cos(angle);
    float sin_a = std::sin(angle);
    return mat2d(cos_a, -sin_a, sin_a, cos_a);
}

inline mat2d rotation_matrix_cw(float angle) {
    float cos_a = std::cos(angle);
    float sin_a = std::sin(angle);
    return mat2d(cos_a, sin_a, -sin_a, cos_a);
}

float angle_norm(const vec2d& a, const vec2d& b) {
    // Both vectors are already normalized
    float angle = std::acos(a.dot(b));
    if (a.cross(b) > 0) {
        return angle;
    }
    else {
        return -angle;
    }
}

float angle(const vec2d& a, const vec2d& b) {
    return angle_norm(a.normalized(), b.normalized());
}

float angle_norm(const vec2d& a) {
    float angle = std::acos(a.v[0]);
    if (a.v[1] > 0) {
        return angle;
    }
    else {
        return -angle;
    }
}

float angle(const vec2d& a) {
    return angle_norm(a.normalized());
}
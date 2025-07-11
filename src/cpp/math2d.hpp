// math2d.hpp
#pragma once
#include "pycam.hpp"
#include <cmath>
#include <float.h>
#include <sstream>
#include <iomanip>

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
        return len != 0 ? vec2d(v[0] / len, v[1] / len) : vec2d(0, 0);
    }
    float dot(const vec2d& rhs) const {
        return v[0] * rhs.v[0] + v[1] * rhs.v[1];
    }
    float cross(const vec2d& rhs) const {
        return v[0] * rhs.v[1] - v[1] * rhs.v[0];
    }
    bool close(const vec2d& rhs) const {
        return std::abs(v[0] - rhs.v[0]) < POS_1D_TOL && std::abs(v[1] - rhs.v[1]) < POS_1D_TOL;
    }
    
    std::string to_string() const {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << "(" << v[0] << ", " << v[1] << ")";
        return ss.str();
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

struct bounding_box {
    float xmin;
    float xmax;
    float ymin;
    float ymax;

    bounding_box(float xmin, float xmax, float ymin, float ymax)
        : xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax) {}
    bounding_box() : xmin(0), xmax(0), ymin(0), ymax(0) {}
    bool intersects(const bounding_box &other) const;
    bool intersects_x(const bounding_box &other) const {
        return (xmin <= other.xmax && xmax >= other.xmin);
    }
    bool intersects_y(const bounding_box &other) const {
        return (ymin <= other.ymax && ymax >= other.ymin);
    }
    void expand(float x, float y) {
        xmin -= x;
        xmax += x;
        ymin -= y;
        ymax += y;
    }
    void translate(float x, float y) {
        xmin += x;
        xmax += x;
        ymin += y;
        ymax += y;
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

inline float angle_norm(const vec2d& a, const vec2d& b) {
    // Both vectors are already normalized
    float angle = std::max(-1.0f, std::min(1.0f, a.dot(b))); // Clamp to avoid NaN
    angle = std::acos(angle);
    if (a.cross(b) > 0) {
        return angle;
    }
    else {
        return -angle;
    }
}

inline float angle(const vec2d& a, const vec2d& b) {
    return angle_norm(a.normalized(), b.normalized());
}

inline float angle_norm(const vec2d& a) {
    float angle = std::max(-1.0f, std::min(1.0f, a.v[0])); // Clamp to avoid NaN
    angle = std::acos(angle);
    if (a.v[1] > 0) {
        return angle;
    }
    else {
        return -angle;
    }
}

inline float angle(const vec2d& a) {
    return angle_norm(a.normalized());
}

inline double modulo(double x, double m) {
    return (x - std::floor(x / m) * m);
}

inline float modulo(float x, float m) {
    return (x - std::floor(x / m) * m);
}

inline int modulo(int x, int m) {
    return (x % m + m) % m; // Ensure non-negative result
}

template <typename T>
T sgn(T x) {
    if (x>0) return 1.0f;
    if (x<0) return -1.0f;
    return 0.0f;
}
#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>
#include <array>
#include "math2d.hpp"
#include "pycam.hpp"

struct compact_point {
    float v[3]; // x, y, bulge

    compact_point(float x, float y, float bulge) {
        v[0] = x; v[1] = y; v[2] = bulge;
    }
    compact_point() : v{0, 0, 0} {}
    float& operator[](size_t i) { return v[i]; }
    const float& operator[](size_t i) const { return v[i]; }
};


class line_segment;
class arc_segment;

class segment {
public:
    friend class line_segment;
    friend class arc_segment;
    friend class path;

    vec2d start;
    vec2d end;

public:
    virtual ~segment() = default;
    segment() {};
    segment(const vec2d& start, const vec2d& end) : start(start), end(end) {}

    virtual segment* clone() const = 0;

    virtual vec2d get_nhat_start() const = 0;
    virtual vec2d get_nhat_end() const = 0;

    virtual bool offset(float distance) = 0;
    virtual bounding_box get_bounding_box() const = 0;

    virtual compact_point to_compact_point() const = 0;

    virtual bool diverges(const segment& other, float direction) const = 0;
    virtual bool diverges_from_line(const line_segment& first, float direction) const = 0;
    virtual bool diverges_from_arc(const arc_segment& first, float direction) const = 0;

    virtual vec2d intersection(const segment& other) const = 0;
    virtual vec2d intersection_with_line(const line_segment& line, bool arg_first) const = 0;
    virtual vec2d intersection_with_arc(const arc_segment& arc, bool arg_first) const = 0;
    
    virtual float trap_area() const = 0;

    result<vec2d> intersects(const segment &other) const;

    virtual bool set_start_point(const vec2d& point) = 0;
    virtual bool set_end_point(const vec2d& point) = 0;
    virtual bool on_segment(const vec2d& point) const = 0;

    virtual segment* bisect(const vec2d& point, bool before) const {
        // Default implementation for segments that can be split
        // This is a fallback and may not be valid for all segment types.
        // Derived classes should override this method if they can split at a point.
        // if (!on_segment(point)) {
        //     return nullptr; // Cannot split if the point is not on the segment
        // }

        // Calculate the new segments
        segment* seg = clone();
        if (before) {
            seg->set_end_point(point);
            return seg;
        }
        else {
            seg->set_start_point(point);
            return seg;
        }
    }
};

class line_segment : public segment {
public:
    vec2d nhat;
    vec2d vhat;
    float s;

public:
    line_segment(const vec2d& start, const vec2d& end);

    vec2d get_nhat_start() const { return nhat; }
    vec2d get_nhat_end() const { return nhat; }

    segment* clone() const override {
        return new line_segment(start, end);
    }

    static line_segment* from_compact_point(const compact_point& p0, const compact_point& p1) {
        return new line_segment(vec2d(p0[0], p0[1]), vec2d(p1[0], p1[1]));
    }

    compact_point to_compact_point() const override {
        return {start.v[0], start.v[1], 0};
    }

    bool offset(float distance) override;
    bounding_box get_bounding_box() const override {
        return bounding_box(
            std::min(start.v[0], end.v[0]),
            std::max(start.v[0], end.v[0]),
            std::min(start.v[1], end.v[1]),
            std::max(start.v[1], end.v[1])
        );
    }

    bool diverges(const segment& other, float direction) const override;
    bool diverges_from_line(const line_segment& first, float direction) const override;
    bool diverges_from_arc(const arc_segment& first, float direction) const override;

    vec2d intersection(const segment& other) const override;
    vec2d intersection_with_line(const line_segment& other, bool arg_first) const override;
    vec2d intersection_with_arc(const arc_segment& arc, bool arg_first) const override;

    float trap_area() const override;
protected:
    bool set_start_point(const vec2d& point) override;
    bool set_end_point(const vec2d& point) override;

    bool on_segment(const vec2d& point) const override;
};

class arc_segment : public segment {
protected:
    arc_segment() = default;
    arc_segment(const vec2d& start, const vec2d& end, const vec2d& center, const vec2d& nhat_start, const vec2d& nhat_end, float radius, float start_angle, float end_angle)
        : segment(start, end), center(center), nhat_start(nhat_start), nhat_end(nhat_end), radius(radius), start_angle(start_angle), end_angle(end_angle) {}

public:
    vec2d center;
    vec2d nhat_start;
    vec2d nhat_end;
    float radius;
    float start_angle;
    float end_angle;

public:    
    vec2d get_nhat_start() const { return nhat_start; }
    vec2d get_nhat_end() const { return nhat_end; }

    static arc_segment* from_compact_point(const compact_point& p0, const compact_point& p1);
    static arc_segment* arc1(const vec2d & center, const vec2d & point, float angle);
    static arc_segment* arc2(const vec2d & point1, const vec2d & point2, float radius, bool is_clockwise); // Three point arc segment
    static arc_segment* arc3(const vec2d & point1, const vec2d & point2, float angle);
    static arc_segment* arc4(const vec2d & point1, const vec2d & point2, float bulge);
    static arc_segment* arc5(const vec2d & point1, const vec2d & point2, const vec2d & point3);

    bool is_clockwise() const { return radius > 0 ? end_angle < start_angle : end_angle > start_angle; }

    segment* clone() const override {
        arc_segment *temp = new arc_segment(start, end, center, nhat_start, nhat_end, radius, start_angle, end_angle);
        return temp;
    }

    float bulge() const { return std::tan((end_angle - start_angle) / 4.0f); };
    compact_point to_compact_point() const override { return {start.v[0], start.v[1], bulge()}; };
    

    bool offset(float distance) override;
    bounding_box get_bounding_box() const override {
        return bounding_box(
            center.v[0] - std::abs(radius), center.v[0] + std::abs(radius),
            center.v[1] - std::abs(radius), center.v[1] + std::abs(radius)
        );
    };

    bool diverges(const segment& other, float direction) const override;
    bool diverges_from_line(const line_segment& line, float direction) const override;
    bool diverges_from_arc(const arc_segment& other, float direction) const override;

    vec2d intersection(const segment& other) const override;
    vec2d intersection_with_line(const line_segment& line, bool arg_first) const override;
    vec2d intersection_with_arc(const arc_segment& other, bool arg_first) const override;

    float trap_area() const override;

protected:
    bool set_start_point(const vec2d& point) override;
    bool set_end_point(const vec2d& point) override;

    bool on_segment(const vec2d& point) const override;
};

struct bb_index {
    bounding_box box;
    segment *seg;
    bool start;
};


class path {

public:
    std::vector<segment*> segments;
    
    path() = default;
    ~path() {
        for (auto seg : segments) {
            delete seg;
        }
    }

    void add_segment(segment* seg) {
        segments.push_back(seg);
    }

    void insert_segment(size_t index, segment* seg) {
        if (index < segments.size()) {
            segments.insert(segments.begin() + index, seg);
        } else {
            segments.push_back(seg);
        }
    }

    bool clockwise_winding() const { return signed_area() > 0; }

    path* offset(float distance, bool arc_join);

    static path* from_compact_array(const std::vector<compact_point> &cp, bool c);
    std::vector<compact_point> to_compact_array() const;

    std::vector<path*> get_closed_loops();

    float signed_area() const;
};

inline line_segment* as_line(segment* seg) {
    return dynamic_cast<line_segment*>(seg);
}
inline arc_segment* as_arc(segment* seg) {
    return dynamic_cast<arc_segment*>(seg);
}

inline float arc_integral_top(float x0, float x1, float r, float h)
{
    float r2 = r * r;
    float area = h * (x1 - x0); // Rectangular contribution
    float a;
    float nr = -r;
    
    if (x1 <= nr) {
        area += -r2 * M_PI_4; // Full semicircle
    }
    else if (x1 >= r) {
        area += r2 * M_PI_4; // Full semicircle
    }
    else {
        a = std::sqrt(r2-x1*x1);
        area += 0.5 * (x1 * a + r2*std::atan2(x1, a));
    }

    if (x0 <= nr) {
        area -= -r2 * M_PI_4; // Full semicircle
    }
    else if (x0 >= r) {
        area -= r2 * M_PI_4; // Full semicircle
    }
    else {
        a = std::sqrt(r2-x0*x0);
        area -= 0.5 * (x0 * a + r2*std::atan2(x0, a));
    }

    return area;
}

inline float arc_integral_bot(float x0, float x1, float r, float h)
{
    float r2 = r * r;
    float area = h * (x1 - x0); // Rectangular contribution
    float a;
    float nr = -r;
    
    if (x1 <= nr) {
        area -= -r2 * M_PI_4; // Full semicircle
    }
    else if (x1 >= r) {
        area -= r2 * M_PI_4; // Full semicircle
    }
    else {
        a = std::sqrt(r2-x1*x1);
        area -= 0.5 * (x1 * a + r2*std::atan2(x1, a));
    }

    if (x0 <= nr) {
        area += -r2 * M_PI_4; // Full semicircle
    }
    else if (x0 >= r) {
        area += r2 * M_PI_4; // Full semicircle
    }
    else {
        a = std::sqrt(r2-x0*x0);
        area += 0.5 * (x0 * a + r2*std::atan2(x0, a));
    }

    return area;
}

#endif
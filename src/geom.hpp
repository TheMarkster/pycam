#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>
#include <array>
#include "math2d.hpp"

struct compact_point {
    float v[3]; // x, y, bulge

    compact_point(float x, float y, float bulge) {
        v[0] = x; v[1] = y; v[2] = bulge;
    }
    compact_point() : v{0, 0, 0} {}
    float& operator[](size_t i) { return v[i]; }
    const float& operator[](size_t i) const { return v[i]; }
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

class line_segment;
class arc_segment;

class segment {
public:
    vec2d start;
    vec2d end;

public:
    virtual ~segment() = default;
    segment() {};
    segment(const vec2d& start, const vec2d& end) : start(start), end(end) {}

    virtual void offset(float distance) = 0;
    virtual bounding_box get_bounding_box() const = 0;

    virtual compact_point to_compact_point() const = 0;

    virtual bool diverges(const segment& other, float direction) const = 0;
    virtual bool diverges_from_line(const line_segment& first, float direction) const = 0;
    virtual bool diverges_from_arc(const arc_segment& first, float direction) const = 0;

    virtual vec2d intersection(const segment& other) const = 0;
    virtual vec2d intersection_with_line(const line_segment& line, bool arg_first) const = 0;
    virtual vec2d intersection_with_arc(const arc_segment& arc, bool arg_first) const = 0;
    
    virtual bool on_segment(const vec2d& point) const = 0;
    bool intersects(const segment& other) const;
};

class line_segment : public segment {
public:
    vec2d nhat;
    vec2d vhat;
    float s;

public:
    line_segment(const vec2d& start, const vec2d& end);

    static line_segment* from_compact_point(const compact_point& p0, const compact_point& p1) {
        return new line_segment(vec2d(p0[0], p0[1]), vec2d(p1[0], p1[1]));
    }

    compact_point to_compact_point() const override {
        return {start.v[0], start.v[1], 0};
    }

    void offset(float distance) override;
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

private:
    bool on_segment(const vec2d& point) const override;
};

class arc_segment : public segment {
public:
    vec2d center;
    vec2d nhat_start;
    vec2d nhat_end;
    float radius;
    float start_angle;
    float end_angle;
    bool is_clockwise;

public:
    arc_segment(const vec2d& center, float radius, float start_angle, float end_angle, bool is_clockwise);

    static arc_segment* from_compact_point(const compact_point& p0, const compact_point& p1);

    float bulge() const { return std::tan((end_angle - start_angle) / 4.0f); };
    compact_point to_compact_point() const override { return {start.v[0], start.v[1], bulge()}; };
    

    void offset(float distance) override;
    bounding_box get_bounding_box() const override {
        return bounding_box(
            center.v[0] - radius, center.v[0] + radius,
            center.v[1] - radius, center.v[1] + radius
        );
    };

    bool diverges(const segment& other, float direction) const override;
    bool diverges_from_line(const line_segment& line, float direction) const override;
    bool diverges_from_arc(const arc_segment& other, float direction) const override;

    vec2d intersection(const segment& other) const override;
    vec2d intersection_with_line(const line_segment& line, bool arg_first) const override;
    vec2d intersection_with_arc(const arc_segment& other, bool arg_first) const override;

private:
    bool on_segment(const vec2d& point) const override;
};

struct bb_index {
    bounding_box box;
    segment *seg;
    bool start;
};

class path {
    std::vector<segment*> segments;

public:
    path() = default;

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

    path* offset(float distance, bool arc_join);

    static path* from_compact_array(const std::vector<compact_point> &cp);
    std::vector<compact_point> to_compact_array() const;
    std::vector<compact_point> find_intersections();
};

#endif
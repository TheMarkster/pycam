#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>
#include "math2d.hpp"


struct bounding_box {
    float xmin;
    float xmax;
    float ymin;
    float ymax;

    bounding_box(float xmin, float xmax, float ymin, float ymax)
        : xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax) {}
    bounding_box() : xmin(0), xmax(0), ymin(0), ymax(0) {}
};

class line_segment;
class arc_segment;

class segment {
protected:
    vec2d start;
    vec2d end;

public:
    segment(const vec2d& start, const vec2d& end) : start(start), end(end) {}

    virtual void offset(float distance) = 0;
    virtual bounding_box get_bounding_box() const = 0;

    virtual bool diverges(const segment& other, float direction) const = 0;
    virtual bool diverges_from_line(const line_segment& first, float direction) const = 0;
    virtual bool diverges_from_arc(const arc_segment& first, float direction) const = 0;

    virtual vec2d intersection(const segment& other) const = 0;
    virtual vec2d intersection_with_line(const line_segment& first) const = 0;
    virtual vec2d intersection_with_arc(const arc_segment& first) const = 0;
};

class line_segment : public segment {
    friend class arc_segment;

    vec2d nhat;
    vec2d vhat;
    float s;
public:
    line_segment(const vec2d& start, const vec2d& end);

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
    vec2d intersection_with_line(const line_segment& first) const override;
    vec2d intersection_with_arc(const arc_segment& first) const override;
};

class arc_segment : public segment {
    friend class line_segment;

    vec2d center;
    vec2d nhat_start;
    vec2d nhat_end;
    float radius;
    float start_angle;
    float end_angle;
    bool is_clockwise;

public:
    arc_segment(const vec2d& center, float radius, float start_angle, float end_angle, bool is_clockwise);

    void offset(float distance) override;
    bounding_box get_bounding_box() const override {
        return bounding_box(
            center.v[0] - radius, center.v[0] + radius,
            center.v[1] - radius, center.v[1] + radius
        );
    }

    bool diverges(const segment& other, float direction) const override;
    bool diverges_from_line(const line_segment& first, float direction) const override;
    bool diverges_from_arc(const arc_segment& first, float direction) const override;

    vec2d intersection(const segment& other) const override;
    vec2d intersection_with_line(const line_segment& first) const override;
    vec2d intersection_with_arc(const arc_segment& first) const override;
};

struct bb_index {
    bounding_box box;
    segment *seg;
    bool start;
};


class path {
    std::vector<segment*> segments;

public:

    void find_intersections();
};





#endif
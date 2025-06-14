#include "geom.hpp"
#include "math2d.hpp"
#include "sorting.hpp"
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool bounding_box::intersects(const bounding_box &other) const {
    float _xmin = std::max(other.xmin, xmin);
    float _xmax = std::min(other.xmax, xmax);
    if (_xmin >= _xmax) return false; // No intersection in x-axis

    float _ymin = std::max(other.ymin, ymin);
    float _ymax = std::min(other.ymax, ymax);
    if (_ymin >= _ymax) return false; // No intersection in y-axis

    return true;
}

line_segment::line_segment(const vec2d& start, const vec2d& end)
    : segment(start, end) {
    vhat = (end - start).normalized();
    nhat = rotate_ccw_90(vhat); // Perpendicular vector
    s = nhat.dot(start);
}

void line_segment::offset(float distance) {
    vec2d offset_vector = nhat * distance;
    start += offset_vector;
    end += offset_vector;
    s += distance;
}

bool line_segment::diverges(const segment& other, float distance) const {
    // Check if the other segment diverges from this line segment
    return other.diverges_from_line(*this, distance);
}

bool line_segment::diverges_from_line(const line_segment& first, float direction) const {
    float cross = first.vhat.cross(vhat) * direction;
    return cross < 0;
}

bool line_segment::diverges_from_arc(const arc_segment& first, float direction) const {
    return first.nhat_end.cross(nhat) * direction * first.radius < 0;
}

vec2d line_segment::intersection(const segment& other) const {
    // Find intersection with another segment
    return other.intersection_with_line(*this, true);
}

vec2d line_segment::intersection_with_line(const line_segment& other, bool arg_first) const {
    // Calculate intersection point with another line segment
    mat2d m = mat2d::from_rvec(nhat, other.nhat);
    vec2d s_vec = vec2d(s, other.s);
    return solve(m, s_vec);
}

vec2d line_segment::intersection_with_arc(const arc_segment& arc, bool arc_first) const {
    // Implement logic to find intersection with an arc segment

    // Intersect vector with normal going through arc's center
    // This is the midpoint of possible intersections
    mat2d m = mat2d::from_rvec(this->nhat, this->vhat);
    vec2d sol = vec2d(this->s, this->vhat.dot(arc.center));
    vec2d intersection_point = solve(m, sol);

    // Distance from midpoint to solutions
    float a = (intersection_point - arc.center).length();
    float b = pow(arc.radius,2) - pow(a,2);
    if (abs(b) < 1e-6) {
        // Midpoint is the intersection point
        return intersection_point;
    } else if (b < 0) {
        // No intersection
        return vec2d(0, 0); // Placeholder for no intersection
    }
    b = sqrt(b);

    // Use closest solution
    // Find offset vector and reduce length by b
    vec2d offset_vector;
    if (arc_first) {
        offset_vector = intersection_point - this->end;
    }
    else {
        offset_vector = intersection_point - this->start;
    }
    float distance = offset_vector.length();
    offset_vector *= (distance - b) / distance;
    
    if (arc_first) {
        return this->end + offset_vector;
    } else {
        return this->start + offset_vector;
    }
}

arc_segment::arc_segment(const vec2d& center, float radius, float start_angle, float end_angle, bool is_clockwise) : 
    center(center), radius(radius), start_angle(start_angle), end_angle(end_angle), is_clockwise(is_clockwise) {
    nhat_start = vec2d(std::cos(start_angle), std::sin(start_angle));
    nhat_end = vec2d(std::cos(end_angle), std::sin(end_angle));
    start = center + nhat_start * radius;
    end = center + nhat_end * radius;
}

arc_segment* arc_segment::from_compact_point(const compact_point& p0, const compact_point& p1) {
        vec2d start = vec2d(p0[0], p0[1]);
        vec2d end = vec2d(p1[0], p1[1]);
        vec2d diff = (start - end)/2;
        float d = diff.length();
        vec2d center = start + diff;
        float delta_angle = std::atan(p0[2])*4.0f; // Convert bulge to angle
        float r = d / std::sin(delta_angle / 2.0f);
        vec2d nhat;
        if (r > 0) {
            nhat = rotate_cw_90(diff.normalized());
        }
        else {
            nhat = rotate_ccw_90(diff.normalized());
        }
        vec2d mp = center + nhat * std::sqrt(r * r - d * d);
        float start_angle = angle(start);
        float end_angle = angle(end);
        if (r > 0) {
            // Clockwise arc; angle decrease from start to end
            if (end_angle > start_angle) {
                end_angle -= 2 * M_PI; // Adjust for clockwise direction
            }
            return new arc_segment(mp, r, start_angle, end_angle, true);
        }
        else {
            // Counter-clockwise arc; angle increases from start to end
            if (end_angle < start_angle) {
                end_angle += 2 * M_PI; // Adjust for counter-clockwise direction
            }
            return new arc_segment(mp, r, start_angle, end_angle, false);
        }
    }

void arc_segment::offset(float distance) {
    radius += distance;
    start = center + nhat_start * radius;
    end = center + nhat_end * radius;
}

bool arc_segment::diverges(const segment& other, float direction) const {
    // Check if the other segment diverges from this arc segment
    return other.diverges_from_arc(*this, direction);
}

bool arc_segment::diverges_from_line(const line_segment& first, float direction) const {
    // Check if the arc diverges from a line segment
    return first.nhat.cross(nhat_start) * direction * radius < 0;
}

bool arc_segment::diverges_from_arc(const arc_segment& first, float direction) const {
    // Check if the arc diverges from another arc segment
    return first.nhat_end.cross(nhat_start) * direction * first.radius * radius < 0;
}

vec2d arc_segment::intersection(const segment& other) const {
    // Find intersection with another segment
    return other.intersection_with_arc(*this, true);
}

vec2d arc_segment::intersection_with_line(const line_segment& line, bool arg_first) const {
    return line.intersection_with_arc(*this, !arg_first);
}

vec2d arc_segment::intersection_with_arc(const arc_segment& other, bool arg_first) const {
    // Find midpoint of solutions
    vec2d v = other.center - center;
    float vlen2 = v.dot(v);
    float vlen = std::sqrt(vlen2);
    v = v / vlen; // Normalize vector
    float mp_distance = (pow(radius, 2) - pow(other.radius, 2) + vlen2) / (2 * vlen);
    vec2d midpoint = center + v * (mp_distance / vlen);
    float solution_distance = std::sqrt(pow(radius, 2) - pow(mp_distance, 2));

    // Determine the direction of the solution
    vec2d reference_vector;
    if (arg_first) {
        reference_vector = other.end;
    } else {
        reference_vector = end;
    }
    reference_vector -= center;

    vec2d n;
    if (v.cross(reference_vector) > 0) {
        // Solution is counter-clockwise
        n = rotate_ccw_90(v);
    }
    else {
        // Solution is clockwise
        n = rotate_cw_90(v);
    }

    return midpoint + n * solution_distance;
}

bool line_segment::on_segment(const vec2d& point) const {
    // Check if the point is on the line segment
    vec2d start_to_point = point - start;
    vec2d end_to_point = point - end;
    return (start_to_point.dot(vhat) >= 0 && end_to_point.dot(vhat) <= 0);
}

bool arc_segment::on_segment(const vec2d& point) const {
    // We assume the point is on the circle
    // Verify it's an angle on the arc segment
    vec2d center_to_point = (point - center).normalized();
    float angle = std::acos(center_to_point.v[0]) * (center_to_point.v[1] >= 0 ? 1 : -1); // Get angle in range [-pi, pi]
    
    if (is_clockwise) {
        if (angle < start_angle) {
            angle += 2 * M_PI; // Adjust angle to be in range [start_angle, end_angle]
        }
        return angle <= end_angle;
    }
    else {
        if (angle > start_angle) {
            angle -= 2 * M_PI; // Adjust angle to be in range [end_angle, start_angle]
        }
        return angle >= end_angle;
    }
}

bool segment::intersects(const segment& other) const {
    vec2d intersection_point = intersection(other);
    return on_segment(intersection_point) && other.on_segment(intersection_point);
}

path* path::from_compact_array(const std::vector<compact_point> &data) {
    // Allocate segments
    path *p = new path();
    size_t n = data.size();
    p->segments.reserve(n);
    compact_point prev = data[n-1];

    size_t i;
    compact_point point;
    for (i=0; i<data.size(); i++) {
        point = data[i];
        if (prev[2] == 0) {
            // Line segment
            p->segments.push_back(line_segment::from_compact_point(prev, point));
        } else {
            // Arc segment
            p->segments.push_back(arc_segment::from_compact_point(prev, point));
        }
        prev = point;
    }
    return p;
}

std::vector<compact_point> path::to_compact_array() const {
    std::vector<compact_point> data;
    size_t i = 0;
    for (i=0; i<segments.size(); i++) {
        data.push_back(segments[i]->to_compact_point());
    }
    return data;
}

std::vector<compact_point> path::find_intersections() {
    // std::vector<vec2d> points;
    // // TODO: Implement intersection finding logic
    // compact_point *data = 
    // std::span<compact_point> result;
    // result.data = new compact_point[points.size()];
    // result.size = points.size();
    // for (size_t i = 0; i < points.size(); i++) {
    //     result.data[i] = {points[i].v[0], points[i].v[1], 0}; // Bulge is set to 0 for intersections
    // }
    std::vector<compact_point> points;
    return points;
}
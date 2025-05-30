#include "geom.hpp"
#include "math2d.hpp"
#include "sorting.hpp"


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
    return other.intersection_with_line(*this);
}

vec2d line_segment::intersection_with_line(const line_segment& first) const {
    // Calculate intersection point with another line segment
    mat2d m = mat2d::from_rvec(nhat, first.nhat);
    vec2d s_vec = vec2d(s, first.s);
    return solve(m, s_vec);
}

vec2d line_segment::intersection_with_arc(const arc_segment& first) const {
    // Implement logic to find intersection with an arc segment
    return vec2d(0, 0); // Placeholder
}

arc_segment::arc_segment(const vec2d& center, float radius, float start_angle, float end_angle, bool is_clockwise) : 
    center(center), radius(radius), start_angle(start_angle), end_angle(end_angle), is_clockwise(is_clockwise) {
    nhat_start = vec2d(std::cos(start_angle), std::sin(start_angle));
    nhat_end = vec2d(std::cos(end_angle), std::sin(end_angle));
    start = center + nhat_start * radius;
    end = center + nhat_end * radius;
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
    return other.intersection_with_arc(*this);
}

vec2d arc_segment::intersection_with_line(const line_segment& first) const {
    // Implement logic to find intersection with a line segment
    return vec2d(0, 0); // Placeholder
}

vec2d arc_segment::intersection_with_arc(const arc_segment& first) const {
    // Implement logic to find intersection with another arc segment
    return vec2d(0, 0); // Placeholder
}

void path::find_intersections() {
    std::vector<sort_item<float, segment*>> items;

    for (auto& seg : segments) {
        bounding_box box = seg->get_bounding_box();
        items.push_back({box.min_x, seg});
        items.push_back({box.max_x, seg});
    }
}
#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>
#include <array>
#include "math2d.hpp"
#include "pycam.hpp"
#include <sstream>
#include <iomanip>
#include <iostream>

struct compact_point {
    float v[3]; // x, y, bulge

    compact_point(float x, float y, float bulge) {
        v[0] = x; v[1] = y; v[2] = bulge;
    }
    compact_point() : v{0, 0, 0} {}
    float& operator[](size_t i) { return v[i]; }
    const float& operator[](size_t i) const { return v[i]; }
};

struct intersection_info {
    vec2d point;
    float pos1;
    float pos2;
};

typedef varray<vec2d, 2> intersection_result;
typedef varray<intersection_info, 2> validated_intersection_result;

class line_segment;
class arc_segment;

class segment {
public:
    vec2d start;
    vec2d end;

    // Constructors/copiers/destructors
    virtual ~segment() = default;
    segment() {};
    segment(const vec2d& start, const vec2d& end) : start(start), end(end) {}

    virtual segment* copy() const = 0;

    segment* bisect(const vec2d& point, bool before) const {
        // Calculate the new segments
        segment* seg;
        if (before) {            
            seg = copy(); // Create a copy of the segment
            
            if (!point.close(end)) {
                seg->set_end_point(point);
            }
            
            return seg;
        }
        else {
            seg = copy(); // Create a copy of the segment
            
            if (!point.close(start)) {
                seg->set_start_point(point);
            }
            
            return seg;
        }
    }

    // Accessors
    virtual const vec2d& get_start() const { return start; }
    virtual const vec2d& get_end() const { return end; }
    virtual const vec2d& get_nhat_start() const = 0;
    virtual const vec2d& get_nhat_end() const = 0;

    // Modifiers
    virtual bool offset(float distance) = 0;
    virtual bool set_start_point(const vec2d& point) = 0;
    virtual bool set_end_point(const vec2d& point) = 0;


    // Queries
    virtual bounding_box get_bounding_box() const = 0;
    virtual compact_point to_compact_point() const = 0;
    virtual float trap_area() const = 0;
    virtual float length() const = 0;
    virtual bool valid() const = 0;

    // Analysis/comparison
    virtual bool diverges(const segment& other, float direction) const {
        return get_nhat_end().cross(other.get_nhat_start()) * direction > ANGULAR_RAD_TOL;
    }
    validated_intersection_result valid_intersections(const segment& other) const;

    // "Dumb" methods that are fast, but need further checking
    virtual intersection_result intersection(const segment& other) const = 0;
    virtual intersection_result intersection_with_line(const line_segment& line, bool arg_first) const = 0;
    virtual intersection_result intersection_with_arc(const arc_segment& arc, bool arg_first) const = 0;
    
    // Validation
    virtual float get_position(const vec2d & point) const = 0; // Points on segment return [0,1]

    // Helpers
    virtual std::string to_string() const = 0;
};

class line_segment : public segment {
protected:
    vec2d nhat;     // Unit normal vector
    vec2d vhat;     // Unit direction vector
    float s;        // Segment length

public:
    // Constructors/copiers/destructors
    line_segment(const vec2d& start, const vec2d& end, const vec2d nhat, const vec2d vhat, float s)
        : segment(start, end), nhat(nhat), vhat(vhat), s(s) {}
    line_segment(const vec2d& start, const vec2d& end);

    segment* copy() const override {
        return new line_segment(start, end, nhat, vhat, s);
    }

    static line_segment* from_compact_point(const compact_point& p0, const compact_point& p1) {
        return new line_segment(vec2d(p0[0], p0[1]), vec2d(p1[0], p1[1]));
    }

    // Accessors
    const vec2d& get_nhat_start() const override { return nhat; }
    const vec2d& get_nhat_end() const override { return nhat; }
    const vec2d& get_nhat() const { return nhat; }
    const vec2d& get_vhat() const { return vhat; }
    float get_s() const { return s; }

    // Modifiers
    bool offset(float distance) override;
    bool set_start_point(const vec2d& point) override;
    bool set_end_point(const vec2d& point) override;

    // Queries
    bounding_box get_bounding_box() const override {
        return bounding_box(
            std::min(start.v[0], end.v[0]),
            std::max(start.v[0], end.v[0]),
            std::min(start.v[1], end.v[1]),
            std::max(start.v[1], end.v[1])
        );
    }

    compact_point to_compact_point() const override {
        return {start.v[0], start.v[1], 0};
    }

    float trap_area() const override;
    float length() const override { return (end - start).length(); }
    bool valid() const override {
        // Check if the segment has non-zero length
        if (start.close(end)) {
            return false; // Zero-length segments are invalid
        }
        vec2d vhat_ref = (end - start).normalized();
        return vhat_ref.dot(vhat) > 0; // Check if the direction is valid
    }

    // Analysis/comparison
    intersection_result intersection(const segment& other) const override;
    intersection_result intersection_with_line(const line_segment& other, bool arg_first) const override;
    intersection_result intersection_with_arc(const arc_segment& arc, bool arg_first) const override;

    // Validation
    float get_position(const vec2d &point) const override;

    // Helpers
    std::string to_string() const override {
        std::ostringstream oss;
        oss << "LineSegment: Start(" << start.to_string() << "), End(" << end.to_string() << ")";
        oss << ", Nhat(" << nhat.to_string() << "), Vhat(" << vhat.to_string() << ")";
        return oss.str();
    }
};

class arc_segment : public segment {
protected:
    vec2d center;       // Center point of the arc
    vec2d nhat_start;   // Unit normal vector at start
    vec2d nhat_end;     // Unit normal vector at end
    float radius;       // Radius of the arc
    float start_angle;  // Start angle in radians
    float end_angle;    // End angle in radians

    // Private constructors
    arc_segment() = default;
    arc_segment(const vec2d& start, const vec2d& end, const vec2d& center, const vec2d& nhat_start, const vec2d& nhat_end, float radius, float start_angle, float end_angle)
        : segment(start, end), center(center), nhat_start(nhat_start), nhat_end(nhat_end), radius(radius), start_angle(start_angle), end_angle(end_angle) {}

public:
    // Constructors/copiers/destructors
    segment* copy() const override {
        return new arc_segment(start, end, center, nhat_start, nhat_end, radius, start_angle, end_angle);
    }

    // Static factory methods
    static arc_segment* from_compact_point(const compact_point& p0, const compact_point& p1);
    static arc_segment* arc1(const vec2d & center, const vec2d & point, float angle); // Two point arc segment
    static arc_segment* arc2(const vec2d & point1, const vec2d & point2, float radius); // Three point arc segment
    static arc_segment* arc3(const vec2d & point1, const vec2d & point2, float angle);
    static arc_segment* arc4(const vec2d & point1, const vec2d & point2, float bulge);
    static arc_segment* arc5(const vec2d & point1, const vec2d & point2, const vec2d & point3);

    // Accessors
    const float& get_radius() const { return radius; }
    const vec2d& get_center() const { return center; }
    const vec2d& get_nhat_start() const override { return nhat_start; }
    const vec2d& get_nhat_end() const override { return nhat_end; }
    float get_start_angle() const { return start_angle; }
    float get_end_angle() const { return end_angle; }

    // Modifiers
    bool offset(float distance) override;
    bool set_start_point(const vec2d& point) override;
    bool set_end_point(const vec2d& point) override;

    // Queries
    bounding_box get_bounding_box() const override {
        return bounding_box(
            center.v[0] - std::abs(radius), center.v[0] + std::abs(radius),
            center.v[1] - std::abs(radius), center.v[1] + std::abs(radius)
        );
    }

    compact_point to_compact_point() const override { 
        return {start.v[0], start.v[1], bulge()}; 
    }

    bool is_clockwise() const { return end_angle < start_angle; }
    float bulge() const { return std::tan((end_angle - start_angle) / 4.0f); }
    float trap_area() const override;
    float length() const override { return std::abs((end_angle - start_angle) * radius); }
    bool valid() const override { return radius * (end_angle - start_angle) > 0; }

    // Analysis/comparison
    intersection_result intersection(const segment& other) const override;
    intersection_result intersection_with_line(const line_segment& line, bool arg_first) const override;
    intersection_result intersection_with_arc(const arc_segment& other, bool arg_first) const override;

    // Validation
    float get_position(const vec2d &point) const override;

    // Helpers
    std::string to_string() const override {
        std::ostringstream oss;
        oss << "ArcSegment: Center(" << center.to_string() << "), Radius(" << radius << ")";
        oss << ", Start Angle(" << start_angle << "), End Angle(" << end_angle << ")";
        oss << ", Start(" << start.to_string() << "), End(" << end.to_string() << ")";
        oss << ", Nhat Start(" << nhat_start.to_string() << "), Nhat End(" << nhat_end.to_string() << ")";
        return oss.str();
    }
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

    path *copy() const {
        path *new_path = new path();
        for (auto seg : segments) {
            new_path->add_segment(seg->copy());
        }
        return new_path;
    }

    void add_segment(segment* seg) {
        if (!seg) {
            return; // Avoid adding null segments
        }
        segments.push_back(seg);
    }

    void add_path(const path &other) {
        for (auto seg : other.segments) {
            segments.push_back(seg->copy());
        }
    }

    void insert_segment(size_t index, segment* seg) {
        if (index < segments.size()) {
            segments.insert(segments.begin() + index, seg);
        } else {
            segments.push_back(seg);
        }
    }

    void remove_invalid_segments() {
        // Remove segments that are not valid
        std::vector<segment*> valid_segments;
        for (auto seg : segments) {
            if (seg != nullptr && seg->valid()) {
                valid_segments.push_back(seg);
            } else {
                delete seg; // Free memory for invalid segments
            }
        }
        segments.clear();
        for (auto seg : valid_segments) {
            segments.push_back(seg);
        }
        valid_segments.clear();
        // segments = std::move(valid_segments);
    }

    void cleanup() {
        // Remove any zero length segments
        size_t i = 0;
        for (i=0; i<segments.size(); i++) {
            segment* seg = segments[i];
            if (seg->length() < 1e-3) {
                delete seg; // Free memory for zero-length segments
                // Update adjacent segments so they share the exact point
                size_t prev_index = i == 0 ? segments.size() - 1 : i - 1;
                size_t next_index = i == segments.size()-1 ? 0 : i + 1;
                segments[prev_index]->set_end_point(segments[next_index]->get_start());
                segments.erase(segments.begin() + i); // Remove the zero-length segment
                i--; // Adjust index after removal
            }

            if (segments.size() < 3) {
                // If we have less than 3 segments, the path is invalid
                for (auto seg : segments) {
                    delete seg; // Free memory for all segments
                }
                segments.clear();
                return;
            }
        }
    }

    bool clockwise_winding() const { return signed_area() > 0; }

    std::vector<path*> offset(float distance, bool arc_join, bool cull);

    static path* from_compact_array(const std::vector<compact_point> &cp, bool c);
    std::vector<compact_point> to_compact_array() const;

    std::vector<path*> get_closed_loops();

    float signed_area() const;

    std::string to_string() const {
        std::ostringstream oss;
        oss << "Path with " << segments.size() << " segments:\n";
        for (const auto& seg : segments) {
            oss << "  - " << seg->to_string() << "\n";
        }
        return oss.str();
    }
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
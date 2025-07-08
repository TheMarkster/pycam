#include "geom.hpp"
#include "math2d.hpp"
#include "sorting.hpp"
#include "intersection.hpp"
#include <cmath>
#include <algorithm>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEBUG

line_segment::line_segment(const vec2d& start, const vec2d& end)
    : segment(start, end) {
    vhat = (end - start).normalized();
    nhat = rotate_cw_90(vhat); // Perpendicular vector
    s = nhat.dot(start);
}

bool line_segment::offset(float distance) {
    vec2d offset_vector = nhat * distance;
    start = start + offset_vector;
    end = end + offset_vector;
    s += distance;
    return true;
}

bool line_segment::diverges(const segment& other, float distance) const {
    // Check if the other segment diverges from this line segment
    if (distance < 0) distance = -1;
    else distance = 1;
    return other.diverges_from_line(*this, distance);
}

bool line_segment::diverges_from_line(const line_segment& first, float direction) const {
    return first.nhat.cross(nhat) * direction > 1e-3; // Tolerance within 0.06deg
}

bool line_segment::diverges_from_arc(const arc_segment& first, float direction) const {
    if (first.radius > 0) {
        return first.nhat_end.cross(nhat) * direction > 1e-3;
    }
    else {
        return (-first.nhat_end).cross(nhat) * direction > 1e-3;
    }
    
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
    vec2d sol = vec2d(nhat.dot(start), this->vhat.dot(arc.center));
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
    vec2d offset_vector = start - intersection_point;
    float distance = offset_vector.length();
    if (distance < arc.radius) {
        // Start point is inside the arc
        // Reverse the offset vector
        offset_vector = end - intersection_point;
        distance = offset_vector.length();
    }
    offset_vector /= distance;
    
    return intersection_point + offset_vector * b;
}

arc_segment* arc_segment::from_compact_point(const compact_point& p0, const compact_point& p1) {
    vec2d start = vec2d(p0[0], p0[1]);
    vec2d end = vec2d(p1[0], p1[1]);
    return arc_segment::arc4(start, end, p0[2]);
}

bool arc_segment::offset(float distance) {
    if (end_angle - start_angle < 0) {
        radius -= distance;
    } else {
        radius += distance;
    }
    start += nhat_start * distance;
    end += nhat_end * distance;
    return true;
}

bool arc_segment::diverges(const segment& other, float direction) const {
    // Check if the other segment diverges from this arc segment
    if (direction < 0) direction = -1;
    else direction = 1;
    return other.diverges_from_arc(*this, direction);
}

bool arc_segment::diverges_from_line(const line_segment& first, float direction) const {
    if (radius > 0) {
        // Arc is counter-clockwise
        return first.nhat.cross(nhat_start) * direction > 1e-3;
    } else {
        // Arc is clockwise
        return first.nhat.cross(-nhat_start) * direction > 1e-3;
    }
}

bool arc_segment::diverges_from_arc(const arc_segment& first, float direction) const {
    // Check if the arc diverges from another arc segment
    return first.nhat_end.cross(nhat_start) * direction > 1e-3;
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
    vec2d midpoint = center + v * mp_distance;
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

float line_segment::get_position(const vec2d& point) const {
    // Check if the point is on the line segment
    vec2d diff = point - start;
    vec2d end_to_point = end - start;
    return diff.dot(vhat) / end_to_point.dot(vhat);
}

float arc_segment::get_position(const vec2d& point) const {
    // We assume the point is on the circle
    // Verify it's an angle on the arc segment
    vec2d center_to_point = (point - center).normalized();
    float point_angle = angle(center_to_point) - start_angle;
    float delta_angle = end_angle - start_angle;

    if (delta_angle < 0 && point_angle > 0) {
        point_angle -= 2 * M_PI;
    } else if (delta_angle > 0 && point_angle < 0) {
        // Normalize to [0, 2*pi)
        point_angle += 2 * M_PI;
    }

    return point_angle / delta_angle;
}

result<vec2d> segment::intersects(const segment& other) const {
    vec2d intersection_point = intersection(other);
    bool success = on_segment(intersection_point) && other.on_segment(intersection_point);
    return result<vec2d>{intersection_point, success};
}

path* path::from_compact_array(const std::vector<compact_point> &data, bool close) {
    // Allocate segments
    path *p = new path();
    size_t n = data.size();
    p->segments.reserve(n);
    compact_point prev = data[0];

    size_t i;
    compact_point point;
    for (i=1; i<data.size(); i++) {
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

    if (close) {
        point = data[0];
        if (prev[2] == 0) {
            // Line segment
            p->segments.push_back(line_segment::from_compact_point(prev, point));
        } else {
            // Arc segment
            p->segments.push_back(arc_segment::from_compact_point(prev, point));
        }
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

inline void intersect(segment* seg1, segment* seg2) {
    vec2d intersection = seg1->intersection(*seg2);
    seg1->set_end_point(intersection);
    seg2->set_start_point(intersection);
}

std::vector<path*> path::offset(float distance, bool arc_join, bool cull) {
    // Create a new path for the offset segments
    path *offset_path = new path();
    segment *prev = NULL;
    segment *offset_segment = NULL;
    segment *temp = NULL;
    bool success;

    prev = segments[segments.size() - 1]->copy();
    prev->offset(distance);

    segment *top;
    vec2d intersection;
    for (segment* seg : segments) {
        offset_segment = seg->copy();
        success = offset_segment->offset(distance);

        if (!success) {
            delete offset_segment;
            continue;
        }

        if (prev->diverges(*seg, distance)) {
            if (arc_join) {
                temp = arc_segment::arc1(
                    seg->start, 
                    prev->end,
                    angle(prev->get_nhat_start(), offset_segment->get_nhat_end())
                );
                if (temp != NULL) {
                    offset_path->add_segment(temp);
                }
                else {
                    temp = NULL;
                }
            }
            else {
                temp = new line_segment(prev->end, offset_segment->start);

                if (temp != NULL) {
                    offset_path->add_segment(temp);
                }
                else {
                    temp = NULL;
                }
            }
            offset_path->add_segment(offset_segment);
            prev = offset_segment;
        }
        else {


// #ifdef DEBUG
//             if (prev->diverges(*offset_segment, distance)) {
//                 std::cout << "WTF MATE!" << std::endl;
//             }
//             std::cout << prev->to_string() << std::endl;
//             std::cout << offset_segment->to_string() << std::endl;
//             segment *prev_copy = prev->copy();
//             segment *offset_copy = offset_segment->copy();
// #endif
            intersect(prev, offset_segment);

// #ifdef DEBUG
//             if (prev->diverges(*offset_segment, distance)) {
//                 std::cout << offset_segment->to_string() << std::endl;
//                 std::cout << prev->to_string() << std::endl;
//                 std::cout << "WARNING: Segments diverge after intersection!" << std::endl;
//             }
//             delete prev_copy;
// #endif

            offset_path->add_segment(offset_segment);
            prev = offset_segment;
        }
    }

    // Close the start and end points
    prev->set_end_point(offset_path->segments[0]->start);

    std::vector<path*> valid;
    if (cull) {
        std::vector<path*> closed_loops = offset_path->get_closed_loops();
        delete offset_path;
        bool cw = clockwise_winding();
        for (path* loop : closed_loops) {
            if (loop->clockwise_winding() == cw) {
                loop->remove_invalid_segments();
                if (loop->segments.size() > 2) {
                    // Check if the loop is valid (has at least 3 segments)
                    valid.push_back(loop);
                } else {
                    delete loop; // Remove invalid loops
                }
            } else {
                // Winding reverses when a sub loop inverts outside of 
                // the main loop.
                delete loop;
            }
        }
    } else {
        offset_path->remove_invalid_segments();
        if (offset_path->segments.size() > 2) {
            valid.push_back(offset_path);
        }
        else {
            delete offset_path; // Remove invalid paths
        }
    }

    // Interior inverted loops have the correct winding
    // Find them by look for enclosed paths
    // TODO: Need to implement point in patht
    // Or maybe keep track of shared points and delete the smaller loops?

    return valid;
}

arc_segment* arc_segment::arc1(const vec2d & center, const vec2d & point, float angle) {
    arc_segment *seg = new arc_segment();

    vec2d rvec = point - center;
    seg->radius = rvec.length();
    rvec = rvec.normalized();

    seg->start = point;
    seg->center = center;
    seg->start_angle = angle_norm(rvec);
    seg->end_angle = seg->start_angle + angle;

    // Foundational equations
    // pos(angle) = center + radius * cis(angle)
    // vhat(angle) = d/dtheta (cis(angle)) = (-sin(angle) + i*cos(angle)) * dangle / dtheta
    // nhat(angle) = rot_cw_90(vhat(angle)) = cis(angle) * dangle / dtheta = (pos-center) / radius * dangle / dtheta
    // Following conditionals check for dangle / dtheta
    if (angle < 0) {
        // Normal for clockwise arc is rotated 180 degrees
        seg->nhat_start = -rvec;
        seg->nhat_end = -vec2d(std::cos(seg->end_angle), std::sin(seg->end_angle));
        seg->end = center - seg->nhat_end * seg->radius; // End point for clockwise arc
    }
    else {
        seg->nhat_start = rvec;
        seg->nhat_end = vec2d(std::cos(seg->end_angle), std::sin(seg->end_angle));
        seg->end = center - seg->nhat_end * seg->radius;
    }

    return seg;
}

arc_segment* arc_segment::arc2(const vec2d & point1, const vec2d & point2, float radius, bool is_clockwise) {
    vec2d midpoint = (point1 + point2) / 2;
    vec2d v21 = (point2 - point1);
    float off = radius * radius - v21.dot(v21)/4;
    v21 = v21.normalized();
    
    vec2d center;
    if (std::abs(off) < 1e-6) {
        center = midpoint;
    }
    else if (off < 0) {
        // No valid arc can be formed
        return nullptr; // or throw an exception
    }
    else {
        off = std::sqrt(off);
        if (is_clockwise) {
            center = midpoint + rotate_cw_90(v21) * off;
        } else {
            center = midpoint + rotate_ccw_90(v21) * off;
        }
    }
    
    vec2d v1 = point1 - center;
    vec2d v2 = point2 - center;

    arc_segment *seg = new arc_segment();
    seg->radius = v1.length();
    v1 = v1.normalized();
    v2 = v2.normalized();

    seg->start = point1;
    seg->end = point2;
    seg->center = center;
    seg->start_angle = angle_norm(v1);
    seg->end_angle = angle_norm(v2);

    if (is_clockwise) {
        // Adjust angles for clockwise direction
        if (seg->end_angle > seg->start_angle) {
            seg->end_angle -= 2 * M_PI; // Adjust for clockwise direction
        }
    } else {
        // Adjust angles for counter-clockwise direction
        if (seg->end_angle < seg->start_angle) {
            seg->end_angle += 2 * M_PI; // Adjust for counter-clockwise direction
        }
    }

    seg->nhat_start = v1;
    seg->nhat_end = v2;
    
    return seg;
}

arc_segment* arc_segment::arc3(const vec2d & point1, const vec2d & point2, float angle, bool invert) {
    arc_segment *seg = new arc_segment();

    vec2d midpoint = (point1 + point2) / 2;
    vec2d v21 = (point2 - point1);
    float chord_length2 = v21.length()/2;
    v21 = v21.normalized();

    // radius * sin(angle / 2) = chord_length / 2
    float radius = chord_length2 / (std::sin(angle / 2));
    float offset = std::sqrt(radius * radius - chord_length2 * chord_length2);
    vec2d center = midpoint + rotate_ccw_90(v21) * offset;

    vec2d v1 = (point1 - center).normalized();

    seg->start = point1;
    seg->end = point2;
    seg->center = center;
    if (invert) {
        seg->radius = -std::abs(radius);
        seg->start_angle = angle_norm(v1) + M_PI;
        seg->start_angle = modulo(seg->start_angle + M_PI, 2 * M_PI) - M_PI; // Normalize to [-pi, pi]
    } else {
        seg->radius = std::abs(radius);
        seg->start_angle = angle_norm(v1);
    }
    seg->end_angle = seg->start_angle + angle;
    seg->nhat_start = -v1;
    seg->nhat_end = -vec2d(std::cos(seg->end_angle), std::sin(seg->end_angle));

    return seg;
}

arc_segment* arc_segment::arc4(const vec2d & point1, const vec2d & point2, float bulge) {
    arc_segment *seg = new arc_segment();

    vec2d v21 = point2 - point1;
    float cl2 = v21.length() / 2;
    // float bulge = height / cl2;
    vec2d midpoint = (point1 + point2) / 2;
    float angle = std::atan(bulge)*4.0f; // Convert bulge to angle
    float radius = cl2 / std::sin(angle / 2.0f);
    vec2d nhat;
    if (bulge < 0) { // Clockwise arc
        nhat = rotate_cw_90(v21.normalized());
    }
    else {
        nhat = rotate_ccw_90(v21.normalized());
    }
    vec2d center;
    if (std::abs(bulge) < 1) {
        center = midpoint + nhat * std::sqrt(radius * radius - cl2 * cl2);
    }
    else {
        center = midpoint - nhat * std::sqrt(radius * radius - cl2 * cl2);
    }
    
    // vec2d center = midpoint;

    vec2d n1 = (point1 - center).normalized();
    vec2d n2 = (point2 - center).normalized();

    seg->start = point1;
    seg->end = point2;
    seg->center = center;
    seg->radius = std::abs(radius);
    // if (radius < 0) {
    //     seg->start_angle = modulo(angle_norm(n1) + 2*M_PI, 2*M_PI) - M_PI;
    // }
    // else {
    //     seg->start_angle = angle_norm(n1);
    // }
    seg->start_angle = angle_norm(n1);
    seg->end_angle = seg->start_angle + angle;
    seg->nhat_start = n1;
    seg->nhat_end = n2;

    return seg;
}

arc_segment* arc_segment::arc5(const vec2d & point1, const vec2d & point2, const vec2d & point3) {
    arc_segment *seg = new arc_segment();

    vec2d mp21 = (point1 + point2) / 2;
    vec2d v21 = point2 - point1;
    vec2d n21 = rotate_ccw_90(v21);

    vec2d mp32 = (point2 + point3) / 2;
    vec2d v32 = point3 - point2;
    vec2d n32 = rotate_ccw_90(v32);

    vec2d sol(v21.dot(mp21), v32.dot(mp32));

    vec2d center = solve(mat2d::from_rvec(v21, v32), sol);

    vec2d v31 = point3 - point1;
    bool is_clockwise = (v21.cross(v31) < 0);

    vec2d n1 = point1 - center;
    vec2d n3 = point3 - center;
    seg->radius = n1.length();
    n1 = n1.normalized();
    n3 = n3.normalized();

    seg->start = point1;
    seg->end = point3;
    seg->center = center;
    seg->start_angle = angle_norm(n1);
    seg->end_angle = angle_norm(n3);
    if (is_clockwise) {
        // Adjust angles for clockwise direction
        if (seg->end_angle > seg->start_angle) {
            seg->end_angle -= 2 * M_PI; // Adjust for clockwise direction
        }
    } else {
        // Adjust angles for counter-clockwise direction
        if (seg->end_angle < seg->start_angle) {
            seg->end_angle += 2 * M_PI; // Adjust for counter-clockwise direction
        }
    }
    seg->nhat_start = n1;
    seg->nhat_end = n3;

    return seg;
}

bool line_segment::set_end_point(const vec2d& point) {
    end = point;
    vec2d delta = end - start;
    if (delta.dot(vhat) < 0) {
        // vec2d temp = start;
        // start = end;
        // end = temp;
        return false;
    }
    return true;
}

bool line_segment::set_start_point(const vec2d& point) {
    start = point;
    vec2d delta = end - start;
    if (delta.dot(vhat) < 0) {
        // vec2d temp = start;
        // start = end;
        // end = temp;
        return false;
    }
    return true;
}

bool arc_segment::set_end_point(const vec2d& point) {
    float dangle = end_angle - start_angle;

    vec2d oldEnd = end - center;
    vec2d newEnd = point - center;
    float angleChange = angle(oldEnd, newEnd);

    end_angle += angleChange;
    nhat_end = rotate_ccw(nhat_end, angleChange);
    end = point;
    
    float dangle_new = end_angle - start_angle;
    if (dangle * dangle_new <= 0) {
        // Angle inversion implies zero-length arc occurred
        return false;
    }
    else
        return true;
}

bool arc_segment::set_start_point(const vec2d& point) {
    float dangle = end_angle - start_angle;

    vec2d oldStart = start - center;
    vec2d newStart = point - center;
    float angleChange = angle(oldStart, newStart);

    start = point;
    nhat_start = rotate_ccw(nhat_start, angleChange);
    start_angle += angleChange;

    float dangle_new = end_angle - start_angle;
    if (dangle * dangle_new < 0) {
        // Angle inversion implies zero-length arc occurred
        return false;
    }
    else
        return true;
}

float line_segment::trap_area() const {
    return 0.5 * (start.v[1] + end.v[1]) * (end.v[0] - start.v[0]);
}

inline float arc_integral(float x)
{
    float area;
    if (x >= 1)
        area = 0.0;
    else if (x <= -1)
        area = M_PI_2;
    else
        area = 0.5 * x * std::sqrt(1-x*x) - std::atan2(std::sqrt(1-x*x), x+1);
    return area;
}

float arc_segment::trap_area() const {
    // Calculates the signed area between the arc segment and the x-axis
    // 

    bool clockwise = is_clockwise();

    float a0, a1;
    bool h1, h2;
    float x0, x1;
    if (clockwise) {
        a0 = modulo(start_angle+M_PI, 2 * M_PI)-M_PI;
        a1 = modulo(end_angle+M_PI, 2 * M_PI)-M_PI;
    } else {
        a0 = modulo(end_angle+M_PI, 2 * M_PI)-M_PI;
        a1 = modulo(start_angle+M_PI, 2 * M_PI)-M_PI;
    }

    x0 = std::cos(a0)*radius;
    x1 = std::cos(a1)*radius;
    h1 = a0 >= 0;
    h2 = a1 >= 0;

    float area = 0.0; // Rectangular contribution
    if (h1 == h2) {
        // Start and stop on same half-plane
        if (h1) {
            if (x1 < x0) {
                area += arc_integral_top(x0, radius, radius, center.v[1]);
                area += arc_integral_top(-radius, x1, radius, center.v[1]);
                // area -= arc_integral_bot(-radius, radius, radius, center.v[1]);
                area -= (2*radius*center.v[1] - M_PI_2*radius*radius);
            }
            else {
                area += arc_integral_top(x0, x1, radius, center.v[1]);
            }
        }
        else {
            if (x1 > x0) {
                area -= arc_integral_bot(-radius, x0, radius, center.v[1]);
                area -= arc_integral_bot(x1, radius, radius, center.v[1]);
                // area += arc_integral_top(-radius, radius, radius, center.v[1]);
                area += (2*radius*center.v[1] + M_PI_2*radius*radius);
            }
            else {
                area -= arc_integral_bot(x1, x0, radius, center.v[1]);
            }
        }
    } else {
        if (h1) {
            area += arc_integral_top(x0, radius, radius, center.v[1]);
            area -= arc_integral_bot(x1, radius, radius, center.v[1]);
        }
        else {
            area += arc_integral_top(-radius, x1, radius, center.v[1]);
            area -= arc_integral_bot(-radius, x0, radius, center.v[1]);
        }
    }

    if (clockwise) {
        return area;
    } else {
        return -area;
    }
}

struct path_or_intersection {
    path *thePath;
    intersection *theIntersection;
};

path *path_between(const std::vector<segment*> *orig, size_t start, size_t end, vec2d startPoint, vec2d endPoint) {
    // Create a path between two intersections
    path *temp = new path();

    if (start == end) {
        segment *seg = orig->at(start)->bisect(startPoint, false); // Bisect at start point

        seg = seg->bisect(endPoint, true); // Bisect at end point
        temp->add_segment(seg);
        return temp;
    }

    size_t ii = start;

    // Start point segment
    temp->add_segment(orig->at(ii)->bisect(startPoint, false));
    ii = (ii + 1) % orig->size();
    
    // Intermediate segments
    while (ii != end) {
        temp->segments.push_back(orig->at(ii)->copy());
        ii = (ii + 1) % orig->size(); // Move to the next segment
    }

    // End point segment
    temp->add_segment(orig->at(ii)->bisect(endPoint, true));

    return temp;
}

inline bool matched_pair(const intersection& a, const intersection& b) {
    // Check if two intersections are a matched pair
    return (a.index1 == b.index2 && a.index2 == b.index1);
}

inline path* get_loop(const std::vector<intersection*> &stack, size_t start) {
    // Create a path from the stack of intersections
    
    path *p;
    const std::vector<segment*> *orig = stack[start]->path1;
    if (matched_pair(*stack[start], *stack[start+1])) {
        return path_between(orig, 
            stack[start]->index1,   stack[start+1]->index1,
            stack[start]->point,    stack[start+1]->point);
    }

    path *temp = new path();
    
    std::vector<const intersection*> points;
    const intersection *prev = stack[start];
    points.push_back(prev);
    size_t i;
    for (i=start+1; i<stack.size(); i++) {
        intersection *current = stack[i];
        if (matched_pair(*prev, *current)) {
            // Flush the points out
            if (points.size() > 1) {
                p = path_between(orig, points[0]->index1, points.back()->index1, points[0]->point, points.back()->point);
                temp->add_path(*p);
                delete p;
            }
            points.clear();
        }
        points.push_back(current);
        prev = current;
    }

    // if (!matched_pair(*stack[start], *stack.back())) {
    //     points.push_back(stack[start]);
    // }
    
    // Flush remaining points
    if (points.size() > 1) {
        p = path_between(orig, points[0]->index1, points.back()->index1, points[0]->point, points.back()->point);
        temp->add_path(*p);
        delete p;
    }

    // Check for bad geometry
    temp->cleanup();
    if (temp->segments.size() < 3) {
        for (auto& seg : temp->segments) {
            delete seg;
        }
        temp->segments.clear();
    }

    return temp;
}

std::vector<path*> path::get_closed_loops() {
    // Return a vector of valid paths from the segments in this path
    std::vector<path*> valid_paths;

    std::vector<std::vector<segment*>*> paths;
    paths.push_back(&segments);
    std::vector<intersection> intersections = find_intersections(paths);

    // Iterate over the path and generate partial segments
    if (intersections.empty()) {
        // No intersections found, the path is valid as is
        valid_paths.push_back(copy());
        return valid_paths;
    }

    size_t n = intersections.size();
    size_t i, j;
    for (i = 0; i < n; i++) {
        // Duplicate the intersections so we have one for each segment
        intersections.push_back(intersections[i]);
        intersections.back().index1 = (intersections[i].index2);
        intersections.back().index2 = (intersections[i].index1);
        intersections.back().pos1 = intersections[i].pos2;
        intersections.back().pos2 = intersections[i].pos1;
    }

    // Sort intersections by their segment indices
    std::sort(intersections.begin(), intersections.end(), [](const intersection& a, const intersection& b) {
        if (a.index1 < b.index1) return true;
        if (a.index1 > b.index1) return false;

        return a.pos1 < b.pos1;
    });

    std::vector<intersection*> stack;

    intersection *int1, *int2;
    path *temp;
    for (i=0; i < intersections.size(); i++) {
        int1 = &intersections[i];
        stack.push_back(int1);
        
        // Check if we've returned to the same intersection
        // and created a loop
        // TODO: Should we use a hashmap for faster lookups?
        for (j=0; j<stack.size()-1; j++) {
            int2 = stack[j];
            if (matched_pair(*int1, *int2)) {
                // Add the path to the output
                temp = get_loop(stack, j);
                if (temp->segments.empty()) {
                    // If the path is empty, we can skip it
                    delete temp;
                }
                else { valid_paths.push_back(temp); }
            
                // Remove intermediate intersections along loop
                stack.resize(j+1); 
                stack.push_back(int1);

                break;
            }
        }
    }

    // Rotate the stack so the first intersection is now last
    // Find the loop that terminates with the first intersection
    intersection *first_intersection = stack[0];
    for (i=1; i<stack.size(); i++) {
        stack[i-1] = stack[i];
    }
    stack.back() = first_intersection;
    
    temp = get_loop(stack, 0);
    if (temp->segments.empty()) {
        // Should the last path ever be empty?
        delete temp;
    } else {
        valid_paths.push_back(temp);
    }

    return valid_paths;
}

float path::signed_area() const {
    // Calculate the signed area of the path
    float area = 0.0f;
    for (const auto& seg : segments) {
        area += seg->trap_area();
    }
    return area;
}
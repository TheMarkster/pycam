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

validated_intersection_result segment::valid_intersections(const segment& other) const {
    intersection_result possible_intersections = intersection(other);

    float pos1, pos2;
    validated_intersection_result result;
    result.count = 0;
    for (size_t i=0; i<possible_intersections.count; i++) {
        pos1 = get_position(possible_intersections.data[i]);
        pos2 = other.get_position(possible_intersections.data[i]);

        // Simple check greatly improves robustness
        // Eliminates intersections at connecting point between adjacent segments
        // Prevents duplicate intersections
        // Other instances fall under "don't care" category
        if (std::abs(pos2) < POS_1D_TOL) continue;

        if (pos1 >= 0 && pos1 <= 1 && pos2 >= 0 && pos2 <= 1) {
            result.data[result.count] = {possible_intersections.data[i], pos1, pos2};
            result.count++;
        }
    }
    return result;
}

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

intersection_result line_segment::intersection(const segment& other) const {
    // Find intersection with another segment
    return other.intersection_with_line(*this, true);
}

intersection_result line_segment::intersection_with_line(const line_segment& other, bool arg_first) const {
    // Calculate intersection point with another line segment
    intersection_result result;
    result.count = 1;
    mat2d m = mat2d::from_rvec(nhat, other.nhat);
    vec2d s_vec = vec2d(s, other.s);
    result.data[0] = solve(m, s_vec);
    return result;
}

intersection_result line_segment::intersection_with_arc(const arc_segment& arc, bool arc_first) const {
    // Implement logic to find intersection with an arc segment

    // Intersect vector with normal going through arc's center
    // This is the midpoint of possible intersections
    mat2d m = mat2d::from_rvec(nhat, vhat);
    vec2d sol = vec2d(nhat.dot(start), vhat.dot(arc.get_center()));
    vec2d intersection_point = solve(m, sol);

    // Distance from midpoint to solutions
    intersection_result result;
    float a = (intersection_point - arc.get_center()).length();
    float b = pow(arc.get_radius(), 2) - pow(a,2);
    if (abs(b) < POS_2D_TOL_SQUARED) {
        // Midpoint is the intersection point
        result.count = 1;
        result.data[0] = intersection_point;
        return result;
    } else if (b < 0) {
        // No intersection
        result.count = 0;
        return result;
    }
    b = sqrt(b);

    result.count = 2;
    result.data[0] = intersection_point + vhat * b;
    result.data[1] = intersection_point - vhat * b;
    return result;
}

arc_segment* arc_segment::from_compact_point(const compact_point& p0, const compact_point& p1) {
    vec2d start = vec2d(p0[0], p0[1]);
    vec2d end = vec2d(p1[0], p1[1]);
    return arc_segment::arc4(start, end, p0[2]);
}

bool arc_segment::offset(float distance) {
    radius += distance;
    start += nhat_start * distance;
    end += nhat_end * distance;
    return true;
}

intersection_result arc_segment::intersection(const segment& other) const {
    // Find intersection with another segment
    return other.intersection_with_arc(*this, true);
}

intersection_result arc_segment::intersection_with_line(const line_segment& line, bool arg_first) const {
    return line.intersection_with_arc(*this, !arg_first);
}

intersection_result arc_segment::intersection_with_arc(const arc_segment& other, bool arg_first) const {
    // Find midpoint of solutions
    vec2d v = other.center - center;
    float vlen2 = v.dot(v);
    float vlen = std::sqrt(vlen2);

    v /= vlen; // Normalize vector
    float mp_distance = (pow(radius, 2) - pow(other.radius, 2) + vlen2) / (2 * vlen);
    vec2d midpoint = center + v * mp_distance;
    float solution_distance = std::sqrt(pow(radius, 2) - pow(mp_distance, 2));

    if (std::isnan(solution_distance)) {
        // No solutions
        intersection_result result;
        result.count = 0;
        return result;
    } else if (std::abs(solution_distance) < POS_2D_TOL) {
        // One solution
        intersection_result result;
        result.count = 1;
        result.data[0] = midpoint;
        return result;
    } else {
        // Two solutions
        intersection_result result;
        result.count = 2;
        v = rotate_ccw_90(v); // Rotate vector 90 degrees counter-clockwise
        result.data[0] = midpoint + v * solution_distance;
        result.data[1] = midpoint - v * solution_distance;
        return result;
    }
}

float line_segment::get_position(const vec2d& point) const {
    if (point.close(start)) {
        return 0.0f;
    }

    if (point.close(end)) {
        return 1.0f;
    }

    // Find position along the line segment and scale to [0, 1] for on segment
    vec2d diff = point - start;
    vec2d end_to_point = end - start;
    return diff.dot(vhat) / end_to_point.dot(vhat);
}

float arc_segment::get_position(const vec2d& point) const {
    if (point.close(start)) {
        return 0.0f;
    }

    if (point.close(end)) {
        return 1.0f;
    }

    // Find angle of point and scale to [0, 1] for on segment
    vec2d v = (point - center).normalized();
    float delta_angle = end_angle - start_angle;
    float angle;
    // We want the radial segment, but nhat 180deg off for clockwise arcs
    if (radius < 0) {
        angle = (angle_norm(nhat_start, v) - M_PI);
    }
    else {
        angle = angle_norm(nhat_start, v);
        if (angle < 0) {
            angle += 2 * M_PI;
        }
    }
    
    return angle / delta_angle;
}

// TODO: This creates bad code patterns. Remove when possible.
// result<intersection_result> segment::intersects(const segment& other) const {
//     intersection_result intersection_point = intersection(other);
//     result<intersection_result> result;
//     result.data.count = 0;

//     // Only include points that are on both segments
//     bool success;
//     for (u_int8_t i = 0; i < intersection_point.count; i++) {
//         vec2d point = intersection_point.point[i];
//         if (other.on_segment(point) && on_segment(point)) {
//             result.data.point[result.data.count++] = point;
//         }
//     }
//     result.success = result.data.count > 0;

//     return result;
// }

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
    intersection_result intersection = seg1->intersection(*seg2);
    vec2d closest;
    if (intersection.count == 0) {
        return;
    } else if (intersection.count == 1) {
        closest = intersection.data[0];
    } else {
        // Two intersection points, find the closest to the end of seg1
        float pos1 = seg1->get_position(intersection.data[0]);
        float pos2 = seg2->get_position(intersection.data[1]);
        if (pos1 >= 0) {
            if (pos2 >= 0) {
                if (pos1 > pos2) {
                    closest = intersection.data[1];
                } else {
                    closest = intersection.data[0];
                }
            } else {
                closest = intersection.data[0];
            }
        }
        else {
            if (pos2 >= 0) {
                closest = intersection.data[1];
            } else {
                // Both points are behind the start of seg1
                if (pos1 > pos2) {
                    closest = intersection.data[0];
                } else {
                    closest = intersection.data[1];
                }
            }
        }
    }

    seg1->set_end_point(closest);
    seg2->set_start_point(closest);
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
                    angle(prev->get_nhat_end(), offset_segment->get_nhat_start())
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
        // offset_path->remove_invalid_segments();
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

// Foundational equations
// pos(angle) = center + radius * cis(angle)
// vhat(angle) = d/dtheta (cis(angle)) = (-sin(angle) + i*cos(angle)) * dangle / dtheta
// nhat(angle) = rot_cw_90(vhat(angle)) = cis(angle) * dangle / dtheta = (pos-center) / radius * dangle / dtheta
//
// Convention:
// Clockwise arcs have negative radius
// Angles are shifted by PI radians
// Allows us to encode orientation in the sign of radius, angle, and bulge

arc_segment* arc_segment::arc1(const vec2d & center, const vec2d & point, float angle) {
    arc_segment *seg = new arc_segment();

    vec2d rvec;
    if (angle < 0) {
        rvec = center - point;
        seg->radius = -rvec.length();
    } else {
        rvec = point - center;
        seg->radius = rvec.length();
    }
    rvec = rvec.normalized();

    seg->start = point;
    seg->center = center;


    // Following conditionals check for dangle / dtheta
    seg->nhat_start = rvec;
    seg->nhat_end = rotate_ccw(seg->nhat_start, angle); // 90 degree rotation
    seg->start_angle = angle_norm(rvec);
    seg->end_angle = seg->start_angle + angle;
    seg->end = seg->center + vec2d(std::cos(seg->end_angle), std::sin(seg->end_angle)) * seg->radius;

    return seg;
}

arc_segment* arc_segment::arc2(const vec2d & point1, const vec2d & point2, float radius) {
    vec2d midpoint = (point1 + point2) / 2;
    vec2d v21 = (point2 - point1);
    float off = radius * radius - v21.dot(v21)/4;
    v21 = v21.normalized();
    
    vec2d center;
    if (std::abs(off) < POS_2D_TOL_SQUARED) {
        center = midpoint;
    }
    else if (off < 0) {
        // No valid arc can be formed
        return nullptr; // or throw an exception
    }
    else {
        off = std::sqrt(off);
        if (radius < 0) {
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

    if (radius < 0) {
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

arc_segment* arc_segment::arc3(const vec2d & point1, const vec2d & point2, float angle) {
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

    seg->end = point2;
    seg->center = center;
    if (angle < 0) {
        seg->start_angle = angle_norm(v1) + M_PI;
        seg->start_angle = modulo(seg->start_angle + M_PI, 2 * M_PI) - M_PI; // Normalize to [-pi, pi]
        seg->nhat_end = -vec2d(std::cos(seg->end_angle), std::sin(seg->end_angle));
        seg->nhat_start = -v1;
    } else {
        seg->start_angle = angle_norm(v1);
        seg->nhat_end = vec2d(std::cos(seg->end_angle), std::sin(seg->end_angle));
        seg->nhat_start = v1;
    }
    seg->end_angle = seg->start_angle + angle;

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
    vec2d n1, n2;
    if (bulge < 0) {
        n1 = (center - point1).normalized();
        n2 = (center - point2).normalized();
    }
    else {
        n1 = (point1 - center).normalized();
        n2 = (point2 - center).normalized();
    }

    seg->start = point1;
    seg->end = point2;
    seg->center = center;
    seg->radius = radius;
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
    vec2d oldEnd = end - center;
    vec2d newEnd = point - center;
    float angleChange = angle(oldEnd, newEnd);

    end_angle += angleChange;
    nhat_end = rotate_ccw(nhat_end, angleChange);
    end = point;
    
    return true;
}

bool arc_segment::set_start_point(const vec2d& point) {
    vec2d oldStart = start - center;
    vec2d newStart = point - center;
    float angleChange = angle(oldStart, newStart);

    start = point;
    nhat_start = rotate_ccw(nhat_start, angleChange);
    start_angle += angleChange;

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
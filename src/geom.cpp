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
    return other.diverges_from_line(*this, distance);
}

bool line_segment::diverges_from_line(const line_segment& first, float direction) const {
    float cross = first.vhat.cross(vhat) * direction;
    if (std::abs(cross) < 1e-6) {
        // Parallel lines, check if they are collinear
        return false;
    }
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

arc_segment* arc_segment::from_compact_point(const compact_point& p0, const compact_point& p1) {
    vec2d start = vec2d(p0[0], p0[1]);
    vec2d end = vec2d(p1[0], p1[1]);
    return arc_segment::arc4(start, end, p0[2]);
}

bool arc_segment::offset(float distance) {
    float temp = radius + distance;
    if (temp * radius < 0) {
        return false;
    }
    radius = temp;
    start = center + nhat_start * radius;
    end = center + nhat_end * radius;
    return true;
}

bool arc_segment::diverges(const segment& other, float direction) const {
    // Check if the other segment diverges from this arc segment
    return other.diverges_from_arc(*this, direction);
}

bool arc_segment::diverges_from_line(const line_segment& first, float direction) const {
    // Check if the arc diverges from a line segment
    float cross = first.nhat.cross(nhat_start) * direction;
    if (std::abs(cross) < 1e-6) {
        // Parallel lines, check if they are collinear
        return false;
    }
    return first.nhat.cross(nhat_start) * direction * radius < 0;
}

bool arc_segment::diverges_from_arc(const arc_segment& first, float direction) const {
    // Check if the arc diverges from another arc segment
    float cross = first.nhat_end.cross(nhat_start) * direction;
    if (std::abs(cross) < 1e-6) {
        // Parallel arcs, check if they are collinear
        return false;
    }
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
    
    if (is_clockwise()) { // clockwise arc
        if (angle > start_angle) {
            angle -= 2 * M_PI; // Adjust angle to be in range [start_angle, end_angle]
        }
        return angle >= end_angle;
    }
    else {
        if (angle < start_angle) {
            angle += 2 * M_PI; // Adjust angle to be in range [end_angle, start_angle]
        }
        return angle <= end_angle;
    }
}

result<vec2d> segment::intersects(const segment& other) const {
    // Exclude adjacent segments
    if (start.close(other.end)) {
        return result<vec2d>{start, false};
    }
    if (end.close(other.start)) {
        return result<vec2d>{end, false};
    }
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

path* path::offset(float distance, bool arc_join) {
    // Create a new path for the offset segments
    path *offset_path = new path();
    segment *prev = NULL;
    segment *offset_segment = NULL;
    segment *temp = NULL;
    bool success;

    prev = segments[segments.size() - 1]->clone();
    prev->offset(distance);

    segment *top;
    vec2d intersection;
    for (segment* seg : segments) {
        offset_segment = seg->clone();
        success = offset_segment->offset(distance);

        if (!success) {
            delete offset_segment;
            continue;
        }

        if (seg->diverges(*prev, distance)) {
            if (arc_join) {
                temp = arc_segment::arc1(seg->start, prev->end, angle(prev->get_nhat_end(), offset_segment->get_nhat_start()));
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
            // while (true) {
            //     if (offset_path->segments.empty()) {
            //         // No segments in the offset path, add the first segment
            //         offset_path->add_segment(offset_segment);
            //         prev = offset_segment;
            //         break;
            //     }
                
            //     top = offset_path->segments[offset_path->segments.size() - 1];
            //     intersection = offset_segment->intersection(*top);

            //     if (!top->set_end_point(intersection)) {
            //         offset_path->segments.pop_back();
            //         delete top;
            //     }
            //     else {
            //         if (offset_segment->set_start_point(intersection)) {
            //             // The segment is now valid
            //             offset_path->add_segment(offset_segment);
            //             prev = offset_segment;
            //             break;
            //         }
            //         else {
            //             // The segment is not valid, remove it
            //             delete offset_segment;
            //             prev = top;
            //             break;
            //         }
            //     }
            // }

            intersect(prev, offset_segment);

            offset_path->add_segment(offset_segment);
            prev = offset_segment;
        }
    }

    // Intersect first and last segments
    // while (true) {
    //     if (offset_path->segments.size() < 3) {
    //         break;
    //     }
        
    //     top = offset_path->segments[0];
    //     prev = offset_path->segments[offset_path->segments.size() - 1];
    //     intersection = prev->intersection(*top);

    //     if (prev->set_end_point(intersection)) {
    //         if (top->set_start_point(intersection)) {
    //             break;
    //         }
    //         else {
    //             // The segment is not valid, remove it
    //             offset_path->segments.pop_back();
    //             delete prev;
    //         }
    //     }
    //     else {
    //         offset_path->segments.erase(offset_path->segments.begin());
    //         delete top;
    //     }
    // }

    // Close the start and end points
    prev->set_end_point(offset_path->segments[0]->start);

    if (offset_path->segments.size() > 2) {
        return offset_path;
    } else if (offset_path->segments.size() == 2) {
        // Valid if at least one arc segment
        for (segment *seg : offset_path->segments) {
            if (dynamic_cast<arc_segment*>(seg) != nullptr) {
                return offset_path;
            }
        }
    }
    
    // Not a valid path
    while (!offset_path->segments.empty()) {
        delete offset_path->segments.back();
        offset_path->segments.pop_back();
    }

    return nullptr;
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
    seg->nhat_start = rvec;
    seg->nhat_end = vec2d(std::cos(seg->end_angle), std::sin(seg->end_angle));
    seg->end = center + seg->nhat_end * seg->radius;

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

    seg->start = point1;
    seg->end = point2;
    seg->center = center;
    seg->radius = std::abs(radius);
    seg->start_angle = angle_norm(v1);
    seg->end_angle = seg->start_angle + angle;
    seg->nhat_start = v1;
    seg->nhat_end = vec2d(std::cos(seg->end_angle), std::sin(seg->end_angle));

    return seg;
}

arc_segment* arc_segment::arc4(const vec2d & point1, const vec2d & point2, float height) {
    arc_segment *seg = new arc_segment();

    vec2d v21 = point2 - point1;
    float cl2 = v21.length() / 2;
    float bulge = height / cl2;
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
    seg->radius = radius;
    if (radius < 0) {
        seg->start_angle = -angle_norm(n1);
        seg->end_angle = seg->start_angle - angle;
    } else {
        seg->start_angle = angle_norm(n1);
        seg->end_angle = seg->start_angle + angle;
    }
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
        // If the new end point is behind the start point, a zero-length segment occurred
        return false;
    }
    return true;
}

bool line_segment::set_start_point(const vec2d& point) {
    start = point;
    vec2d delta = end - start;
    if (delta.dot(vhat) < 0) {
        // If the new start point is ahead of the end point, a zero-length segment occurred
        return false;
    }
    return true;
}

bool arc_segment::set_end_point(const vec2d& point) {
    float dangle = end_angle - start_angle;
    end = point;
    vec2d nhat_new = (point - center).normalized();
    end_angle += angle_norm(nhat_end, nhat_new);
    nhat_end = nhat_new;
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
    start = point;
    vec2d nhat_new = (point - center).normalized();
    start_angle += angle_norm(nhat_start, nhat_new);
    nhat_start = nhat_new;
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



inline double modulo(double x, double m) {
    return (x - std::floor(x / m) * m);
}

inline float modulo(float x, float m) {
    return (x - std::floor(x / m) * m);
}

inline int modulo(int x, int m) {
    return (x % m + m) % m; // Ensure non-negative result
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

std::vector<path*> path::get_closed_loops() {
    // Return a vector of valid paths from the segments in this path
    std::vector<path*> valid_paths;

    std::vector<std::vector<segment*>*> paths;
    paths.push_back(&segments);
    std::vector<intersection> intersections = find_intersections(paths);

    // Iterate over the path and generate partial segments
    if (intersections.empty()) {
        // No intersections found, the path is valid as is
        valid_paths.push_back(this);
        return valid_paths;
    }

    size_t n = intersections.size();
    size_t i, j, ii, jj;
    for (i = 0; i < n; i++) {
        // Duplicate the intersections so we have one for each segment
        intersections.push_back(intersections[i]);
        intersections.back().index1 = (intersections[i].index2);
        intersections.back().index2 = (intersections[i].index1);
    }

    // Sort intersections by their segment indices
    std::sort(intersections.begin(), intersections.end(), [](const intersection& a, const intersection& b) {
        return a.index1 < b.index1;
    });

    linked_list<path_or_intersection> stack;
    linked_item<path_or_intersection> *item, *prevItem;

    segment *seg;
    path *temp, *merger;
    intersection *prev, *current;
    prev = &intersections[intersections.size() - 1]; // Start with the last intersection
    for (i=0; i<intersections.size(); i++) {
        current = &intersections[i];

        // Create a path between the two intersections
        temp = new path();
        ii = prev->index1;
        jj = current->index1;
        seg = segments[prev->index1]->bisect(prev->point, false);
        temp->add_segment(seg);
        ii = (ii+1) % segments.size(); // Move to the next segment
        while (ii != jj) {
            seg = segments[ii];
            temp->segments.push_back(seg->clone());
            ii = (ii+1) % segments.size(); // Move to the next segment
        }
        seg = segments[current->index1]->bisect(current->point, true);
        temp->add_segment(seg);
        
        // Add path to the stack
        item = stack.add();
        item->si.thePath = temp;
        item->si.theIntersection = nullptr;

        // Check if intersection has occurred
        // If it has, pop all items until prev intersection is found
        // Merge paths and add to output
        item = stack.head;
        size_t k;
        while (item) {
            if (item->si.theIntersection != nullptr) {
                if (item->si.theIntersection->point.close(current->point)) {
                    merger = new path(); // Clone the current path
                    while (item) {
                        if (item->si.thePath != nullptr) {
                            // We have a segment to merge
                            for (k = 0; k < item->si.thePath->segments.size(); k++) {
                                merger->add_segment(item->si.thePath->segments[k]->clone());
                            }

                            delete item->si.thePath; // Free the path
                        }

                        prevItem = item;
                        item = item->nextItem; // Move to the next item
                        prevItem->remove();
                    }

                    valid_paths.push_back(merger);
                    goto intersection_found; // Exit the loop
                }
            }
            item = item->nextItem; // Move to the next item
        }

        // First instance of intersection, add to stack
        item = stack.add();
        item->si.thePath = nullptr;
        item->si.theIntersection = current; // Mark this intersection

intersection_found:
        prev = current;
    }

    
    if (stack.head) {
        item = stack.head;
        while (item != nullptr) {
            merger = new path(); // Clone the current path
            if (item->si.thePath) {
                for (i=0; i < item->si.thePath->segments.size(); i++) {
                    merger->add_segment(item->si.thePath->segments[i]->clone());
                }
                delete item->si.thePath; // Free the path
            }
            else {
#ifdef DEBUG
                // We have a path without a matching intersection
                std::cout << "DEBUG: Found a path without a matching intersection." << std::endl;
#endif
            }
            item = item->nextItem; // Move to the next item
        }
        valid_paths.push_back(merger);
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
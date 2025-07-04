#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <vector>
#include "math2d.hpp"
#include "pycam.hpp"
#include "geom.hpp"

struct intersection {
    const std::vector<segment*> *path1;
    size_t index1;
    const std::vector<segment*> *path2;
    size_t index2;
    vec2d point;
};

struct segment_info {
    size_t id;
    const std::vector<segment*> *path; // Pointer to the path containing this segment
    segment *seg; // Pointer to the segment itself
    size_t index;
    bounding_box box;
    bool start;
    float x;
};


std::vector<intersection> find_intersections(const std::vector<std::vector<segment*>*> &paths);

#endif
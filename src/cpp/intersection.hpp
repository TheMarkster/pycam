#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <vector>
#include "math2d.hpp"
#include "pycam.hpp"
#include "geom.hpp"

struct intersection {
    const std::vector<segment*> *path1;
    size_t index1;
    float pos1; // Position on the segment in path1
    const std::vector<segment*> *path2;
    size_t index2;
    float pos2; // Position on the segment in path2
    vec2d point;

    // Default constructor
    intersection() : path1(nullptr), index1(0), path2(nullptr), index2(0), point() {}
};

struct segment_info {
    size_t id;
    const std::vector<segment*> *path; // Pointer to the path containing this segment
    segment *seg; // Pointer to the segment itself
    size_t index;
    bounding_box box;
    bool start;
    float x;

    std::string to_string() const {
        std::stringstream ss;
        ss << "Segment:" << std::endl << seg->to_string() << std::endl
           << "index=" << index 
           << ", box=[" << box.xmin << "," << box.xmax << "," << box.ymin << "," << box.ymax << "]"
           << ", start=" << (start ? "true" : "false")
           << ", x=" << x;
        return ss.str();
    }
};


std::vector<intersection> find_intersections(const std::vector<std::vector<segment*>*> &paths);

#endif
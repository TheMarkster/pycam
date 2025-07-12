#include <cmath>
#include <algorithm>
#include <iostream>
#include "geom.hpp"
#include "math2d.hpp"
#include "intersection.hpp"

#define BULGE_90 1.0f // tan(90 degrees / 2)
#define BULGE_60 0.57735026918962576450f // tan(60 degrees / 2)
#define BULGE_45 0.41421356237309504880f // tan(45 degrees / 2)
#define BULGE_30 0.26794919243112270647f // tan(30 degrees / 2)
#define BULGE_22p5 0.19891236738f // tan(22.5 degrees / 2)
#define BULGE_15 0.13165249758f // tan(15 degrees / 2)

std::vector<path*> offset(std::vector<path*> &paths, float distance, bool arc_join, bool cull, bool cleanup = false) {
    std::vector<path*> offset_paths;
    std::vector<path*> temp;
    for (path* p : paths) {
        temp = p->offset(distance, arc_join, cull);
        for (auto &p2 : temp) {
            if (p2->segments.empty()) {
                delete p2; // Clean up empty paths
            } else {
                offset_paths.push_back(p2);
            }
        }

        if (cleanup) {
            delete p;
        }
    }

    if (cleanup) {
        paths.clear(); // Clear the original paths if we are cleaning up
    }

    return offset_paths;
}

int main() {
	path *p = path::from_compact_array({
		compact_point(0.00000, 10.00000, 0.00000),
		compact_point(10.00000, 20.00000, 0.00000),
		compact_point(10.00000, 40.00000, 0.00000),
		compact_point(0.00000, 50.00000, 0.00000),
		compact_point(0.00000, 60.00000, 0.00000),
		compact_point(30.00000, 60.00000, 0.00000),
		compact_point(30.00000, 50.00000, 0.00000),
		compact_point(20.00000, 40.00000, 0.00000),
		compact_point(20.00000, 20.00000, 0.00000),
		compact_point(30.00000, 10.00000, 0.00000),
		compact_point(30.00000, 0.00000, 0.00000),
		compact_point(0.00000, 0.00000, 0.00000),
	}, true);


    std::cout << "Path: " << std::endl;
    std::cout << p->to_string() << std::endl;

    std::vector<path*> paths;
    paths.push_back(p); // Add the original path to the vector

    size_t i;
    for (i=0; i<1; i++) {
        paths = offset(paths, 1, true, true, true);
        
        if (paths.empty()) {
            std::cout << "No valid offset paths found." << std::endl;
            break; // Exit if no valid paths are found
        }

        std::cout << "Path: " << std::endl;
        for (path* offset_path : paths) {
            std::cout << offset_path->to_string() << std::endl;
        }
        std::cout << "----" << std::endl;
    }
    
    for (path* offset_path : paths) {
        delete offset_path; // Clean up allocated memory for each offset path
    }
    return 0;
}
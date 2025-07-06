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


int main() {
    path *p = path::from_compact_array({
        compact_point(5.00000, 3.24609, 0.00000),
        compact_point(2.09566, 6.87652, -0.17030),
        compact_point(1.00000, 10.00000, 0.00000),
        compact_point(1.00000, 18.00000, -0.17030),
        compact_point(2.09566, 21.12348, 0.00000),
        compact_point(5.00000, 24.75390, 0.00000),
        compact_point(5.00000, 23.00000, 0.00000),
        compact_point(5.00000, 23.00000, 0.00000),
        compact_point(5.00000, 24.75391, 0.00000),
        compact_point(7.90434, 21.12348, -0.17030),
        compact_point(9.00000, 18.00000, 0.00000),
        compact_point(9.00000, 10.00000, -0.17030),
        compact_point(7.90434, 6.87652, 0.00000),
        compact_point(5.00000, 3.24609, 0.00000),
        compact_point(5.00000, 5.00000, 0.00000),
        compact_point(5.00000, 5.00000, 0.00000),
    }, true);



    std::cout << "Path: " << std::endl;
    std::cout << p->to_string() << std::endl;

    std::vector<path*> paths = p->get_closed_loops();


    for (path* offset_path : paths) {
        // std::cout << "Offset path with " << offset_path->segments.size() << " segments." << std::endl;
        delete offset_path; // Clean up allocated memory for each offset path
    }

    delete p; // Clean up allocated memory
    return 0;
}
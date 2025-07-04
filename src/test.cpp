#include <cmath>
#include <algorithm>
#include <iostream>
#include "geom.hpp"
#include "math2d.hpp"

#define BULGE_90 1.0f // tan(90 degrees / 2)
#define BULGE_60 0.57735026918962576450f // tan(60 degrees / 2)
#define BULGE_45 0.41421356237309504880f // tan(45 degrees / 2)
#define BULGE_30 0.26794919243112270647f // tan(30 degrees / 2)
#define BULGE_22p5 0.19891236738f // tan(22.5 degrees / 2)
#define BULGE_15 0.13165249758f // tan(15 degrees / 2)


int main() {
    path *p = path::from_compact_array({
        compact_point(8.50000, 4.47383, 0.00000),
        compact_point(4.82870, 9.06296, -0.17030),
        compact_point(4.50000, 10.00000, 0.00000),
        compact_point(4.50000, 18.00000, -0.17030),
        compact_point(4.82870, 18.93704, 0.00000),
        compact_point(8.50000, 23.52617, 0.00000),
        compact_point(8.50000, 26.50000, 0.00000),
        compact_point(1.50000, 26.50000, 0.00000),
        compact_point(1.50000, 23.52617, 0.00000),
        compact_point(5.17130, 18.93704, -0.17030),
        compact_point(5.50000, 18.00000, 0.00000),
        compact_point(5.50000, 10.00000, -0.17030),
        compact_point(5.17130, 9.06296, 0.00000),
        compact_point(1.50000, 4.47383, 0.00000),
        compact_point(1.50000, 1.50000, 0.00000),
        compact_point(8.50000, 1.50000, 0.00000)
    }, true);

    std::vector<path*> loops = p->get_closed_loops();

    for (path *pp : loops) {
        delete pp; // Clean up allocated memory for each loop
    }
    // std::cout << arc->trap_area() << std::endl;
    delete p; // Clean up allocated memory
    return 0;
}
#include "math2d.hpp"

bool bounding_box::intersects(const bounding_box &other) const {
    float _xmin = std::max(other.xmin, xmin);
    float _xmax = std::min(other.xmax, xmax);
    if (_xmin >= _xmax) return false; // No intersection in x-axis

    float _ymin = std::max(other.ymin, ymin);
    float _ymax = std::min(other.ymax, ymax);
    if (_ymin >= _ymax) return false; // No intersection in y-axis

    return true;
}
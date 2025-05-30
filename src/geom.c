#include "geom.h"
#include "math2d.h"


void intersect_line(line *l0, line *l1) {
    vec2d intersection;
    mat2d m = mat_from_rvec(l0->nhat, l1->nhat);
    vec2d s = vec_from_float(l0->s, l1->s);
    intersection = solve(m, s);
    l0->p1 = intersection;
    l1->p0 = intersection;
}
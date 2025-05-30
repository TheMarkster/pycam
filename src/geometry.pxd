# geometry.pxd

cdef extern from "math2d.h":
    cdef struct vec2d:
        float v[2]

    cdef struct mat2d:
        float m[2][2]

    # Constructors
    mat2d mat_from_rvec(vec2d a, vec2d b)
    mat2d mat_from_cvec(vec2d a, vec2d b)

    vec2d cvec1_from_mat(mat2d m)
    vec2d cvec2_from_mat(mat2d m)
    vec2d rvec1_from_mat(mat2d m)
    vec2d rvec2_from_mat(mat2d m)

    vec2d vec_from_float(float x, float y)

    # Operations
    mat2d mm_multiply(mat2d a, mat2d b)
    vec2d mv_multiply(mat2d m, vec2d v)
    vec2d vm_multiply(vec2d v, mat2d m)
    mat2d inverse(mat2d m)
    float determinant(mat2d m)
    float dot(vec2d a, vec2d b)
    vec2d solve(mat2d m, vec2d s)

cdef extern from "geom.h":
    cdef struct line:
        vec2d p0
        vec2d p1
        float s
        vec2d vhat
        vec2d nhat

    void intersect_line(line *l0, line *l1)

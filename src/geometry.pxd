# distutils: language = c++
from libcpp.vector cimport vector
from libc.math cimport sinf, cosf, sqrtf
from libc.float cimport FLT_MAX
from libcpp cimport bool
from libcpp.array cimport array

ctypedef array[float, 3] compact_point

cdef extern from "math2d.hpp" namespace "":
    cdef cppclass vec2d:
        float v[2]
        vec2d()
        vec2d(float x, float y)
        vec2d operator+(const vec2d& rhs) const
        vec2d operator-(const vec2d& rhs) const
        vec2d operator*(float scalar) const
        vec2d operator/(float scalar) const
        vec2d operator-() const
        bool operator==(const vec2d& rhs) const
        bool operator!=(const vec2d& rhs) const
        float length() const
        vec2d normalized() const
        float dot(const vec2d& rhs) const
        float cross(const vec2d& rhs) const

    cdef cppclass mat2d:
        float m[2][2]
        mat2d()
        mat2d(float a, float b, float c, float d)
        @staticmethod
        mat2d from_rvec(const vec2d& a, const vec2d& b)
        @staticmethod
        mat2d from_cvec(const vec2d& a, const vec2d& b)
        mat2d operator*(const mat2d& rhs) const
        vec2d operator*(const vec2d& v) const
        mat2d operator*(float scalar) const
        mat2d operator/(float scalar) const
        mat2d operator+(const mat2d& rhs) const
        mat2d operator-(const mat2d& rhs) const
        mat2d operator-() const
        bool operator==(const mat2d& rhs) const
        bool operator!=(const mat2d& rhs) const
        float determinant() const
        mat2d inverse() const
        mat2d transposed() const

    vec2d solve(const mat2d& m, const vec2d& s)
    vec2d rotate_ccw_90(const vec2d& v)
    vec2d rotate_cw_90(const vec2d& v)
    vec2d rotate_ccw(const vec2d& v, float angle)
    vec2d rotate_cw(const vec2d& v, float angle)
    mat2d rotation_matrix_ccw(float angle)
    mat2d rotation_matrix_cw(float angle)
    float angle(const vec2d &a, const vec2d &b)
    float angle_norm(const vec2d &a, const vec2d &b)
    float angle(const vec2d &a)
    float angle_norm(const vec2d &a)
    


cdef extern from "geom.hpp" namespace "":
    cdef cppclass compact_path:
        vector<compact_point> points

    cdef cppclass bounding_box:
        float xmin
        float xmax
        float ymin
        float ymax

        bounding_box(float xmin, float xmax, float ymin, float ymax)
        bounding_box()
        bool intersects(const bounding_box &other) const
        void expand(float x, float y)
        void translate(float x, float y)

    cdef cppclass segment:
        vec2d start
        vec2d end

        segment(const vec2d& start, const vec2d& end)
        void offset(float distance)
        bounding_box get_bounding_box() const

        bool diverges(const segment& other, float direction) const
        bool diverges_from_line(const line_segment& first, float direction) const
        bool diverges_from_arc(const arc_segment& first, float direction) const

        vec2d intersection(const segment& other) const
        vec2d intersection_with_line(const line_segment& first, bool arg_first) const
        vec2d intersection_with_arc(const arc_segment& first, bool arg_first) const
        bool intersects(const segment& other) const
        bool on_segment(const vec2d& point)

    cdef cppclass line_segment(segment):
        line_segment(const vec2d& start, const vec2d& end)
        void offset(float distance)
        bounding_box get_bounding_box() const
        bool diverges(const segment& other, float direction) const
        bool diverges_from_line(const line_segment& first, float direction) const
        bool diverges_from_arc(const arc_segment& first, float direction) const
        vec2d intersection(const segment& other) const
        vec2d intersection_with_line(const line_segment& other, bool arg_first) const
        vec2d intersection_with_arc(const arc_segment& arc, bool arg_first) const
        bool intersects(const segment& other) const
        bool on_segment(const vec2d& point)

    cdef cppclass arc_segment(segment):
        arc_segment(const vec2d& center, float radius, float start_angle, float end_angle, bool is_clockwise)
        void offset(float distance)
        bounding_box get_bounding_box() const
        bool diverges(const segment& other, float direction) const
        bool diverges_from_line(const line_segment& first, float direction) const
        bool diverges_from_arc(const arc_segment& first, float direction) const
        vec2d intersection(const segment& other) const
        vec2d intersection_with_line(const line_segment& line, bool arg_first) const
        vec2d intersection_with_arc(const arc_segment& other, bool arg_first) const
        bool intersects(const segment& other) const
        bool on_segment(const vec2d& point)

    cdef cppclass bb_index:
        bounding_box box
        segment* seg
        bool start

    cdef cppclass path:
        path()
        void add_segment(segment* seg)
        void insert_segment(size_t index, segment* seg)

        path offset(float distance, bool arc_join)

        @staticmethod
        path from_compact_array(compact_path& cp)
        vector<compact_point> to_compact_array() const
        vector<vec2d> find_intersections()

# distutils: language = c++
from libcpp.vector cimport vector
from libcpp.span cimport span
from libc.math cimport sinf, cosf, sqrtf
from libc.float cimport FLT_MAX
from libcpp cimport bool
from libcpp.string cimport string

cdef extern from "pycam.hpp" namespace "":
    cdef cppclass result[T]:
        T data
        bool success

    cdef cppclass linked_list[T]:
        pass
    
    cdef cppclass linked_item[T]:
        T *si
        linked_item *prevItem
        linked_item *nextItem
        linked_list[T] list
    
    cdef cppclass linkedList[T]:
        linked_item[T] *head
        linked_item[T] *tail

cdef extern from "math2d.hpp" namespace "":
    cdef cppclass vec2d:
        float[2] v
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
        float[2][2] m
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
    cdef cppclass compact_point:
        float[3] data
        float& operator[](size_t i)

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
        result[vec2d] intersects(const segment& other) const
        bool on_segment(const vec2d& point)

        float trap_area() const

        string to_string() const

    cdef cppclass line_segment(segment):
        vec2d nhat
        vec2d vhat
        float s

        line_segment(const vec2d& start, const vec2d& end)
        void offset(float distance)
        bounding_box get_bounding_box() const
        bool diverges(const segment& other, float direction) const
        bool diverges_from_line(const line_segment& first, float direction) const
        bool diverges_from_arc(const arc_segment& first, float direction) const
        vec2d intersection(const segment& other) const
        vec2d intersection_with_line(const line_segment& other, bool arg_first) const
        vec2d intersection_with_arc(const arc_segment& arc, bool arg_first) const
        result[vec2d] intersects(const segment& other) const
        bool on_segment(const vec2d& point)

        string to_string() const

    cdef cppclass arc_segment(segment):
        vec2d center
        vec2d nhat_start
        vec2d nhat_end
        float radius
        float start_angle
        float end_angle

        arc_segment *copy() const

        @staticmethod
        arc_segment* from_compact_point(const compact_point& p0, const compact_point& p1)
        @staticmethod
        arc_segment* arc1(const vec2d & center, const vec2d & point, float angle)
        @staticmethod
        arc_segment* arc2(const vec2d & point1, const vec2d & point2, float radius, bool is_clockwise)
        @staticmethod
        arc_segment* arc3(const vec2d & point1, const vec2d & point2, float angle)
        @staticmethod
        arc_segment* arc4(const vec2d & point1, const vec2d & point2, float bulge)
        @staticmethod
        arc_segment* arc5(const vec2d & point1, const vec2d & point2, const vec2d & point3)
        
        void offset(float distance)
        bounding_box get_bounding_box() const
        bool is_clockwise() const
        bool diverges(const segment& other, float direction) const
        bool diverges_from_line(const line_segment& first, float direction) const
        bool diverges_from_arc(const arc_segment& first, float direction) const
        vec2d intersection(const segment& other) const
        vec2d intersection_with_line(const line_segment& line, bool arg_first) const
        vec2d intersection_with_arc(const arc_segment& other, bool arg_first) const
        result[vec2d] intersects(const segment& other) const
        bool on_segment(const vec2d& point)

        string to_string() const

    cdef cppclass bb_index:
        bounding_box box
        segment* seg
        bool start

    cdef cppclass path:
        vector[segment*] segments

        path()
        void add_segment(segment* seg)
        void insert_segment(size_t index, segment* seg)

        vector[path*] offset(float distance, bool arc_join, bool cull)

        @staticmethod
        path* from_compact_array(const vector[compact_point] &cp, bint close)
        vector[compact_point] to_compact_array() const

        bool clockwise_winding() const
        float signed_area() const

        vector[path*] get_closed_loops()

        string to_string() const

    cdef cppclass segment_info:
        size_t id
        const path *p
        size_t index
        const segment *seg
        bounding_box box
        bool start
        float x
    
    cdef line_segment* as_line(segment* seg)
    cdef arc_segment* as_arc(segment* seg)

cdef extern from "intersection.hpp" namespace "":
    vector[intersection] find_intersections(const vector[vector[segment*]*] &paths)

    cdef cppclass intersection:
        vector[segment*] *path1
        size_t index1
        vector[segment*] *path2
        size_t index2
        vec2d point

    cdef cppclass segment_info:
        size_t id
        vector[segment*] *path
        segment *seg
        size_t index
        bounding_box box
        bool start
        float x


cdef vector_to_numpy_double(vector[double] v)
cdef vector_to_numpy_float(vector[float] v)
cdef vector_to_numpy_compact_point(vector[compact_point] v)
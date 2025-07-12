# distutils: language = c++
from libcpp.vector cimport vector
from libcpp.span cimport span
from libc.math cimport sinf, cosf, sqrtf
from libc.float cimport FLT_MAX
from libcpp cimport bool
from libcpp.string cimport string

cdef extern from "pycam.hpp":
# template <typename T, size_t N>
# struct varray {
#     T data[N];
#     size_t count;
# };

    cdef cppclass varray[T, N]:
        T* data
        size_t count

    cdef cppclass result[T]:
        T data
        bool success

    cdef cppclass linked_list[T]:
        linked_item[T] *head
        linked_item[T] *tail
        linked_item[T]* add()
        linked_item[T]* add(T si)
        size_t size() const
    
    cdef cppclass linked_item[T]:
        T si
        linked_item *prevItem
        linked_item *nextItem
        linked_list[T] *list
        void remove()
        void insert_after(T new_si)
        void insert_before(T new_si)

cdef extern from "math2d.hpp":
    cdef cppclass vec2d:
        float[2] v
        vec2d() except +
        vec2d(float x, float y) except +
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
        bool close(const vec2d& rhs, float epsilon) const
        string to_string() const

    cdef cppclass bounding_box:
        float xmin, xmax, ymin, ymax
        bounding_box() except +
        bounding_box(float xmin, float xmax, float ymin, float ymax) except +
        bool intersects(const bounding_box &other) const
        void expand(float x, float y)
        void translate(float x, float y)

    cdef cppclass mat2d:
        float[2][2] m
        mat2d() except +
        mat2d(float a, float b, float c, float d) except +
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
    float modulo(float x, float m)
    


cdef extern from "geom.hpp":
    

    cdef cppclass compact_point:
        float[3] v
        compact_point() except +
        compact_point(float x, float y, float bulge) except +
        float& operator[](size_t i)
        const float& operator[](size_t i) const

    cdef cppclass intersection_info:
        vec2d point
        float pos1
        float pos2

    # Template specializations for intersection results
    cdef cppclass intersection_result:
        vec2d* data
        size_t count

    cdef cppclass validated_intersection_result:
        intersection_info* data
        size_t count

    cdef cppclass segment:
        vec2d start
        vec2d end

        segment() except +
        segment(const vec2d& start, const vec2d& end) except +
        segment* copy() const
        
        const vec2d& get_start() const
        const vec2d& get_end() const
        const vec2d& get_nhat_start() const
        const vec2d& get_nhat_end() const
        
        bool offset(float distance)
        bounding_box get_bounding_box() const
        compact_point to_compact_point() const

        bool diverges(const segment& other, float direction) const
        validated_intersection_result valid_intersections(const segment& other) const

        intersection_result intersection(const segment& other) const
        intersection_result intersection_with_line(const line_segment& first, bool arg_first) const
        intersection_result intersection_with_arc(const arc_segment& first, bool arg_first) const
        
        float trap_area() const
        float length() const
        float get_position(const vec2d& point) const
        bool valid() const
        bool on_segment(const vec2d& point, float epsilon) const
        
        bool set_start_point(const vec2d& point)
        bool set_end_point(const vec2d& point)
        segment* bisect(const vec2d& point, bool before) const

        string to_string() const

    cdef cppclass line_segment(segment):
        line_segment(const vec2d& start, const vec2d& end) except +
        
        # Accessors for protected members
        const vec2d& get_nhat_start() const
        const vec2d& get_nhat_end() const
        const vec2d& get_nhat() const
        const vec2d& get_vhat() const
        float get_s() const
        
        @staticmethod
        line_segment* from_compact_point(const compact_point& p0, const compact_point& p1)

    cdef cppclass arc_segment(segment):
        # Accessors for protected members
        const float& get_radius() const
        const vec2d& get_center() const
        const vec2d& get_nhat_start() const
        const vec2d& get_nhat_end() const
        float get_start_angle() const
        float get_end_angle() const
        
        @staticmethod
        arc_segment* from_compact_point(const compact_point& p0, const compact_point& p1)
        @staticmethod
        arc_segment* arc1(const vec2d & center, const vec2d & point, float angle)
        @staticmethod
        arc_segment* arc2(const vec2d & point1, const vec2d & point2, float radius)
        @staticmethod
        arc_segment* arc3(const vec2d & point1, const vec2d & point2, float angle)
        @staticmethod
        arc_segment* arc4(const vec2d & point1, const vec2d & point2, float bulge)
        @staticmethod
        arc_segment* arc5(const vec2d & point1, const vec2d & point2, const vec2d & point3)
        
        bool is_clockwise() const
        float bulge() const

    cdef cppclass path:
        vector[segment*] segments

        path() except +
        path* copy() const
        
        void add_segment(segment* seg)
        void add_path(const path& other)
        void insert_segment(size_t index, segment* seg)
        void remove_invalid_segments()
        void cleanup()

        vector[path*] offset(float distance, bool arc_join, bool cull)

        @staticmethod
        path* from_compact_array(const vector[compact_point] &cp, bool close)
        vector[compact_point] to_compact_array() const

        bool clockwise_winding() const
        float signed_area() const

        vector[path*] get_closed_loops()

        string to_string() const

    line_segment* as_line(segment* seg)
    arc_segment* as_arc(segment* seg)
    
    float arc_integral_top(float x0, float x1, float r, float h)
    float arc_integral_bot(float x0, float x1, float r, float h)

cdef extern from "intersection.hpp":
    cdef cppclass intersection:
        vector[segment*] *path1
        vector[segment*] *path2
        size_t index1
        size_t index2
        float pos1
        float pos2
        vec2d point

    cdef cppclass segment_info:
        size_t id
        vector[segment*] *path
        segment *seg
        size_t index
        bounding_box box
        bool start
        float x

    vector[intersection] find_intersections(const vector[vector[segment*]*] &paths)


cdef vector_to_numpy_double(vector[double] v)
cdef vector_to_numpy_float(vector[float] v)
cdef vector_to_numpy_compact_point(vector[compact_point] v)
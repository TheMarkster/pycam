# distutils: language = c++
from libcpp.vector cimport vector
from libc.stdlib cimport malloc, free
import numpy as np
cimport numpy as cnp
from cpython.pycapsule cimport PyCapsule_New, PyCapsule_Destructor

from geometry cimport (
    vec2d, mat2d,
    line_segment, arc_segment, segment,
    bounding_box, path,
    rotate_ccw_90, rotate_cw_90, rotate_ccw, rotate_cw,
    rotation_matrix_ccw, rotation_matrix_cw, compact_point
)

cdef class Vec2D:
    cdef vec2d* cpp_v

    def __cinit__(self, float x=0, float y=0):
        self.cpp_v = new vec2d(x, y)

    def __dealloc__(self):
        del self.cpp_v

    def __repr__(self):
        return f"Vec2D({self.cpp_v.v[0]}, {self.cpp_v.v[1]})"

    cpdef Vec2D _add_vec2d(self, Vec2D other):
        cdef vec2d result = self.cpp_v[0] + other.cpp_v[0]
        return Vec2D(result.v[0], result.v[1])

    def __add__(self, other):
        if isinstance(other, Vec2D):
            return self._add_vec2d(other)

    property x:
        def __get__(self): return self.cpp_v.v[0]
        def __set__(self, float value): self.cpp_v.v[0] = value

    property y:
        def __get__(self): return self.cpp_v.v[1]
        def __set__(self, float value): self.cpp_v.v[1] = value

    def length(self): return self.cpp_v.length()
    def normalized(self): return Vec2D(self.cpp_v.normalized().v[0], self.cpp_v.normalized().v[1])
    def dot(self, Vec2D other): return self.cpp_v.dot(other.cpp_v[0])
    def cross(self, Vec2D other): return self.cpp_v.cross(other.cpp_v[0])


cdef class Mat2D:
    cdef mat2d* cpp_m

    def __cinit__(self, float a=1, float b=0, float c=0, float d=1):
        self.cpp_m = new mat2d(a, b, c, d)

    def __dealloc__(self):
        del self.cpp_m

    def __repr__(self):
        m = self.cpp_m.m
        return f"Mat2D([{m[0][0]}, {m[0][1]}], [{m[1][0]}, {m[1][1]}])"

    def determinant(self):
        return self.cpp_m.determinant()

    def inverse(self):
        cdef mat2d inv = self.cpp_m.inverse()
        return Mat2D(inv.m[0][0], inv.m[0][1], inv.m[1][0], inv.m[1][1])


cdef class BoundingBox:
    cdef bounding_box* cpp_box

    def __cinit__(self, float xmin=0, float xmax=0, float ymin=0, float ymax=0):
        self.cpp_box = new bounding_box(xmin, xmax, ymin, ymax)

    def __dealloc__(self):
        del self.cpp_box

    def __repr__(self):
        b = self.cpp_box
        return f"BoundingBox(xmin={b.xmin}, xmax={b.xmax}, ymin={b.ymin}, ymax={b.ymax})"

    def intersects(self, BoundingBox other):
        return self.cpp_box.intersects(other.cpp_box[0])
    
    def expand(self, x, y):
        self.cpp_box.expand(x, y)
    
    def translate(self, x, y):
        self.cpp_box.translate(x, y)


cdef class Segment:
    """Abstract base class. Not instantiable directly."""
    cdef segment* cpp_seg
    cdef bint owner
    
    def get_bounding_box(self):
        cdef bounding_box bb = self.cpp_seg.get_bounding_box()
        return BoundingBox(bb.xmin, bb.xmax, bb.ymin, bb.ymax)

    def intersection(self, Segment other):
        cdef vec2d pt = self.cpp_seg.intersection(other.cpp_seg[0])
        return Vec2D(pt.v[0], pt.v[1])
    
    def intersects(self, Segment other):
        return self.cpp_seg.intersects(other.cpp_seg[0])


cdef class LineSegment(Segment):
    def __cinit__(self, Vec2D start, Vec2D end):
        self.cpp_seg = new line_segment(start.cpp_v[0], end.cpp_v[0])
        self.owner = True

    def __dealloc__(self):
        if self.owner:
            del self.cpp_seg


cdef class ArcSegment(Segment):
    def __cinit__(self, Vec2D center, float radius, float start_angle, float end_angle, bint clockwise):
        self.cpp_seg = new arc_segment(center.cpp_v[0], radius, start_angle, end_angle, clockwise)
        self.owner = True
    
    @property
    def start(self):
        v = Vec2D(self.cpp_seg.start.v[0], self.cpp_seg.start.v[1])
        return v

    @start.setter
    def start(self, value: Vec2D):
        self.cpp_seg.start = value.cpp_v[0]

    def __dealloc__(self):
        if self.owner:
            del self.cpp_seg


cdef class Path:
    cdef path* cpp_path

    def __cinit__(self):
        self.cpp_path = new path()

    def __dealloc__(self):
        del self.cpp_path

    def find_intersections(self):
        self.cpp_path.find_intersections()

    @staticmethod
    def from_compact_array(cnp.ndarray[cnp.float32_t, ndim=2] arr):
        if arr.shape[1] != 3:
            raise ValueError("Array must have shape (N, 3)")

        cdef size_t n = arr.shape[0]
        cdef vector[compact_point] vec
        vec.reserve(n)

        cdef int i
        cdef compact_point pt
        for i in range(n):
            pt[0] = arr[i, 0]
            pt[1] = arr[i, 1]
            pt[2] = arr[i, 2]
            vec.push_back(pt)
        
        cdef Path p = Path.__new__(Path)
        cdef path *cpp_path = path.from_compact_array(vec)
        p.cpp_path = cpp_path
        return p

    def to_compact_array(self):
        cdef vector[compact_point] cp = self.cpp_path.to_compact_array()
        cdef np_arr = vector_to_numpy_compact_point(cp)

        return np_arr

cdef vector_to_numpy_double(vector[double] v):
    """Convert a C++ vector of doubles to a NumPy array."""
    cdef cnp.ndarray[cnp.double_t, ndim=1] arr = np.empty(len(v), dtype=np.double)
    cdef int i
    for i in range(len(v)):
        arr[i] = v[i]
    return arr

cdef vector_to_numpy_float(vector[float] v):
    """Convert a C++ vector of floats to a NumPy array."""
    cdef cnp.ndarray[cnp.float_t, ndim=1] arr = np.empty(len(v), dtype=np.float32)
    cdef int i
    for i in range(len(v)):
        arr[i] = v[i]
    return arr

cdef vector_to_numpy_compact_point(vector[compact_point] v):
    """Convert a C++ vector of compact_point to a NumPy array."""
    cdef size_t n = v.size()
    cdef cnp.ndarray[cnp.float32_t, ndim=2] arr = np.empty((n, 3), dtype=np.float32)
    cdef int i
    cdef compact_point pt
    for i in range(n):
        pt = v[i]
        arr[i, 0] = pt[0]
        arr[i, 1] = pt[1]
        arr[i, 2] = pt[2]
    return arr
# distutils: language = c++
from libcpp.vector cimport vector
from libc.stdlib cimport malloc, free
import numpy as np
cimport numpy as cnp
from cpython.pycapsule cimport PyCapsule_New, PyCapsule_Destructor
from typing import List


from geometry cimport (
    vec2d, mat2d,
    line_segment, arc_segment, segment,
    bounding_box, path,
    rotate_ccw_90, rotate_cw_90, rotate_ccw, rotate_cw,
    rotation_matrix_ccw, rotation_matrix_cw, compact_point,
    as_line, as_arc,
    intersection, find_intersections,
    intersection_result, validated_intersection_result, intersection_info
)

def squeeze(arr):
    if len(arr) == 0:
        return None
    elif len(arr) == 1:
        return arr[0]
    else:
        return arr

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

    @property
    def x(self) -> float:
        return self.cpp_v.v[0]

    @x.setter
    def x(self, value: float):
        self.cpp_v.v[0] = value
    
    @property
    def y(self) -> float:
        return self.cpp_v.v[1]

    @y.setter
    def y(self, value: float):
        self.cpp_v.v[1] = value

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

    @property
    def start(self) -> Vec2D:
        return Vec2D(self.cpp_seg.start.v[0], self.cpp_seg.start.v[1])

    @property
    def end(self) -> Vec2D:
        return Vec2D(self.cpp_seg.end.v[0], self.cpp_seg.end.v[1])
    
    def get_bounding_box(self):
        cdef bounding_box bb = self.cpp_seg.get_bounding_box()
        return BoundingBox(bb.xmin, bb.xmax, bb.ymin, bb.ymax)

    def intersection(self, Segment other):
        cdef intersection_result result = self.cpp_seg.intersection(other.cpp_seg[0])
        intersections = []
        for i in range(result.count):
            intersections.append(Vec2D(result.data[i].v[0], result.data[i].v[1]))

        return squeeze(intersections)
    
    def trap_area(self) -> float:
        if self.cpp_seg is not NULL:
            return self.cpp_seg.trap_area()
        return 0.0
    
    def c_to_string(self):
        """Convert the segment to a string representation using C++."""
        if self.cpp_seg is NULL:
            raise ValueError("Segment is not initialized")
        return self.cpp_seg.to_string().decode('utf-8')


cdef class LineSegment(Segment):
    def __cinit__(self, Vec2D start, Vec2D end):
        self.cpp_seg = new line_segment(start.cpp_v[0], end.cpp_v[0])

    def __dealloc__(self):
        del self.cpp_seg

    @property
    def nhat(self) -> Vec2D:
        cdef line_segment *seg = <line_segment*>self.cpp_seg
        cdef vec2d nhat_val = seg.get_nhat()
        return Vec2D(nhat_val.v[0], nhat_val.v[1])
    
    @property
    def vhat(self) -> Vec2D:
        cdef line_segment *seg = <line_segment*>self.cpp_seg
        cdef vec2d vhat_val = seg.get_vhat()
        return Vec2D(vhat_val.v[0], vhat_val.v[1])
    
    @property
    def s(self) -> float:
        cdef line_segment *seg = <line_segment*>self.cpp_seg
        return seg.get_s()
    
    def to_dict(self):
        """Convert the line segment to a dictionary representation."""
        cdef line_segment *seg = <line_segment*>self.cpp_seg
        cdef vec2d start_val, end_val, nhat_val, vhat_val
        start_val = seg.get_start()
        end_val = seg.get_end()
        nhat_val = seg.get_nhat()
        vhat_val = seg.get_vhat()
        return {
            'start': (start_val.v[0], start_val.v[1]),
            'end': (end_val.v[0], end_val.v[1]),
            'nhat': (nhat_val.v[0], nhat_val.v[1]),
            'vhat': (vhat_val.v[0], vhat_val.v[1]),
        }
        
    def __dealloc__(self):
        if self.owner:
            del self.cpp_seg


cdef class ArcSegment(Segment):
    def __cinit__(self):
        pass
    
    def __dealloc__(self):
        if self.cpp_seg is not NULL:
            del self.cpp_seg

    @staticmethod
    def arc1(Vec2D center, Vec2D point, float angle):
        cdef arc_segment *arc = arc_segment.arc1(center.cpp_v[0], point.cpp_v[0], angle)
        seg = ArcSegment()
        seg.cpp_seg = arc
        return seg

    @staticmethod
    def arc2(point1: Vec2D, point2: Vec2D, radius):
        cdef arc_segment *seg = arc_segment.arc2(point1.cpp_v[0], point2.cpp_v[0], radius)
        if seg is not NULL:
            arc_seg = ArcSegment()
            arc_seg.cpp_seg = seg
            return arc_seg

        raise ValueError("Failed to create arc or line segment from points")
    
    @staticmethod
    def arc3(point1: Vec2D, point2: Vec2D, angle: float):
        seg = ArcSegment()
        cdef arc_segment *arc = arc_segment.arc3(point1.cpp_v[0], point2.cpp_v[0], angle)
        seg.cpp_seg = arc
        return seg
    
    @staticmethod
    def arc4(point1: Vec2D, point2: Vec2D, bulge: float):
        seg = ArcSegment()
        cdef arc_segment *arc = arc_segment.arc4(point1.cpp_v[0], point2.cpp_v[0], bulge)
        seg.cpp_seg = arc
        return seg
    
    @staticmethod
    def arc5(point1: Vec2D, point2: Vec2D, point3: Vec2D):
        seg = ArcSegment()
        cdef arc_segment *arc = arc_segment.arc5(point1.cpp_v[0], point2.cpp_v[0], point3.cpp_v[0])
        seg.cpp_seg = arc
        return seg
    
    def _check_valid(self):
        if self.cpp_seg is NULL:
            raise ValueError("Segment is not initialized")

    @property
    def nhat_start(self):
        self._check_valid()
        cdef arc_segment *seg = <arc_segment*>self.cpp_seg
        cdef vec2d nhat_start_val = seg.get_nhat_start()
        return Vec2D(nhat_start_val.v[0], nhat_start_val.v[1])
    
    @property
    def nhat_end(self) -> Vec2D:
        self._check_valid()
        cdef arc_segment *seg = <arc_segment*>self.cpp_seg
        cdef vec2d nhat_end_val = seg.get_nhat_end()
        return Vec2D(nhat_end_val.v[0], nhat_end_val.v[1])
    
    @property
    def center(self) -> Vec2D:
        self._check_valid()
        cdef arc_segment *seg = <arc_segment*>self.cpp_seg
        cdef vec2d center_val = seg.get_center()
        return Vec2D(center_val.v[0], center_val.v[1])
    
    @property
    def start(self) -> Vec2D:
        self._check_valid()
        cdef arc_segment *seg = <arc_segment*>self.cpp_seg
        cdef vec2d start_val = seg.get_start()
        return Vec2D(start_val.v[0], start_val.v[1])
    
    @property
    def radius(self) -> float:
        self._check_valid()
        cdef arc_segment *seg = <arc_segment*>self.cpp_seg
        return seg.get_radius()

    @property
    def start_angle(self) -> float:
        self._check_valid()
        cdef arc_segment *seg = <arc_segment*>self.cpp_seg
        return seg.get_start_angle()

    @property
    def end_angle(self) -> float:
        self._check_valid()
        cdef arc_segment *seg = <arc_segment*>self.cpp_seg
        return seg.get_end_angle()

    @property
    def is_clockwise(self) -> bool:
        self._check_valid()
        cdef arc_segment *seg = <arc_segment*>self.cpp_seg
        return seg.is_clockwise()

    def to_dict(self):
        """Convert the arc segment to a dictionary representation."""
        self._check_valid()
        cdef arc_segment *seg = <arc_segment*>self.cpp_seg
        cdef vec2d center_val = seg.get_center()
        cdef vec2d start_val = seg.get_start()
        cdef vec2d end_val = seg.get_end()
        cdef vec2d nhat_start_val = seg.get_nhat_start()
        cdef vec2d nhat_end_val = seg.get_nhat_end()
        return {
            'center': (center_val.v[0], center_val.v[1]),
            'start': (start_val.v[0], start_val.v[1]),
            'end': (end_val.v[0], end_val.v[1]),
            'radius': seg.get_radius(),
            'start_angle': seg.get_start_angle(),
            'end_angle': seg.get_end_angle(),
            'nhat_start': (nhat_start_val.v[0], nhat_start_val.v[1]),
            'nhat_end': (nhat_end_val.v[0], nhat_end_val.v[1]),
            'is_clockwise': seg.is_clockwise()
        }

    def __dealloc__(self):
        if self.owner:
            del self.cpp_seg


cdef class Path:
    cdef path* cpp_path

    def __cinit__(self, empty=False):
        """Initialize a new Path. If empty is True, creates an empty path."""
        if empty:
            self.cpp_path = NULL
        else:
            self.cpp_path = new path()

    def __dealloc__(self):
        if self.cpp_path is not NULL:
            del self.cpp_path
            self.cpp_path = NULL
    
    def clockwise_winding(self) -> bool:
        """Check if the path has clockwise winding."""
        if self.cpp_path is NULL:
            raise ValueError("Path is not initialized")
        return self.cpp_path.clockwise_winding()

    def signed_area(self) -> float:
        """Calculate the signed area of the path."""
        if self.cpp_path is NULL:
            raise ValueError("Path is not initialized")
        return self.cpp_path.signed_area()

    def offset(self, float distance, arc: bool = True, cull: bool = True) -> List['Path']:
        if self.cpp_path is NULL:
            raise ValueError("Path is not initialized")
        
        cdef vector[path*] cpp_path = self.cpp_path.offset(distance, arc, cull)
        cdef size_t n = cpp_path.size()

        out = []
        for i in range(n):
            p = Path(True)
            p.cpp_path = cpp_path[i]
            out.append(p)

        return out

    def segments(self) -> List[Segment]:
        l = list()
        cdef vector[segment*] segs = self.cpp_path.segments
        cdef size_t n = segs.size()

        cdef line_segment *line
        cdef arc_segment *arc
        cdef segment* s
        cdef vec2d start_val, end_val
        cdef int i
        for i in range(n):
            s = segs[i]
            line = as_line(s)
            arc = as_arc(s)
            if (line != NULL):
                start_val = line.get_start()
                end_val = line.get_end()
                l.append(LineSegment(
                    Vec2D(start_val.v[0], start_val.v[1]),
                    Vec2D(end_val.v[0], end_val.v[1])
                ))
            else:
                temp = ArcSegment()
                temp.cpp_seg = arc.copy()
                l.append(temp)

        return l
    
    def get_closed_loops(self) -> List['Path']:
        """Get closed loops from the path."""
        if self.cpp_path is NULL:
            raise ValueError("Path is not initialized")
        
        cdef vector[path*] cpp_loops = self.cpp_path.get_closed_loops()
        cdef size_t n = cpp_loops.size()
        loops = []

        for i in range(n):
            p = Path(True)
            p.cpp_path = cpp_loops[i]
            loops.append(p)

        return loops

    @staticmethod
    def from_compact_array(cnp.ndarray[cnp.float32_t, ndim=2] arr, close: bool = True):
        if arr.shape[1] != 3:
            raise ValueError("Array must have shape (N, 3)")

        cdef size_t n = arr.shape[0]
        cdef vector[compact_point] vec
        vec.reserve(n)

        cdef int i
        cdef compact_point pt
        for i in range(n):
            pt = compact_point(arr[i, 0], arr[i, 1], arr[i, 2])
            vec.push_back(pt)
        
        cdef path *cpp_path = path.from_compact_array(vec, close)
        p = Path(True)
        p.cpp_path = cpp_path
        return p

    def to_compact_array(self):
        cdef vector[compact_point] cp = self.cpp_path.to_compact_array()
        cdef np_arr = vector_to_numpy_compact_point(cp)

        return np_arr
    
    def c_to_string(self):
        """Convert the path to a string representation using C++."""
        if self.cpp_path is NULL:
            raise ValueError("Path is not initialized")
        return self.cpp_path.to_string().decode('utf-8')

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

def intersections(List[Path] paths):
    """Find intersections between multiple paths."""
    cdef vector[vector[segment*]*] cpp_paths
    cdef size_t i, j
    cdef Path p

    cdef vector[segment*] *segs
    for i in range(len(paths)):
        p = paths[i]
        if p.cpp_path is NULL:
            continue
        segs = &p.cpp_path.segments
        cpp_paths.push_back(segs)

    cdef vector[intersection] result = find_intersections(cpp_paths)

    out = []
    cdef size_t path1_index, path2_index
    cdef intersection inter
    cdef size_t path1_address, path2_address
    for j in range(len(paths)):
        p = paths[j]
        path1_address = <size_t>(&p.cpp_path.segments)

    for i in range(result.size()):
        inter = result[i]

        for j in range(len(paths)):
            p = paths[j]
            if &p.cpp_path.segments == inter.path1:
                path1_index = j
            if &p.cpp_path.segments == inter.path2:
                path2_index = j
        # Convert pointer address to integer
        # This is a workaround to get the memory address of the path segments
        path1_address = <size_t>inter.path1
        path2_address = <size_t>inter.path2
        out.append({
            'path1': paths[path1_index],
            'index1': inter.index1,
            'path2': paths[path2_index],
            'index2': inter.index2,
            'point': Vec2D(inter.point.v[0], inter.point.v[1])
        })
    return out
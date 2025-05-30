# geometry.pyx

cimport geometry  # import all C declarations from geometry.pxd

cdef class Vec2D:
    cdef geometry.vec2d c_vec

    def __cinit__(self, float x=0, float y=0):
        self.c_vec.v[0] = x
        self.c_vec.v[1] = y

    @property
    def x(self):
        return self.c_vec.v[0]

    @x.setter
    def x(self, float value):
        self.c_vec.v[0] = value

    @property
    def y(self):
        return self.c_vec.v[1]

    @y.setter
    def y(self, float value):
        self.c_vec.v[1] = value

    def __repr__(self):
        return f"Vec2D({self.x}, {self.y})"


cdef class Line:
    cdef geometry.line c_line

    def __cinit__(self, Vec2D p0=None, Vec2D p1=None, float s=0,
                  Vec2D vhat=None, Vec2D nhat=None):
        if p0 is None:
            p0 = Vec2D()
        if p1 is None:
            p1 = Vec2D()
        if vhat is None:
            vhat = Vec2D()
        if nhat is None:
            nhat = Vec2D()

        self.c_line.p0 = p0.c_vec
        self.c_line.p1 = p1.c_vec
        self.c_line.s = s
        self.c_line.vhat = vhat.c_vec
        self.c_line.nhat = nhat.c_vec

    def intersect(self, Line other):
        geometry.intersect_line(&self.c_line, &other.c_line)

    def __repr__(self):
        return (f"Line(p0={Vec2D(self.c_line.p0.v[0], self.c_line.p0.v[1])}, "
                f"p1={Vec2D(self.c_line.p1.v[0], self.c_line.p1.v[1])}, "
                f"s={self.c_line.s})")

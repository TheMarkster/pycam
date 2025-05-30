"""
Lines are represented as two points
Arcs are represented as a center point, radius, start angle, and total angle. Sign of the angle indicates direction.

All data is stored in a single Nx4 array.
"""

import numpy as np
import numba as nb

@nb.njit
def determinant(v1, v2):
    """Calculate the determinant of a 2x2 matrix."""
    return v1[0]*v2[1] - v1[1]*v2[0]

@nb.njit
def determinant2(lh11, lh12, lh21, lh22):
    """Calculate the determinant of a 2x2 matrix."""
    return lh11*lh22 - lh12*lh21

@nb.njit
def solve(v1, v2, c1, c2):
    D = determinant(v1, v2)
    s1 = (c1*v2[1]-c2*v1[1]) / D
    s2 = (c2*v1[0]-c1*v2[0]) / D
    return np.array([s1, s2])

@nb.njit
def solve2(lh11, lh12, lh21, lh22, rh1, rh2):
    D = lh11*lh21 - lh12*lh22
    s1 = (rh1*lh22-rh2*lh12) / D
    s2 = (rh2*lh11-rh1*lh21) / D
    return s1, s2

@nb.njit
def norm2(v):
    return v[0]**2 + v[1]**2

@nb.njit
def norm(v):
    return np.sqrt(norm2(v))

@nb.njit
def dot(v1, v2):
    return v1[0]*v2[0] + v1[1]*v2[1]

@nb.njit
def cross(v1, v2):
    return v1[0]*v2[1] - v1[1]*v2[0]

@nb.njit(inline="always")
def line2circle(points, lineIndex, circleIndex):
    """Find the intersection between a line and a circle."""
    for i in range(len(lineIndex)):
        linePoint0 = points[lineIndex[i], :2]
        linePoint1 = points[lineIndex[i], 2:4]

        lineVector = linePoint1 - linePoint0
        lineVectorNorm = lineVector / np.linalg.norm(lineVector)
        lineNormal = np.array([-lineVectorNorm[1], lineVectorNorm[0]])

        arcCenter = points[circleIndex[i], :2]
        radius = points[circleIndex[i], 2]

        c1 = lineNormal.dot(linePoint0)
        c2 = lineVectorNorm.dot(arcCenter)

        intersection = solve(lineNormal, lineVectorNorm, c1, c2)
        centerVector = np.linalg.norm(intersection-arcCenter)
        off = np.sqrt(radius**2 - np.sum(centerVector**2))

        solutions = intersection + off*lineVectorNorm, intersection - off*lineVectorNorm

@nb.njit
def phase(v):
    vnorm = v / norm(v)
    phase = np.arccos(vnorm[0])
    if vnorm[1] < 0:
        phase = 2*np.pi - phase
    return phase


@nb.njit
def bulge2arc(p0, p1, bulge, normalized=True):
    v01 = p1 - p0
    chord_length2 = norm2(v01)
    chord_length = np.sqrt(chord_length2)
    if normalized:
        bulge = bulge * chord_length
    v01hat = v01 / chord_length
    normal = np.array([-v01hat[1], v01hat[0]])
    center_dist = (chord_length2/4 + bulge**2)/(2*bulge)
    chord_center = (p0 + p1) / 2
    center = chord_center - center_dist*normal
    radius = norm(p0-center)
    
    cw = bulge > 0

    phase0 = phase(p0 - center)
    phase1 = phase(p1 - center)
    dphase = (phase1 - phase0) % (2*np.pi)
    if cw:
        dphase = dphase - 2*np.pi
    else:
        radius = -radius
        phase0 += np.pi
        # dphase = -dphase

    return center, radius, phase0, dphase
    # return center, cw


@nb.njit
def calcBulge(radius, dphase):
    return (radius - np.cos(dphase/2)) / (2 * np.sin(dphase/2))

@nb.njit
def calcRadius(chord_len, bulge):
    bulge = bulge * chord_len
    # chord_len = radius*np.sin(dphase/2)*2
    radius = (chord_len**2/4 + bulge**2)/(2*bulge)
    dphase = np.asin(chord_len/(2*radius))*2 
    return radius, dphase

@nb.njit
def arc2bulge(center, radius, phase0, dphase):
    p0 = center + radius*np.array([np.cos(phase0), np.sin(phase0)])
    p1 = center + radius*np.array([np.cos(phase0 + dphase), np.sin(phase0 + dphase)])
    pbulge = center + radius*np.array([np.cos(phase0 + dphase/2), np.sin(phase0 + dphase/2)])
    chord_center = (p0 + p1) / 2
    vbulge = pbulge - chord_center
    v01 = p1 - p0
    bulge = norm(vbulge) / norm(v01)
    if cross(v01, vbulge) < 0:
        bulge = -bulge
    return p0, p1, bulge

# @nb.njit
# def intersection(p0, v1, c, r, offset=0, solution=0):
#     v1_orth = np.array([v1[1], -v1[0]])
#     c1 = dot(v1_orth, p0)
#     c2 = dot(v1, c)

#     rintersection = solve(v1_orth, v1, c1+offset, c2)
#     rvec = rintersection - c
#     rlen2 = norm2(rvec)
#     r2 = (r+offset)**2
#     if rlen2 > r2:
#         return np.array([np.nan, np.nan])
#     else:
#         adj = np.sqrt(r2 - rlen2)

#         if solution == -1:
#             return rintersection - adj * v1
#         elif solution == 1:
#             return rintersection + adj * v1
#         else:
#             return rintersection - adj * v1, rintersection + adj * v1

@nb.njit
def line_intersect_arc(lineData, arcData, lineIndex, arcIndex, reverse=False):
    p0_line = lineData[lineIndex, 0:2]
    p1_line = lineData[lineIndex, 2:4]

    c = arcData[arcIndex, 0:2]
    r = arcData[arcIndex, 2]
    phase0 = arcData[arcIndex, 3]
    dphase = arcData[arcIndex, 4]

    v_line = p1_line - p0_line
    v_line_hat = v_line / g.norm(v_line)
    n_line_hat = np.array([-v_line_hat[1], v_line_hat[0]])

    # Find midpoint of possible solutions
    s1 = g.dot(n_line_hat, p0_line)
    s2 = g.dot(v_line_hat, c)
    intersection = g.solve(n_line_hat, v_line_hat, s1, s2)
    a = g.norm(intersection - c)
    b = (r**2 - a**2)**0.5 # Distance from midpoint to solutions

    # Determine which solution to use
    if reverse:
        phase = phase0 + dphase
    else:
        phase = phase0
    varc = np.array([np.cos(phase), np.sin(phase)])
    side = cross(varc, n_line_hat) # Positive means it's the closest solution


    if side < 0:
        # Closest solution
        intersection = intersection - b * v_line_hat
    else:
        # Farthest solution
        intersection = intersection + b * v_line_hat


    # Find phase change
    varc_p = intersection-c
    varc_p = varc_p / norm(varc_p)
    if g.cross(varc, varc_p) > 0:
        # ccw rotation
        dphase_p = np.acos(dot(varc, varc_p))
    else:
        # cw rotation
        dphase_p = -np.acos(dot(varc, varc_p))

    # Record intersected point
    if reverse:
        arcData[arcIndex, 4] += dphase_p
        lineData[lineIndex, 0] = intersection[0]
        lineData[lineIndex, 1] = intersection[1]
    else:
        arcData[arcIndex, 3] += dphase_p
        arcData[arcIndex, 4] -= dphase_p
        lineData[lineIndex, 2] = intersection[0]
        lineData[lineIndex, 3] = intersection[1]

class Path:
    """Path class to store lines and arcs"""
    def __init__(self):
        self.line_segments = np.empty((0, 4), np.float64)
        self.arc_segments = np.empty((0, 5), np.float64)
        self.segment_index = np.empty((0, 2), np.int32)

    @staticmethod
    def load_compact_path(path):
        """Compact arrays are standard storage from a DXF and have dimensions Nx4.

        First two columns are the points along the path.
        Third and fourth columns represent the starting and ending line width.
        The last column is the bulge factor."""
        pass

    def join(self, startIndex, stopIndex, joinMethod=0):
        """Joins adjacent segments together. Used after modifying the path.

        Inputs:
            startIndex: Index of the first segment to join.
            stopIndex: Index of the last segment to join.
            joinMethod: Method to use for joining segments.
                0: Join with miters.
                1: Join with arcs.
        """
        
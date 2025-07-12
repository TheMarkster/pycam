"""
PyCam - A Python CAM (Computer-Aided Manufacturing) library
"""

__version__ = "0.1.0"
__author__ = "Mark Greene"
__email__ = "markdanielgreene@gmail.com"

# Try to import the compiled geometry module
try:
    from .core.geometry import *
    _has_geometry = True
except ImportError:
    _has_geometry = False
    import warnings
    warnings.warn(
        "Could not import compiled geometry module. "
        "Please ensure the C++/Cython extensions are built.",
        ImportWarning
    )

# Import other modules
from . import gcode

__all__ = [
    "gcode",
]

# Add geometry exports if available
if _has_geometry:
    # These would be defined in your geometry.pyx file
    # __all__.extend(["Path", "LineSegment", "ArcSegment", ...])
    pass

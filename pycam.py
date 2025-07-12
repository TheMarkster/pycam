"""
PyCam - A Python CAM (Computer-Aided Manufacturing) library
"""

__version__ = "0.1.0"
__author__ = "Mark Greene"
__email__ = "markdanielgreene@gmail.com"

# Import the compiled geometry module
try:
    from geometry import *
    __all__ = ["geometry"]
except ImportError:
    import warnings
    warnings.warn(
        "Could not import compiled geometry module. "
        "Please ensure the C++/Cython extensions are built.",
        ImportWarning
    )
    __all__ = []

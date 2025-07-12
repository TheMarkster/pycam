#!/usr/bin/env python3
"""
Basic usage example for PyCam
"""

import pycam

def main():
    print(f"PyCam version: {pycam.__version__}")
    
    # Check if geometry module is available
    if hasattr(pycam, '_has_geometry') and pycam._has_geometry:
        print("Geometry module is available")
        # Add geometry usage examples here
    else:
        print("Geometry module is not available - please build the C++/Cython extensions")
    
    # Example with gcode module
    print("GCode module available:", hasattr(pycam, 'gcode'))

if __name__ == "__main__":
    main()

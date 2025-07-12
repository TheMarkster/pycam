#!/usr/bin/env python3
"""
Setup script for pycam - handles C++/Cython compilation
"""
import os
import sys
from pathlib import Path

import numpy as np
from Cython.Build import cythonize
from setuptools import Extension, setup

# Define paths
project_root = Path(__file__).parent
cpp_src_dir = project_root / "src" / "cpp"
cython_src_dir = project_root / "cython"

# C++ source files (relative paths)
cpp_sources = [
    "src/cpp/geom.cpp",
    "src/cpp/math2d.cpp",
    "src/cpp/intersection.cpp",
    "src/cpp/pycam.cpp",
]

# Cython source files (relative paths)
cython_sources = [
    "cython/geometry.pyx",
]

# All source files for the extension
all_sources = cpp_sources + cython_sources

# Include directories (relative paths)
include_dirs = [
    "src/cpp",
    "cython",
    np.get_include(),
]

# Compiler arguments
extra_compile_args = [
    "-std=c++17",
    "-O3",
    "-Wall",
    "-Wextra",
    "-fPIC",
]

# Linker arguments
extra_link_args = []

# Platform-specific settings
if sys.platform == "win32":
    extra_compile_args.extend(["/std:c++17", "/O2"])
    extra_compile_args = [arg for arg in extra_compile_args if not arg.startswith("-")]
elif sys.platform == "darwin":
    extra_compile_args.extend(["-stdlib=libc++"])
    extra_link_args.extend(["-stdlib=libc++"])

# Define the extension - place in the same directory as pycam.py
extensions = [
    Extension(
        name="geometry",
        sources=all_sources,
        include_dirs=include_dirs,
        extra_compile_args=extra_compile_args,
        extra_link_args=extra_link_args,
        language="c++",
        define_macros=[("NPY_NO_DEPRECATED_API", "NPY_1_7_API_VERSION")],
    )
]

# Cythonize extensions
extensions = cythonize(
    extensions,
    compiler_directives={
        "language_level": "3",
        "boundscheck": False,
        "wraparound": False,
        "cdivision": True,
        "embedsignature": True,
    },
    include_path=["cython"],
)

if __name__ == "__main__":
    setup(
        ext_modules=extensions,
        zip_safe=False,
    )

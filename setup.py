from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy
import os

SRC_DIR = "src"

extensions = [
    Extension(
        "geometry",
        sources=[
            os.path.join(SRC_DIR, "geometry.pyx"),
            os.path.join(SRC_DIR, "geom.cpp")
        ],
        include_dirs=[SRC_DIR, numpy.get_include()],
        language="c++",
        extra_compile_args=["/std:c++20"],
    )
]

setup(
    name="geometry",
    ext_modules=cythonize(
        extensions,
        compiler_directives={"language_level": "3"},
    ),
    zip_safe=False,
)

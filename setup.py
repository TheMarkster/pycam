from setuptools import setup, Extension
from Cython.Build import cythonize
import os

SRC_DIR = "src"

extensions = [
    Extension(
        "geometry",
        sources=[
            os.path.join(SRC_DIR, "geometry.pyx"),
            os.path.join(SRC_DIR, "geom.cpp")
        ],
        include_dirs=[SRC_DIR],
        language="c++",
        extra_compile_args=["-std=c++17"],
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

from setuptools import setup, Extension
from Cython.Build import cythonize
import os

extensions = [
    Extension(
        "geometry",
        sources=[os.path.join("src", "geometry.pyx")],
        include_dirs=[os.path.join("src")],  # if your headers are there
    )
]

setup(
    name="geometry",
    ext_modules=cythonize(extensions),
    zip_safe=False,
)

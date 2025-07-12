# PyCam - Python CAM Library

A Python library for Computer-Aided Manufacturing (CAM) with high-performance C++ and Cython extensions for geometric operations.

## Project Structure

```
pycam/
├── src/
│   ├── pycam/                 # Main Python package
│   │   ├── __init__.py
│   │   ├── core/              # Core functionality (C++/Cython)
│   │   │   ├── __init__.py
│   │   │   └── geometry.py    # Python wrappers
│   │   └── gcode/             # G-code generation
│   │       ├── __init__.py
│   │       ├── types.py
│   │       └── program/
│   └── cpp/                   # C++ source files
│       ├── geom.cpp
│       ├── geom.hpp
│       ├── math2d.cpp
│       ├── math2d.hpp
│       ├── intersection.cpp
│       ├── intersection.hpp
│       ├── pycam.cpp
│       └── pycam.hpp
├── cython/                    # Cython extension files
│   ├── geometry.pyx
│   └── geometry.pxd
├── tests/                     # Test files
├── examples/                  # Example usage
├── notebooks/                 # Jupyter notebooks
├── pyproject.toml            # Project configuration
├── setup.py                  # Build configuration
├── Makefile                  # Build automation
└── build.py                  # Build script
```

## Building the Project

### Prerequisites

- Python 3.8+
- C++ compiler (g++, clang++, or MSVC)
- NumPy
- Cython

### Quick Build

1. **Using the build script (recommended):**
   ```bash
   python build.py
   ```

2. **Using Make:**
   ```bash
   make install
   ```

3. **Manual build:**
   ```bash
   # Install dependencies
   pip install numpy cython
   
   # Build extensions
   python setup.py build_ext --inplace
   
   # Install package
   pip install -e .
   ```

### Build Options

- `make clean` - Clean build artifacts
- `make test` - Run tests
- `make format` - Format code
- `make lint` - Run linting
- `make wheel` - Build wheel distribution
- `make dist` - Build both wheel and source distribution

## Development Setup

1. **Clone the repository:**
   ```bash
   git clone https://github.com/markg/pycam.git
   cd pycam
   ```

2. **Install development dependencies:**
   ```bash
   make install-dev
   ```

3. **Run tests:**
   ```bash
   make test
   ```

## Usage

```python
import pycam

# Check if geometry module is available
if pycam._has_geometry:
    # Use high-performance geometry operations
    pass
else:
    print("Geometry module not available - please build extensions")

# Use G-code generation
from pycam.gcode import Program
```

## Testing

Run tests with:
```bash
make test
```

Or with coverage:
```bash
make test-cov
```

## Building for Distribution

To build wheel and source distributions:
```bash
make dist
```

This will create files in the `dist/` directory that can be uploaded to PyPI.

## Troubleshooting

### Common Build Issues

1. **Missing C++ compiler:**
   - Linux: `sudo apt-get install build-essential`
   - macOS: `xcode-select --install`
   - Windows: Install Visual Studio Build Tools

2. **Missing NumPy/Cython:**
   ```bash
   pip install numpy cython
   ```

3. **Permission errors:**
   - Use virtual environment: `python -m venv venv && source venv/bin/activate`

### Clean Build

If you encounter issues, try a clean build:
```bash
make clean-all
make install
```

## License

MIT License - see LICENSE file for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run tests: `make test`
5. Format code: `make format`
6. Submit a pull request

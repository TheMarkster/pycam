# Makefile for pycam project

# Variables
PYTHON := python3
PIP := pip
VENV := venv
VENV_BIN := $(VENV)/bin
PYTHON_VENV := $(VENV_BIN)/python
PIP_VENV := $(VENV_BIN)/pip

# Default target
.PHONY: all
all: install

# Create virtual environment
$(VENV):
	$(PYTHON) -m venv $(VENV)
	$(PIP_VENV) install --upgrade pip setuptools wheel

# Install dependencies
.PHONY: install-deps
install-deps: $(VENV)
	$(PIP_VENV) install numpy cython

# Build the C++/Cython extensions
.PHONY: build
build: install-deps
	$(PYTHON_VENV) setup.py build_ext --inplace

# Install the package in development mode
.PHONY: install
install: build
	$(PIP_VENV) install -e .

# Install development dependencies
.PHONY: install-dev
install-dev: install
	$(PIP_VENV) install -e ".[dev]"

# Clean build artifacts
.PHONY: clean
clean:
	rm -rf build/
	rm -rf dist/
	rm -rf *.egg-info/
	rm -rf src/pycam.egg-info/
	find src/ tests/ examples/ cython/ -name "*.pyc" -delete 2>/dev/null || true
	find src/ tests/ examples/ cython/ -name "__pycache__" -delete 2>/dev/null || true
	rm -f *.so
	rm -f src/pycam/core/*.so
	find cython/ -name "*.c" -delete 2>/dev/null || true
	find cython/ -name "*.cpp" -delete 2>/dev/null || true

# Run tests
.PHONY: test
test: install-dev
	$(PYTHON_VENV) -m pytest tests/

# Run tests with coverage
.PHONY: test-cov
test-cov: install-dev
	$(PYTHON_VENV) -m pytest tests/ --cov=pycam --cov-report=html --cov-report=term

# Format code
.PHONY: format
format: install-dev
	$(PYTHON_VENV) -m black src/ tests/
	$(PYTHON_VENV) -m isort src/ tests/

# Lint code
.PHONY: lint
lint: install-dev
	$(PYTHON_VENV) -m flake8 src/ tests/
	$(PYTHON_VENV) -m mypy src/

# Build wheel
.PHONY: wheel
wheel: clean
	$(PYTHON_VENV) -m build --wheel

# Build source distribution
.PHONY: sdist
sdist: clean
	$(PYTHON_VENV) -m build --sdist

# Build both wheel and source distribution
.PHONY: dist
dist: clean
	$(PYTHON_VENV) -m build

# Install from wheel
.PHONY: install-wheel
install-wheel: wheel
	$(PIP_VENV) install dist/*.whl

# Remove virtual environment
.PHONY: clean-venv
clean-venv:
	rm -rf $(VENV)

# Full clean (including venv)
.PHONY: clean-all
clean-all: clean clean-venv

# Debug: print build info
.PHONY: debug
debug:
	@echo "Python: $(PYTHON)"
	@echo "Virtual env: $(VENV)"
	@echo "Python (venv): $(PYTHON_VENV)"
	@echo "Pip (venv): $(PIP_VENV)"
	@echo "Current directory: $(shell pwd)"
	@echo "Python path: $(shell $(PYTHON_VENV) -c 'import sys; print(sys.path)' 2>/dev/null || echo 'venv not found')"

# Help
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  all           - Build and install the package (default)"
	@echo "  install-deps  - Install build dependencies"
	@echo "  build         - Build C++/Cython extensions"
	@echo "  install       - Install package in development mode"
	@echo "  install-dev   - Install with development dependencies"
	@echo "  clean         - Clean build artifacts"
	@echo "  clean-venv    - Remove virtual environment"
	@echo "  clean-all     - Clean everything including venv"
	@echo "  test          - Run tests"
	@echo "  test-cov      - Run tests with coverage"
	@echo "  format        - Format code with black and isort"
	@echo "  lint          - Run linting tools"
	@echo "  wheel         - Build wheel distribution"
	@echo "  sdist         - Build source distribution"
	@echo "  dist          - Build both wheel and source distribution"
	@echo "  debug         - Print build information"
	@echo "  help          - Show this help message"

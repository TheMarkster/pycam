#!/usr/bin/env python3
"""
Build script for PyCam - handles the complete build process
"""

import os
import sys
import subprocess
import shutil
from pathlib import Path

def run_command(cmd, cwd=None):
    """Run a shell command and return the result"""
    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Command failed with return code {result.returncode}")
        print(f"stdout: {result.stdout}")
        print(f"stderr: {result.stderr}")
        return False
    return True

def clean_build():
    """Clean previous build artifacts"""
    print("Cleaning build artifacts...")
    
    # Directories to remove
    clean_dirs = [
        "build",
        "dist",
        "src/pycam.egg-info",
        "*.egg-info",
    ]
    
    for pattern in clean_dirs:
        for path in Path(".").glob(pattern):
            if path.is_dir():
                print(f"Removing {path}")
                shutil.rmtree(path)
    
    # Files to remove - be more selective
    # Only clean .pyc and __pycache__ in our source directories
    for src_dir in ["src", "cython", "tests", "examples"]:
        src_path = Path(src_dir)
        if src_path.exists():
            for pattern in ["**/*.pyc", "**/__pycache__"]:
                for path in src_path.glob(pattern):
                    if path.is_file():
                        print(f"Removing {path}")
                        path.unlink()
                    elif path.is_dir():
                        print(f"Removing {path}")
                        shutil.rmtree(path)
    
    # Only remove .so files that are our build artifacts (not in venv)
    for so_file in Path(".").glob("*.so"):
        # Skip if the file is in a virtual environment
        if not any(part in str(so_file) for part in ['venv', 'env', 'site-packages', '.env', 'lib/python']):
            print(f"Removing {so_file}")
            so_file.unlink()
    
    # Clean .so files from our package directory
    pycam_core = Path("src/pycam/core")
    if pycam_core.exists():
        for so_file in pycam_core.glob("*.so"):
            print(f"Removing {so_file}")
            so_file.unlink()

def build_extensions():
    """Build the C++/Cython extensions"""
    print("Building C++/Cython extensions...")
    
    # Build extensions in place
    if not run_command([sys.executable, "setup.py", "build_ext", "--inplace"]):
        print("Failed to build extensions")
        return False
    
    return True

def install_package():
    """Install the package in development mode"""
    print("Installing package in development mode...")
    
    if not run_command([sys.executable, "-m", "pip", "install", "-e", "."]):
        print("Failed to install package")
        return False
    
    return True

def copy_libraries():
    """Copy built libraries to the current directory (original working location)"""
    print("Copying built libraries...")
    
    # Only look for .so files in the build directory
    so_files = []
    
    # Check build directory
    build_dir = Path("build")
    if build_dir.exists():
        so_files.extend(build_dir.glob("**/*.so"))
    
    if not so_files:
        print("No .so files found in build directory")
        # Check if there are any .so files in current directory already
        current_so = list(Path(".").glob("geometry*.so"))
        if current_so:
            print(f"Found existing .so files: {current_so}")
        return
    
    for so_file in so_files:
        # Copy to current directory (where it was working before)
        target_path = Path(".") / so_file.name
        print(f"Copying {so_file} to {target_path}")
        shutil.copy2(so_file, target_path)

def main():
    """Main build function"""
    print("Starting PyCam build process...")
    
    # Check if we have the required tools
    try:
        import numpy
        import Cython
    except ImportError as e:
        print(f"Missing required dependency: {e}")
        print("Please install numpy and Cython first:")
        print("pip install numpy cython")
        sys.exit(1)
    
    # Step 1: Clean previous builds
    clean_build()
    
    # Step 2: Build extensions
    if not build_extensions():
        print("Build failed!")
        sys.exit(1)
    
    # Step 3: Copy libraries
    copy_libraries()
    
    print("Build completed successfully!")
    print("\nYou can now import the geometry module directly:")
    print("import geometry")
    print("\nOr test with your existing code.")

if __name__ == "__main__":
    main()

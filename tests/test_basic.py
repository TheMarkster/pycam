"""
Test module for pycam
"""

import pytest


def test_import():
    """Test that pycam can be imported"""
    import pycam
    assert pycam.__version__ == "0.1.0"


def test_gcode_import():
    """Test that gcode module can be imported"""
    import pycam.gcode
    # Add more specific tests as needed


@pytest.mark.skipif(not hasattr(pytest, "pycam_geometry_available"), 
                   reason="Geometry module not available")
def test_geometry_import():
    """Test that geometry module can be imported (if available)"""
    try:
        import pycam.core.geometry
        # Add geometry-specific tests here
    except ImportError:
        pytest.skip("Geometry module not compiled")

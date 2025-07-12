from enum import Enum
from io import StringIO
from pycam.gcode.types import *

class Tool:
    ...

class Probe(Enum):
    ...

class Operation:
    ...

class Program:
    def __init__(self):
        self.segments = []
        self.out = StringIO()

    def preamble(self):
        """Set up the program with any necessary preamble commands."""
        raise NotImplementedError("Subclasses should implement this method to handle preamble commands.")
    
    def tool_change(self):
        """Set the current tool for the program."""
        raise NotImplementedError("Subclasses should implement this method to handle tool changes.")

    def probe(self, probe_action: Probe):
        """Probe the current tool position or perform any necessary probing actions."""
        raise NotImplementedError("Subclasses should implement this method to handle probing actions.")

    def end_program(self):
        """Finalize the program with any necessary end commands."""
        raise NotImplementedError("Subclasses should implement this method to handle end commands.")    
    
    def add_operation(self, operation: Operation):
        """Add an operation to the program."""
        raise NotImplementedError("Subclasses should implement this method to handle adding operations.")
    
    def write_comment(self, comment: str):
        """Write a comment to the program."""
        self.out.write(f"({comment})\n")
    
    @property
    def unit(self) -> Unit:
        """Get the current unit of measurement for the program."""
        return self._unit
    
    @unit.setter
    def unit(self, unit: Unit):
        """Set the current unit of measurement for the program."""
        self._unit = unit
    
    @property
    def out(self) -> StringIO:
        """Get the output stream for the program."""
        return self._out

    @out.setter
    def out(self, out: StringIO):
        """Set the output stream for the program."""
        self._out = out

    @property
    def tool(self):
        """Get the current tool for the program."""
        return self._tool
    
    @tool.setter
    def tool(self, tool):
        """Set the current tool for the program."""
        if self._tool is None or self._tool != tool:
            self.tool_change()
    
        self._tool = tool
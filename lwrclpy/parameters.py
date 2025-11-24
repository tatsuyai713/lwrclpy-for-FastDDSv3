from __future__ import annotations
from enum import IntEnum
from typing import Any


class ParameterType(IntEnum):
    NOT_SET = 0
    BOOL = 1
    INTEGER = 2
    DOUBLE = 3
    STRING = 4
    BYTE_ARRAY = 5
    BOOL_ARRAY = 6
    INTEGER_ARRAY = 7
    DOUBLE_ARRAY = 8
    STRING_ARRAY = 9


def _infer_type(value: Any) -> ParameterType:
    """Infer a ParameterType from a Python value (best-effort)."""
    if value is None:
        return ParameterType.NOT_SET
    if isinstance(value, bool):
        return ParameterType.BOOL
    if isinstance(value, int) and not isinstance(value, bool):
        return ParameterType.INTEGER
    if isinstance(value, float):
        return ParameterType.DOUBLE
    if isinstance(value, str):
        return ParameterType.STRING
    if isinstance(value, (bytes, bytearray, memoryview)):
        return ParameterType.BYTE_ARRAY
    if isinstance(value, (list, tuple)):
        if not value:
            return ParameterType.NOT_SET
        if all(isinstance(v, bool) for v in value):
            return ParameterType.BOOL_ARRAY
        if all(isinstance(v, int) and not isinstance(v, bool) for v in value):
            return ParameterType.INTEGER_ARRAY
        if all(isinstance(v, float) for v in value):
            return ParameterType.DOUBLE_ARRAY
        if all(isinstance(v, str) for v in value):
            return ParameterType.STRING_ARRAY
        if all(isinstance(v, (bytes, bytearray, memoryview)) for v in value):
            return ParameterType.BYTE_ARRAY
    return ParameterType.NOT_SET


class Parameter:
    """Lightweight rclpy.Parameter lookalike used for declare/get/set."""

    def __init__(self, name: str, value: Any = None):
        self._name = name
        if isinstance(value, ParameterType):
            self._type = value
            self._value = None
        else:
            self._value = value
            self._type = _infer_type(value)

    @property
    def name(self) -> str:
        return self._name

    @property
    def value(self) -> Any:
        return self._value

    @property
    def type(self) -> ParameterType:
        return self._type

    # rclpy uses type_ to avoid shadowing builtins; offer both.
    @property
    def type_(self) -> ParameterType:
        return self._type

    def __repr__(self) -> str:
        return f"Parameter(name={self._name!r}, type={self._type.name}, value={self._value!r})"


class SetParametersResult:
    """Simple result object mimicking rcl_interfaces.msg.SetParametersResult."""

    def __init__(self, successful: bool = True, reason: str = ""):
        self.successful = bool(successful)
        self.reason = reason

    def __bool__(self) -> bool:
        return self.successful


def coerce_parameter(obj: Any) -> Parameter:
    """Accept a Parameter or (name, value) pair and return a Parameter."""
    if isinstance(obj, Parameter):
        return obj
    if isinstance(obj, tuple) and len(obj) == 2:
        return Parameter(obj[0], obj[1])
    raise TypeError("set_parameters expects Parameter or (name, value) tuples")

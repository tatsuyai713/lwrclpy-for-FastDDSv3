from __future__ import annotations

from lwrclpy import (
    Node,
    Rate,
    create_node as _create_node_impl,
    create_rate as _create_rate_impl,
    Parameter,
    ParameterType,
    SetParametersResult,
    shutdown as _core_shutdown,  # Use lwrclpy.shutdown (with force_exit)
)
from lwrclpy.context import init as _core_init, ok as _core_ok

from .executors import (
    spin,
    spin_once,
    spin_some,
    spin_until_future_complete,
    SingleThreadedExecutor,
    MultiThreadedExecutor,
    Executor,
    ExternalShutdownException,
)
from .duration import Duration
from . import logging
from . import qos


class _InitContext:
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        shutdown()
        return False


def init(*, args=None, context=None):
    del context  # compatibility placeholder
    _core_init(args=args)
    return _InitContext()


def shutdown():
    _core_shutdown()


def ok() -> bool:
    return _core_ok()


def create_node(name: str, **kwargs) -> Node:
    return _create_node_impl(name, **kwargs)


def create_rate(hz: float) -> Rate:
    return _create_rate_impl(hz)


__all__ = [
    "Node",
    "Rate",
    "init",
    "shutdown",
    "ok",
    "spin",
    "spin_once",
    "spin_some",
    "spin_until_future_complete",
    "SingleThreadedExecutor",
    "MultiThreadedExecutor",
    "Executor",
    "ExternalShutdownException",
    "create_node",
    "create_rate",
    "Parameter",
    "ParameterType",
    "SetParametersResult",
    "Duration",
    "logging",
    "qos",
]

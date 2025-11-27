"""Utilities for exposing ROS 2-style Service classes with Request/Response attributes."""

from __future__ import annotations

import importlib
import importlib.util
from types import ModuleType


_DEFAULT_SERVICE_PACKAGES = (
    "action_msgs",
    "diagnostic_msgs",
    "example_interfaces",
    "gazebo_msgs",
    "lifecycle_msgs",
    "nav_msgs",
    "rcl_interfaces",
    "sensor_msgs",
    "std_msgs",
    "test_msgs",
    "tf2_msgs",
)

_PATCHED_MODULES: set[str] = set()


def install_service_aliases(extra_packages: list[str] | tuple[str, ...] | None = None) -> None:
    """Ensure every <Name>_Request/<Name>_Response pair has a rclpy-compatible wrapper class."""
    packages = list(_DEFAULT_SERVICE_PACKAGES)
    if extra_packages:
        packages.extend(extra_packages)
    for pkg in packages:
        _patch_package(pkg)


def _patch_package(package: str) -> None:
    if not package:
        return
    try:
        spec = importlib.util.find_spec(f"{package}.srv")
    except (ImportError, AttributeError):
        return
    if spec is None:
        return
    try:
        module = importlib.import_module(f"{package}.srv")
    except Exception:
        return
    _patch_module(module)


def _patch_module(module: ModuleType) -> None:
    module_name = getattr(module, "__name__", None)
    if module_name in _PATCHED_MODULES:
        return

    req_suffix = "_Request"
    resp_suffix = "_Response"
    created: list[str] = []

    for attr in dir(module):
        if attr.startswith("_") or not attr.endswith(req_suffix):
            continue
        base = attr[: -len(req_suffix)]
        if not base:
            continue
        resp_name = f"{base}{resp_suffix}"
        if not hasattr(module, resp_name):
            continue
        if hasattr(module, base):
            continue
        req_cls = getattr(module, attr)
        resp_cls = getattr(module, resp_name)
        wrapper = type(base, (), {"Request": req_cls, "Response": resp_cls})
        setattr(module, base, wrapper)
        created.append(base)

    if created:
        existing_all = list(getattr(module, "__all__", []))
        for name in created:
            if name not in existing_all:
                existing_all.append(name)
        if existing_all:
            module.__all__ = tuple(existing_all)

    if module_name:
        _PATCHED_MODULES.add(module_name)

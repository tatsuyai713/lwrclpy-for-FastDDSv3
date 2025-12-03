import os
from ._bootstrap_fastdds import ensure_fastdds
ensure_fastdds()

# Optional: patch SWIG-generated msg classes for attribute-style access.
# Disabled by default for stability; set LWRCLPY_PATCH_MSG_ATTRS=1 to enable.
if os.environ.get("LWRCLPY_PATCH_MSG_ATTRS") == "1":
    try:
        from .compat import patch_known_message_modules, patch_loaded_msg_modules
        patch_known_message_modules()
        patch_loaded_msg_modules()
    except Exception:
        pass

from .context import init, shutdown, ok, get_participant
from .executors import spin, spin_once, spin_some, SingleThreadedExecutor, MultiThreadedExecutor
from .node import Node, Rate, create_node, create_rate
from .parameters import Parameter, ParameterType, SetParametersResult
from .qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from .timer import create_timer
from . import timer
from .client import Client
from .service import Service
from .action import ActionServer, ActionClient, GoalResponse, CancelResponse
from .service_aliases import install_service_aliases as _install_service_aliases

_install_service_aliases()

__all__ = [
    "init", "shutdown", "ok", "spin", "spin_once", "spin_some",
    "SingleThreadedExecutor", "MultiThreadedExecutor", "Node", "Rate", "create_node", "create_rate",
    "Parameter", "ParameterType", "SetParametersResult",
    "QoSProfile", "ReliabilityPolicy", "DurabilityPolicy", "HistoryPolicy",
    "create_timer", "Client", "Service",
    "timer", "ActionServer", "ActionClient", "GoalResponse", "CancelResponse",
]

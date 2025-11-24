import threading
import os

try:
    import fastdds  # type: ignore
except Exception:
    fastdds = None

__all__ = ["init", "shutdown", "ok", "get_participant"]

_lock = threading.RLock()
_initialized = False
_shutdown = False
_participant = None
_domain_env = os.environ.get("LWRCL_DOMAIN_ID")
# ROS 2 互換：ROS_DOMAIN_ID をフォールバックに使う
if _domain_env is None:
    _domain_env = os.environ.get("ROS_DOMAIN_ID")
try:
    _domain = int(_domain_env) if _domain_env is not None else 0
except ValueError:
    _domain = 0


def init(args=None):
    global _initialized, _shutdown, _participant
    with _lock:
        if _initialized:
            return
        if fastdds is None:
            raise RuntimeError(
                "fastdds Python bindings not found. Build Fast-DDS-python and source its setup.bash.")
        factory = fastdds.DomainParticipantFactory.get_instance()
        pq = fastdds.DomainParticipantQos()
        factory.get_default_participant_qos(pq)
        _participant = factory.create_participant(_domain, pq)
        if _participant is None:
            raise RuntimeError("Failed to create DomainParticipant")
        _initialized = True
        _shutdown = False


def shutdown():
    global _initialized, _shutdown, _participant
    with _lock:
        if not _initialized:
            return
        factory = fastdds.DomainParticipantFactory.get_instance()
        if _participant is not None:
            factory.delete_participant(_participant)
            _participant = None
        _shutdown = True
        _initialized = False


def ok() -> bool:
    return _initialized and not _shutdown


def get_participant():
    if not _initialized:
        raise RuntimeError("lwrclpy.init() must be called first")
    return _participant

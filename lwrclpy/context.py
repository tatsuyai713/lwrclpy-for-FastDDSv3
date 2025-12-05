import threading
import os
import atexit

try:
    import fastdds  # type: ignore
except Exception:
    fastdds = None

__all__ = ["init", "shutdown", "ok", "is_shutdown", "get_participant"]

_lock = threading.RLock()
_initialized = False
_shutdown = False
_participant = None
_tracked_entities = []  # Track all entities for proper cleanup order
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
        
        # Don't register atexit shutdown - let Fast DDS handle cleanup automatically
        # atexit.register(shutdown)


def shutdown(*, force_exit: bool = False):
    """Shutdown the context.
    
    Args:
        force_exit: If True, use os._exit(0) to bypass Python cleanup and avoid
                   Fast DDS v3 "double free" errors. Default is False for compatibility.
    """
    global _initialized, _shutdown, _participant, _tracked_entities
    with _lock:
        if not _initialized or _shutdown:
            return
        
        # Mark as shutting down first to prevent new operations
        _shutdown = True
        
        # Don't explicitly destroy entities - let Fast DDS handle all cleanup
        # Attempting to destroy entities during shutdown causes "double free" errors
        # Fast DDS will automatically clean up all entities when the process terminates
        _tracked_entities.clear()
        
        # Don't delete participant - let Fast DDS clean it up on process exit
        _participant = None
        _initialized = False
    
    # If force_exit is requested, use os._exit to bypass Python cleanup
    # This avoids "double free or corruption (fasttop)" errors in Fast DDS v3
    if force_exit:
        os._exit(0)


def ok() -> bool:
    return _initialized and not _shutdown


def is_shutdown() -> bool:
    """Return True if context is shutting down or has shutdown."""
    return _shutdown


def get_participant():
    if not _initialized:
        raise RuntimeError("lwrclpy.init() must be called first")
    return _participant


def track_entity(entity):
    """Track an entity for proper cleanup order during shutdown."""
    with _lock:
        if entity not in _tracked_entities:
            _tracked_entities.append(entity)


def untrack_entity(entity):
    """Remove an entity from tracking (when manually destroyed)."""
    with _lock:
        try:
            _tracked_entities.remove(entity)
        except ValueError:
            pass

import threading
import time
from typing import Optional

from .publisher import Publisher
from .subscription import Subscription
from .qos import QoSProfile
from .typesupport import RegisteredType
from .utils import resolve_service_type, SERVICE_REQUEST_PREFIX, SERVICE_RESPONSE_PREFIX
from .context import get_participant, ok, track_entity, untrack_entity
from .utils import get_or_create_topic
from .future import Future


class Client:
    """Best-effort rclpy-like Client (single outstanding request)."""

    def __init__(
        self,
        service_type,
        service_name: str,
        qos_profile: QoSProfile,
        topic_prefix: str = "",
        *,
        enqueue_cb=None,
    ):
        self._participant = get_participant()
        self._service_name = service_name
        self._prefix = topic_prefix
        self._enqueue_cb = enqueue_cb or (lambda cb, msg: cb(msg))

        req_cls, res_cls, _req_pubsub, _res_pubsub = resolve_service_type(service_type)
        self._request_cls = req_cls
        self._response_cls = res_cls

        # Register types
        self._req_type_name = RegisteredType(req_cls).register()
        self._res_type_name = RegisteredType(res_cls).register()

        req_topic, res_topic = _service_topics(service_name, topic_prefix)

        self._publisher = Publisher(
            self._participant,
            self._make_topic(req_topic, self._req_type_name),
            qos_profile,
            msg_ctor=self._request_cls,
        )
        self._lock = threading.Lock()
        self._pending_future: Future | None = None

        def _on_response(msg):
            future = None
            with self._lock:
                future = self._pending_future
                self._pending_future = None
            if future:
                future.set_result(msg)

        self._subscription = Subscription(
            self._participant,
            self._make_topic(res_topic, self._res_type_name),
            qos_profile,
            _on_response,
            self._response_cls,
            enqueue_cb=lambda cb, msg: cb(msg),  # Execute callback directly, don't enqueue
        )
        
        # Track for proper cleanup
        track_entity(self)

    def _make_topic(self, name: str, type_name: str):
        # Publisher/Subscription expect Topic objects; reuse participant
        topic_obj, _ = get_or_create_topic(self._participant, name, type_name)
        return topic_obj

    def call(self, request, timeout: Optional[float] = None):
        """Send request and block for one response.
        
        Note: This implementation busy-waits because lwrclpy callbacks are enqueued
        but not automatically processed. For proper behavior, use with an executor
        or call from within a spin() context.
        """
        future = self.call_async(request)
        start_time = time.monotonic() if timeout is not None else None
        
        # Busy wait for response (callbacks are enqueued via enqueue_cb)
        while ok() and not future.done():
            time.sleep(0.001)  # Small sleep to avoid busy spin
            
            # Check timeout
            if timeout is not None:
                elapsed = time.monotonic() - start_time
                if elapsed >= timeout:
                    # Timeout - clear pending future to allow next request
                    with self._lock:
                        if self._pending_future is future:
                            self._pending_future = None
                    future.cancel()
                    return None
        
        # Get result if available
        if future.done():
            try:
                return future.result(0.0)
            except Exception:
                return None
        return None

    def call_async(self, request) -> Future:
        """Send request asynchronously, returning a Future resolved with the response."""
        future = Future()
        with self._lock:
            if self._pending_future is not None:
                raise RuntimeError("Only one pending service request is supported at a time")
            self._pending_future = future
        self._publisher.publish(request)
        return future

    def send_request(self, request):
        """Compatibility alias used by some examples."""
        self._publisher.publish(request)
        return True

    def wait_for_service(self, timeout_sec: Optional[float] = None) -> bool:
        # DDS discovery is out of scope; always True for now.
        if timeout_sec is None:
            return True
        # simulate wait
        import time
        time.sleep(0 if timeout_sec < 0 else min(timeout_sec, 0.01))
        return True

    def destroy(self):
        """Clean up client resources in the correct order."""
        # Untrack from global cleanup
        untrack_entity(self)
        
        # Clear pending future first
        self._pending_future = None
        
        # Destroy subscription first (stop receiving)
        if hasattr(self, '_subscription') and self._subscription:
            try:
                self._subscription.destroy()
            except Exception:
                pass
            self._subscription = None
        
        # Then destroy publisher
        if hasattr(self, '_publisher') and self._publisher:
            try:
                self._publisher.destroy()
            except Exception:
                pass
            self._publisher = None


def _service_topics(name: str, prefix: str = ""):
    cleaned = name.lstrip("/")
    req = f"{SERVICE_REQUEST_PREFIX}{cleaned}Request"
    res = f"{SERVICE_RESPONSE_PREFIX}{cleaned}Reply"
    if prefix:
        if not req.startswith(prefix):
            req = prefix + req
        if not res.startswith(prefix):
            res = prefix + res
    return req, res

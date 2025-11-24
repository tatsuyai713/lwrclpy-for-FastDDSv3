import threading
from typing import Optional

from .publisher import Publisher
from .subscription import Subscription
from .qos import QoSProfile
from .typesupport import RegisteredType
from .utils import resolve_service_type
from .context import get_participant
from .utils import get_or_create_topic


class Client:
    """Best-effort rclpy-like Client (single outstanding request)."""

    def __init__(self, service_type, service_name: str, qos_profile: QoSProfile, topic_prefix: str = ""):
        self._participant = get_participant()
        self._service_name = service_name
        self._prefix = topic_prefix

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
        self._response = None
        self._cond = threading.Condition()

        def _on_response(msg):
            with self._cond:
                self._response = msg
                self._cond.notify_all()

        self._subscription = Subscription(
            self._participant,
            self._make_topic(res_topic, self._res_type_name),
            qos_profile,
            _on_response,
            self._response_cls,
            enqueue_cb=lambda cb, msg: cb(msg),
        )

    def _make_topic(self, name: str, type_name: str):
        # Publisher/Subscription expect Topic objects; reuse participant
        topic_obj, _ = get_or_create_topic(self._participant, name, type_name)
        return topic_obj

    def call(self, request, timeout: Optional[float] = None):
        """Send request and block for one response. Single outstanding request supported."""
        with self._cond:
            self._response = None
        self._publisher.publish(request)
        with self._cond:
            if timeout is None:
                while self._response is None:
                    self._cond.wait()
            else:
                self._cond.wait(timeout=timeout)
            return self._response

    def send_request(self, request):
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
        self._subscription.destroy()
        self._publisher.destroy()


def _service_topics(name: str, prefix: str = ""):
    cleaned = name.lstrip("/")
    req = f"rq/{cleaned}"
    res = f"rr/{cleaned}"
    if prefix and not req.startswith(prefix):
        req = prefix + req
        res = prefix + res
    return req, res

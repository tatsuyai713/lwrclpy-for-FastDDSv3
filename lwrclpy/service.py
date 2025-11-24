from .publisher import Publisher
from .subscription import Subscription
from .qos import QoSProfile
from .typesupport import RegisteredType
from .utils import resolve_service_type
from .utils import get_or_create_topic
from .context import get_participant


class Service:
    """Best-effort rclpy-like Service (one callback per request)."""

    def __init__(self, service_type, service_name: str, callback, qos_profile: QoSProfile, topic_prefix: str = ""):
        self._participant = get_participant()
        self._service_name = service_name
        self._callback = callback
        self._prefix = topic_prefix

        req_cls, res_cls, _req_pubsub, _res_pubsub = resolve_service_type(service_type)
        self._request_cls = req_cls
        self._response_cls = res_cls

        # Register types
        self._req_type_name = RegisteredType(req_cls).register()
        self._res_type_name = RegisteredType(res_cls).register()

        req_topic, res_topic = _service_topics(service_name, topic_prefix)

        self._response_pub = Publisher(
            self._participant,
            self._make_topic(res_topic, self._res_type_name),
            qos_profile,
            msg_ctor=self._response_cls,
        )

        def _on_request(msg):
            response = self._response_cls()
            try:
                ret = self._callback(msg, response)
                if ret is not None:
                    response = ret
            except Exception:
                # swallow user errors
                return
            self._response_pub.publish(response)

        self._request_sub = Subscription(
            self._participant,
            self._make_topic(req_topic, self._req_type_name),
            qos_profile,
            _on_request,
            self._request_cls,
            enqueue_cb=lambda cb, msg: cb(msg),
        )

    def _make_topic(self, name: str, type_name: str):
        topic_obj, _ = get_or_create_topic(self._participant, name, type_name)
        return topic_obj

    def destroy(self):
        self._request_sub.destroy()
        self._response_pub.destroy()


def _service_topics(name: str, prefix: str = ""):
    cleaned = name.lstrip("/")
    req = f"rq/{cleaned}"
    res = f"rr/{cleaned}"
    if prefix and not req.startswith(prefix):
        req = prefix + req
        res = prefix + res
    return req, res

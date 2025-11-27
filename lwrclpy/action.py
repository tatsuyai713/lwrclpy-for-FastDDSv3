import asyncio
import threading
import time
import uuid
from enum import Enum
from typing import Callable, Optional

try:
    from action_msgs.msg import GoalStatusArray, GoalStatus
except Exception:  # pragma: no cover - optional dependency for ROS2 compatibility
    GoalStatusArray = None
    GoalStatus = None

from .context import get_participant
from .future import Future
from .qos import QoSProfile
from .typesupport import RegisteredType
from .utils import (
    resolve_generated_type,
    resolve_action_type,
    resolve_name,
    get_or_create_topic,
    ACTION_PREFIX,
)
from .publisher import Publisher
from .subscription import Subscription


class GoalResponse(Enum):
    REJECT = 0
    ACCEPT = 1


class CancelResponse(Enum):
    REJECT = 0
    ACCEPT = 1


_STATUS_UNKNOWN = 0
_STATUS_ACCEPTED = 1
_STATUS_EXECUTING = 2
_STATUS_CANCELING = 3
_STATUS_SUCCEEDED = 4
_STATUS_CANCELED = 5
_STATUS_ABORTED = 6


def _set_uuid(field, uid: uuid.UUID):
    data = list(uid.bytes)
    if hasattr(field, "uuid"):
        try:
            setattr(field, "uuid", data)
            return
        except Exception:
            pass
    try:
        for i, b in enumerate(data):
            field[i] = b
    except Exception:
        pass


def _uuid_bytes(field) -> bytes:
    if field is None:
        return b""
    if hasattr(field, "uuid"):
        data = getattr(field, "uuid")
        return bytes(int(x) & 0xFF for x in data)
    try:
        return bytes(int(x) & 0xFF for x in field)
    except Exception:
        return b""


def _copy_goal_id(goal_id_field):
    if goal_id_field is None:
        return None
    try:
        new_field = goal_id_field.__class__()
    except Exception:
        new_field = type("GoalID", (), {})()
    if hasattr(goal_id_field, "uuid"):
        try:
            setattr(new_field, "uuid", list(getattr(goal_id_field, "uuid")))
            return new_field
        except Exception:
            pass
    try:
        for i, val in enumerate(goal_id_field):
            new_field[i] = val
    except Exception:
        pass
    return new_field


def _zero_stamp(msg):
    for attr in ("stamp", "goal_info", "goal_id"):
        target = getattr(msg, attr, None)
        if target is None:
            continue
        for sec_name, nsec_name in (("sec", "nanosec"), ("seconds", "nanoseconds")):
            if hasattr(target, sec_name) and hasattr(target, nsec_name):
                try:
                    setattr(target, sec_name, 0)
                    setattr(target, nsec_name, 0)
                except Exception:
                    pass
                return


def _register_type(cls):
    resolved_cls = resolve_generated_type(cls)[1]
    return RegisteredType(resolved_cls).register()


class _ServerGoalHandle:
    def __init__(self, server, goal_id, request):
        self.request = request
        self._server = server
        self._goal_id = goal_id
        self._active = True
        self._cancel_requested = False
        self._executing = False

    @property
    def is_active(self) -> bool:
        return self._active

    @property
    def is_cancel_requested(self) -> bool:
        return self._cancel_requested

    def _set_cancel_requested(self):
        self._cancel_requested = True
        try:
            self._server._set_status(self._goal_id, _STATUS_CANCELING)
        except Exception:
            pass

    def execute(self):
        if self._executing:
            return
        self._executing = True
        try:
            self._server._set_status(self._goal_id, _STATUS_EXECUTING)
        except Exception:
            pass
        self._server._start_execute(self)

    def publish_feedback(self, feedback_msg=None):
        self._server._publish_feedback(self._goal_id, feedback_msg)

    def succeed(self, result=None):
        if not self._active:
            return
        self._active = False
        self._server._complete_goal(self._goal_id, result, _STATUS_SUCCEEDED)

    def abort(self, result=None):
        if not self._active:
            return
        self._active = False
        self._server._complete_goal(self._goal_id, result, _STATUS_ABORTED)

    def canceled(self, result=None):
        if not self._active:
            return
        self._active = False
        self._server._complete_goal(self._goal_id, result, _STATUS_CANCELED)


class ActionServer:
    """ActionServer with ROS2-like topic behavior."""

    def __init__(
        self,
        node,
        action_type,
        action_name: str,
        execute_callback: Callable,
        *,
        goal_callback: Optional[Callable] = None,
        cancel_callback: Optional[Callable] = None,
        handle_accepted_callback: Optional[Callable] = None,
        callback_group=None,
    ):
        del callback_group
        self._participant = get_participant()
        self._node = node
        self._execute_callback = execute_callback
        self._goal_callback = goal_callback
        self._cancel_callback = cancel_callback
        self._handle_accepted_callback = handle_accepted_callback
        self._lock = threading.Lock()
        self._goal_handles: dict[bytes, _ServerGoalHandle] = {}
        # map goal_id bytes -> (status_code, result_obj, goal_id_copy)
        self._results: dict[bytes, tuple[int, object | None, object | None]] = {}
        self._pending_result_requests: set[bytes] = set()
        self._status_entries: dict[bytes, tuple[object, int]] = {}

        types = resolve_action_type(action_type)
        self._goal_cls = types["goal"]
        self._result_cls = types["result"]
        self._feedback_cls = types["feedback"]
        self._feedback_msg_cls = types["feedback_msg"]
        self._send_goal_req_cls = types["send_goal_req"]
        self._send_goal_res_cls = types["send_goal_res"]
        self._get_result_req_cls = types["get_result_req"]
        self._get_result_res_cls = types["get_result_res"]
        self._cancel_req_cls = types["cancel_req"]
        self._cancel_res_cls = types["cancel_res"]

        # ROS2-like name resolution (absolute, relative, private) + DDS action prefix (ra/)
        resolved = resolve_name(action_name, node.get_namespace(), node.get_name()).lstrip("/")
        base_name = resolved if resolved.startswith(ACTION_PREFIX) else f"{ACTION_PREFIX}{resolved}"
        self._send_goal_topic = f"{base_name}/_action/send_goal"
        self._get_result_topic = f"{base_name}/_action/get_result"
        self._feedback_topic = f"{base_name}/_action/feedback"
        self._cancel_topic = f"{base_name}/_action/cancel_goal"
        self._status_topic = f"{base_name}/_action/status" if GoalStatusArray is not None else None

        qos = QoSProfile()
        self._send_goal_res_pub = Publisher(
            self._participant,
            self._create_topic(f"{self._send_goal_topic}/_response", _register_type(self._send_goal_res_cls)),
            qos,
            msg_ctor=resolve_generated_type(self._send_goal_res_cls)[1],
        )
        self._get_result_res_pub = Publisher(
            self._participant,
            self._create_topic(f"{self._get_result_topic}/_response", _register_type(self._get_result_res_cls)),
            qos,
            msg_ctor=resolve_generated_type(self._get_result_res_cls)[1],
        )
        self._feedback_pub = Publisher(
            self._participant,
            self._create_topic(self._feedback_topic, _register_type(self._feedback_msg_cls)),
            qos,
            msg_ctor=resolve_generated_type(self._feedback_msg_cls)[1],
        )
        self._status_pub = None
        self._status_msg_ctor = None
        self._goal_status_ctor = None
        if self._status_topic and GoalStatusArray is not None and GoalStatus is not None:
            try:
                self._status_msg_ctor = resolve_generated_type(GoalStatusArray)[1]
                self._goal_status_ctor = resolve_generated_type(GoalStatus)[1]
                self._status_pub = Publisher(
                    self._participant,
                    self._create_topic(self._status_topic, _register_type(GoalStatusArray)),
                    qos,
                    msg_ctor=self._status_msg_ctor,
                )
            except Exception:
                self._status_pub = None
                self._status_msg_ctor = None
                self._goal_status_ctor = None
        self._cancel_res_pub = Publisher(
            self._participant,
            self._create_topic(f"{self._cancel_topic}/_response", _register_type(self._cancel_res_cls)),
            qos,
            msg_ctor=resolve_generated_type(self._cancel_res_cls)[1],
        )

        send_goal_req_ctor = resolve_generated_type(self._send_goal_req_cls)[1]
        get_result_req_ctor = resolve_generated_type(self._get_result_req_cls)[1]
        cancel_req_ctor = resolve_generated_type(self._cancel_req_cls)[1]

        self._send_goal_sub = Subscription(
            self._participant,
            self._create_topic(f"{self._send_goal_topic}/_request", _register_type(self._send_goal_req_cls)),
            qos,
            self._on_send_goal,
            send_goal_req_ctor,
            enqueue_cb=self._node._enqueue_callback,
        )
        self._get_result_sub = Subscription(
            self._participant,
            self._create_topic(f"{self._get_result_topic}/_request", _register_type(self._get_result_req_cls)),
            qos,
            self._on_get_result,
            get_result_req_ctor,
            enqueue_cb=self._node._enqueue_callback,
        )
        self._cancel_sub = Subscription(
            self._participant,
            self._create_topic(f"{self._cancel_topic}/_request", _register_type(self._cancel_req_cls)),
            qos,
            self._on_cancel,
            cancel_req_ctor,
            enqueue_cb=self._node._enqueue_callback,
        )

    def _create_topic(self, name: str, type_name: str):
        topic_obj, _ = get_or_create_topic(self._participant, name, type_name)
        return topic_obj

    def _on_send_goal(self, request_msg):
        goal_id = getattr(request_msg, "goal_id", None)
        key = _uuid_bytes(goal_id)
        accepted = True
        if self._goal_callback:
            resp = self._goal_callback(request_msg.goal)
            accepted = resp == GoalResponse.ACCEPT

        response = self._send_goal_res_cls()
        try:
            setattr(response, "accepted", bool(accepted))
        except Exception:
            pass
        _zero_stamp(response)

        if accepted:
            handle = _ServerGoalHandle(self, _copy_goal_id(goal_id), request_msg.goal)
            with self._lock:
                self._goal_handles[key] = handle
            self._set_status(goal_id, _STATUS_ACCEPTED)
            if self._handle_accepted_callback:
                try:
                    self._handle_accepted_callback(handle)
                except Exception:
                    pass
            else:
                handle.execute()
        self._send_goal_res_pub.publish(response)

    def _on_get_result(self, request_msg):
        goal_id = getattr(request_msg, "goal_id", None)
        key = _uuid_bytes(goal_id)
        publish_now = False
        with self._lock:
            if key in self._results:
                publish_now = True
            else:
                self._pending_result_requests.add(key)
        if publish_now:
            self._publish_result_for_goal(key)

    def _on_cancel(self, request_msg):
        response = self._cancel_res_cls()
        goal_info = getattr(request_msg, "goal_info", None)
        goal_id = getattr(goal_info, "goal_id", goal_info)
        key = _uuid_bytes(goal_id)
        handle = self._goal_handles.get(key)
        allowed = False
        if handle and self._cancel_callback:
            try:
                allowed = self._cancel_callback(handle) == CancelResponse.ACCEPT
            except Exception:
                allowed = False
        elif handle:
            allowed = True
        if allowed and handle:
            handle._set_cancel_requested()
        try:
            if hasattr(response, "return_code"):
                setattr(response, "return_code", CancelResponse.ACCEPT.value if allowed else CancelResponse.REJECT.value)
        except Exception:
            pass
        self._cancel_res_pub.publish(response)

    def _start_execute(self, goal_handle: _ServerGoalHandle):
        def _run():
            try:
                result = self._execute_callback(goal_handle)
                if asyncio.iscoroutine(result):
                    result = asyncio.run(result)
                if result is not None and goal_handle.is_active:
                    goal_handle.succeed(result)
            except Exception:
                goal_handle.abort()

        threading.Thread(target=_run, daemon=True).start()

    def _publish_feedback(self, goal_id, feedback_msg=None):
        msg = self._feedback_msg_cls()
        try:
            if hasattr(msg, "goal_id"):
                setattr(msg, "goal_id", _copy_goal_id(goal_id))
            if feedback_msg is not None:
                setattr(msg, "feedback", feedback_msg)
        except Exception:
            pass
        self._feedback_pub.publish(msg)

    def _complete_goal(self, goal_id, result, status_code):
        key = _uuid_bytes(goal_id)
        if isinstance(result, self._result_cls):
            result_obj = result
        else:
            result_obj = self._result_cls()
            if result is not None:
                for attr in dir(result):
                    if attr.startswith("_"):
                        continue
                    try:
                        setattr(result_obj, attr, getattr(result, attr))
                    except Exception:
                        pass
        goal_id_copy = _copy_goal_id(goal_id)
        with self._lock:
            self._results[key] = (status_code, result_obj, goal_id_copy)
            self._goal_handles.pop(key, None)
            publish_now = key in self._pending_result_requests
        self._set_status(goal_id, status_code)
        if publish_now:
            self._publish_result_for_goal(key)

    def _publish_result_for_goal(self, key: bytes):
        entry = None
        with self._lock:
            entry = self._results.get(key)
            if entry is None:
                return
            status, result_obj, goal_id_copy = entry
            self._results.pop(key, None)
            self._pending_result_requests.discard(key)
        res_msg = self._get_result_res_cls()
        try:
            if hasattr(res_msg, "status"):
                setattr(res_msg, "status", status)
            if hasattr(res_msg, "result") and result_obj is not None:
                setattr(res_msg, "result", result_obj)
            if hasattr(res_msg, "goal_id"):
                setattr(res_msg, "goal_id", _copy_goal_id(goal_id_copy))
        except Exception:
            pass
        self._get_result_res_pub.publish(res_msg)
        if status in (_STATUS_SUCCEEDED, _STATUS_ABORTED, _STATUS_CANCELED):
            self._clear_status_entry(key)

    def _set_status(self, goal_id, status_code: int):
        if self._status_pub is None or goal_id is None:
            return
        key = _uuid_bytes(goal_id)
        goal_id_copy = _copy_goal_id(goal_id)
        with self._lock:
            existing = self._status_entries.get(key)
            if existing is not None:
                goal_id_copy = existing[0]
            self._status_entries[key] = (goal_id_copy, status_code)
            snapshot = list(self._status_entries.values())
        self._publish_status_snapshot(snapshot)

    def _clear_status_entry(self, key: bytes):
        if self._status_pub is None:
            return
        with self._lock:
            removed = self._status_entries.pop(key, None)
            snapshot = list(self._status_entries.values()) if removed is not None else None
        if removed is not None and snapshot is not None:
            self._publish_status_snapshot(snapshot)

    def _publish_status_snapshot(self, entries):
        if self._status_pub is None or self._status_msg_ctor is None or self._goal_status_ctor is None:
            return
        try:
            msg = self._status_msg_ctor()
        except Exception:
            return
        _zero_stamp(msg)
        status_list = []
        for goal_id_copy, status_code in entries:
            try:
                status_msg = self._goal_status_ctor()
            except Exception:
                continue
            try:
                goal_info = getattr(status_msg, "goal_info", None)
                if goal_info is not None and hasattr(goal_info, "goal_id"):
                    setattr(goal_info, "goal_id", _copy_goal_id(goal_id_copy))
                elif hasattr(status_msg, "goal_id"):
                    setattr(status_msg, "goal_id", _copy_goal_id(goal_id_copy))
                if hasattr(status_msg, "status"):
                    setattr(status_msg, "status", status_code)
            except Exception:
                pass
            status_list.append(status_msg)
        try:
            setattr(msg, "status_list", status_list)
        except Exception:
            pass
        self._status_pub.publish(msg)

    def destroy(self):
        for pub in (
            self._send_goal_res_pub,
            self._get_result_res_pub,
            self._feedback_pub,
            self._status_pub,
            self._cancel_res_pub,
        ):
            if not pub:
                continue
            try:
                pub.destroy()
            except Exception:
                pass
        for sub in (self._send_goal_sub, self._get_result_sub, self._cancel_sub):
            if not sub:
                continue
            try:
                sub.destroy()
            except Exception:
                pass


class ClientGoalHandle:
    def __init__(self, client, goal_id):
        self._client = client
        self._goal_id = goal_id
        self._accepted = False
        self._done = False

    @property
    def accepted(self) -> bool:
        return self._accepted

    def _set_accepted(self, accepted: bool):
        self._accepted = accepted

    def get_result_async(self) -> Future:
        return self._client._request_result(self._goal_id)

    def cancel_goal_async(self) -> Future:
        return self._client._request_cancel(self._goal_id)


class ActionClient:
    """ActionClient mirroring rclpy API."""

    def __init__(self, node, action_type, action_name: str, *, callback_group=None):
        del callback_group
        self._participant = get_participant()
        self._node = node
        self._lock = threading.Lock()
        self._feedback_callbacks: dict[bytes, Optional[Callable]] = {}
        self._send_goal_futures: dict[bytes, Future] = {}
        self._result_futures: dict[bytes, Future] = {}
        self._cancel_futures: dict[bytes, Future] = {}
        self._goal_handles: dict[bytes, ClientGoalHandle] = {}

        types = resolve_action_type(action_type)
        self._goal_cls = types["goal"]
        self._result_cls = types["result"]
        self._feedback_cls = types["feedback"]
        self._feedback_msg_cls = types["feedback_msg"]
        self._send_goal_req_cls = types["send_goal_req"]
        self._send_goal_res_cls = types["send_goal_res"]
        self._get_result_req_cls = types["get_result_req"]
        self._get_result_res_cls = types["get_result_res"]
        self._cancel_req_cls = types["cancel_req"]
        self._cancel_res_cls = types["cancel_res"]

        resolved = resolve_name(action_name, node.get_namespace(), node.get_name()).lstrip("/")
        base_name = resolved if resolved.startswith(ACTION_PREFIX) else f"{ACTION_PREFIX}{resolved}"
        self._send_goal_topic = f"{base_name}/_action/send_goal"
        self._get_result_topic = f"{base_name}/_action/get_result"
        self._feedback_topic = f"{base_name}/_action/feedback"
        self._cancel_topic = f"{base_name}/_action/cancel_goal"

        qos = QoSProfile()
        self._send_goal_pub = Publisher(
            self._participant,
            self._create_topic(f"{self._send_goal_topic}/_request", _register_type(self._send_goal_req_cls)),
            qos,
            msg_ctor=resolve_generated_type(self._send_goal_req_cls)[1],
        )
        self._get_result_pub = Publisher(
            self._participant,
            self._create_topic(f"{self._get_result_topic}/_request", _register_type(self._get_result_req_cls)),
            qos,
            msg_ctor=resolve_generated_type(self._get_result_req_cls)[1],
        )
        self._cancel_pub = Publisher(
            self._participant,
            self._create_topic(f"{self._cancel_topic}/_request", _register_type(self._cancel_req_cls)),
            qos,
            msg_ctor=resolve_generated_type(self._cancel_req_cls)[1],
        )

        send_goal_res_ctor = resolve_generated_type(self._send_goal_res_cls)[1]
        get_result_res_ctor = resolve_generated_type(self._get_result_res_cls)[1]
        feedback_ctor = resolve_generated_type(self._feedback_msg_cls)[1]
        cancel_res_ctor = resolve_generated_type(self._cancel_res_cls)[1]

        self._send_goal_res_sub = Subscription(
            self._participant,
            self._create_topic(f"{self._send_goal_topic}/_response", _register_type(self._send_goal_res_cls)),
            qos,
            self._on_send_goal_response,
            send_goal_res_ctor,
            enqueue_cb=self._node._enqueue_callback,
        )
        self._result_sub = Subscription(
            self._participant,
            self._create_topic(f"{self._get_result_topic}/_response", _register_type(self._get_result_res_cls)),
            qos,
            self._on_result_response,
            get_result_res_ctor,
            enqueue_cb=self._node._enqueue_callback,
        )
        self._feedback_sub = Subscription(
            self._participant,
            self._create_topic(self._feedback_topic, _register_type(self._feedback_msg_cls)),
            qos,
            self._on_feedback,
            feedback_ctor,
            enqueue_cb=self._node._enqueue_callback,
        )
        self._cancel_res_sub = Subscription(
            self._participant,
            self._create_topic(f"{self._cancel_topic}/_response", _register_type(self._cancel_res_cls)),
            qos,
            self._on_cancel_response,
            cancel_res_ctor,
            enqueue_cb=self._node._enqueue_callback,
        )

    def _create_topic(self, name: str, type_name: str):
        topic_obj, _ = get_or_create_topic(self._participant, name, type_name)
        return topic_obj

    def wait_for_server(self, timeout_sec: Optional[float] = None) -> bool:
        if timeout_sec is not None and timeout_sec > 0:
            time.sleep(min(timeout_sec, 0.01))
        return True

    def send_goal_async(self, goal_msg, feedback_callback: Optional[Callable] = None) -> Future:
        request = self._send_goal_req_cls()
        goal_id = getattr(request, "goal_id", None)
        if goal_id is None:
            try:
                goal_id = type("GoalID", (), {})()
                setattr(request, "goal_id", goal_id)
            except Exception:
                pass
        uid = uuid.uuid4()
        if goal_id is not None:
            _set_uuid(goal_id, uid)
        try:
            setattr(request, "goal", goal_msg)
        except Exception:
            pass
        key = _uuid_bytes(goal_id)
        future = Future()
        handle = ClientGoalHandle(self, _copy_goal_id(goal_id))
        handle._set_accepted(False)
        with self._lock:
            self._send_goal_futures[key] = future
            self._goal_handles[key] = handle
        self._feedback_callbacks[key] = feedback_callback
        self._send_goal_pub.publish(request)
        return future

    def _on_send_goal_response(self, msg):
        goal_id = getattr(msg, "goal_id", None)
        key = _uuid_bytes(goal_id)
        accepted = bool(getattr(msg, "accepted", True))
        future = None
        handle = None
        with self._lock:
            future = self._send_goal_futures.pop(key, None)
            handle = self._goal_handles.get(key)
            if not accepted:
                self._goal_handles.pop(key, None)
                self._feedback_callbacks.pop(key, None)
        if handle:
            handle._set_accepted(accepted)
        if future:
            future.set_result(handle)

    def _request_result(self, goal_id) -> Future:
        key = _uuid_bytes(goal_id)
        future = Future()
        with self._lock:
            self._result_futures[key] = future
        request = self._get_result_req_cls()
        try:
            setattr(request, "goal_id", _copy_goal_id(goal_id))
        except Exception:
            pass
        self._get_result_pub.publish(request)
        return future

    def _on_result_response(self, msg):
        goal_id = getattr(msg, "goal_id", None)
        key = _uuid_bytes(goal_id)
        future = None
        with self._lock:
            future = self._result_futures.pop(key, None)
            self._feedback_callbacks.pop(key, None)
            self._goal_handles.pop(key, None)
        if future:
            future.set_result(msg)

    def _request_cancel(self, goal_id) -> Future:
        key = _uuid_bytes(goal_id)
        future = Future()
        with self._lock:
            self._cancel_futures[key] = future
        request = self._cancel_req_cls()
        goal_info = getattr(request, "goal_info", None)
        if goal_info is not None:
            try:
                setattr(goal_info, "goal_id", _copy_goal_id(goal_id))
            except Exception:
                pass
        else:
            try:
                setattr(request, "goal_id", _copy_goal_id(goal_id))
            except Exception:
                pass
        self._cancel_pub.publish(request)
        return future

    def _on_cancel_response(self, msg):
        return_code = getattr(msg, "return_code", CancelResponse.REJECT.value)
        code = int(return_code)
        # Resolve all pending cancel futures
        with self._lock:
            futures = list(self._cancel_futures.values())
            self._cancel_futures.clear()
        for future in futures:
            future.set_result(code)

    def _on_feedback(self, msg):
        goal_id = getattr(msg, "goal_id", None)
        key = _uuid_bytes(goal_id)
        cb = self._feedback_callbacks.get(key)
        if cb:
            try:
                cb(msg)
            except Exception:
                pass

    def destroy(self):
        for pub in (self._send_goal_pub, self._get_result_pub, self._cancel_pub):
            try:
                pub.destroy()
            except Exception:
                pass
        for sub in (self._send_goal_res_sub, self._result_sub, self._feedback_sub, self._cancel_res_sub):
            try:
                sub.destroy()
            except Exception:
                pass

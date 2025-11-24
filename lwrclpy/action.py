"""
ROS 2 Action-like implementation using SendGoal/GetResult/Feedback/Status/Cancel topics.
Assumes action_msgs/unique_identifier_msgs IDL are present and generated.
"""
import threading
import uuid
import time
from enum import Enum
from typing import Callable, Optional

from .qos import QoSProfile
from .typesupport import RegisteredType
from .utils import resolve_generated_type, resolve_action_type, get_or_create_topic
from .publisher import Publisher
from .subscription import Subscription
from .context import get_participant


class GoalResponse(Enum):
    REJECT = 0
    ACCEPT = 1


class CancelResponse(Enum):
    REJECT = 0
    ACCEPT = 1


def _set_uuid(msg, u: uuid.UUID):
    # Try common field names
    data = list(u.bytes)
    for field in ("uuid",):
        if hasattr(msg, field):
            try:
                setattr(msg, field, data)
                return
            except Exception:
                pass


def _set_stamp_zero(msg):
    for field in ("stamp", "goal_info"):
        target = msg
        if hasattr(msg, field):
            try:
                target = getattr(msg, field)
            except Exception:
                continue
        for sec_name, nsec_name in (("sec", "nanosec"), ("seconds", "nanoseconds")):
            try:
                setattr(target, sec_name, 0)
                setattr(target, nsec_name, 0)
                return
            except Exception:
                continue


def _set_status(res_msg, status: int):
    for field in ("status", "return_code"):
        if hasattr(res_msg, field):
            try:
                setattr(res_msg, field, status)
                return
            except Exception:
                pass


class _GoalHandle:
    def __init__(self, goal_msg, goal_id, result_pub, feedback_pub, feedback_cls, status_pub, status_msg_cls):
        self._goal_msg = goal_msg
        self._goal_id = goal_id
        self._result_pub = result_pub
        self._feedback_pub = feedback_pub
        self._feedback_cls = feedback_cls
        self._status_pub = status_pub
        self._status_msg_cls = status_msg_cls
        self._done = False

    def get_goal(self):
        return self._goal_msg

    def publish_feedback(self, feedback_msg=None):
        msg = feedback_msg if feedback_msg is not None else self._feedback_cls()
        # populate goal_id if field exists
        for field in ("goal_id",):
            if hasattr(msg, field):
                try:
                    setattr(msg, field, self._goal_id)
                except Exception:
                    pass
        self._feedback_pub.publish(msg)

    def succeed(self, result=None):
        if self._done:
            return
        self._done = True
        if result is not None:
            try:
                if hasattr(result, "goal_id"):
                    setattr(result, "goal_id", self._goal_id)
            except Exception:
                pass
            self._result_pub.publish(result)
        self._publish_status(4)  # STATUS_SUCCEEDED

    def abort(self, result=None):
        self._publish_status(5)  # STATUS_ABORTED
        self.succeed(result)

    def canceled(self, result=None):
        self._publish_status(6)  # STATUS_CANCELED
        self.succeed(result)

    def _publish_status(self, status_code: int):
        try:
            status_msg = self._status_msg_cls()
            _set_status(status_msg, status_code)
            if hasattr(status_msg, "goal_id"):
                setattr(status_msg, "goal_id", self._goal_id)
            array_msg = None
            try:
                from action_msgs.msg import GoalStatusArray  # type: ignore
                array_msg = GoalStatusArray.GoalStatusArray() if hasattr(GoalStatusArray, "GoalStatusArray") else GoalStatusArray()
            except Exception:
                array_msg = None
            if array_msg is not None and hasattr(array_msg, "status_list"):
                try:
                    array_msg.status_list = [status_msg]
                    _set_stamp_zero(array_msg)
                    self._status_pub.publish(array_msg)
                    return
                except Exception:
                    pass
            self._status_pub.publish(status_msg)
        except Exception:
            pass


class ActionServer:
    """ActionServer with ROS2-like topics (SendGoal/GetResult/Feedback/Status/Cancel)."""

    def __init__(
        self,
        node,
        action_type,
        action_name: str,
        execute_callback: Callable,
        *,
        goal_callback: Optional[Callable] = None,
        cancel_callback: Optional[Callable] = None,
        callback_group=None,
    ):
        self._participant = get_participant()
        self._execute_callback = execute_callback
        self._goal_callback = goal_callback
        self._cancel_callback = cancel_callback
        self._goals = {}

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

        # register types
        self._goal_type_name = RegisteredType(resolve_generated_type(self._goal_cls)[1]).register()
        self._result_type_name = RegisteredType(resolve_generated_type(self._result_cls)[1]).register()
        self._feedback_type_name = RegisteredType(resolve_generated_type(self._feedback_msg_cls)[1]).register()
        self._status_type_name = RegisteredType(resolve_generated_type(types["cancel_res"]).register if False else resolve_generated_type(types["cancel_res"])[1]).register()

        # topics
        base = action_name.strip("/")
        send_goal_req_topic = f"{base}/_action/send_goal/_request"
        send_goal_res_topic = f"{base}/_action/send_goal/_response"
        get_result_req_topic = f"{base}/_action/get_result/_request"
        get_result_res_topic = f"{base}/_action/get_result/_response"
        feedback_topic = f"{base}/_action/feedback"
        status_topic = f"{base}/_action/status"
        cancel_req_topic = f"{base}/_action/cancel_goal/_request"
        cancel_res_topic = f"{base}/_action/cancel_goal/_response"

        qos = QoSProfile()
        # publishers
        self._send_goal_res_pub = Publisher(
            self._participant,
            self._make_topic(send_goal_res_topic, RegisteredType(resolve_generated_type(self._send_goal_res_cls)[1]).register()),
            qos,
            msg_ctor=resolve_generated_type(self._send_goal_res_cls)[1],
        )
        self._get_result_res_pub = Publisher(
            self._participant,
            self._make_topic(get_result_res_topic, RegisteredType(resolve_generated_type(self._get_result_res_cls)[1]).register()),
            qos,
            msg_ctor=resolve_generated_type(self._get_result_res_cls)[1],
        )
        self._feedback_pub = Publisher(
            self._participant,
            self._make_topic(feedback_topic, self._feedback_type_name),
            qos,
            msg_ctor=resolve_generated_type(self._feedback_msg_cls)[1],
        )
        self._status_pub = Publisher(
            self._participant,
            self._make_topic(status_topic, RegisteredType(resolve_generated_type(types["cancel_res"])[1]).register()),
            qos,
            msg_ctor=resolve_generated_type(self._cancel_res_cls)[1],
        )
        self._cancel_res_pub = Publisher(
            self._participant,
            self._make_topic(cancel_res_topic, RegisteredType(resolve_generated_type(self._cancel_res_cls)[1]).register()),
            qos,
            msg_ctor=resolve_generated_type(self._cancel_res_cls)[1],
        )

        send_goal_req_ctor = resolve_generated_type(self._send_goal_req_cls)[1]
        get_result_req_ctor = resolve_generated_type(self._get_result_req_cls)[1]
        cancel_req_ctor = resolve_generated_type(self._cancel_req_cls)[1]

        def _on_send_goal(req_msg):
            goal_id = getattr(req_msg, "goal_id", None)
            goal = getattr(req_msg, "goal", None)
            accepted = True
            if self._goal_callback:
                resp = self._goal_callback(goal)
                if resp == GoalResponse.REJECT:
                    accepted = False
            res_msg = self._send_goal_res_cls()
            try:
                setattr(res_msg, "accepted", accepted)
            except Exception:
                pass
            _set_stamp_zero(res_msg)
            if accepted and goal_id is not None:
                gh = _GoalHandle(goal, goal_id, self._get_result_res_pub, self._feedback_pub, self._feedback_msg_cls, self._status_pub, self._cancel_res_cls)
                self._goals[_uuid_bytes(goal_id)] = (gh, self._result_cls())
                # run execute in background
                thr = threading.Thread(target=self._run_execute, args=(gh,), daemon=True)
                thr.start()
            self._send_goal_res_pub.publish(res_msg)

        def _on_get_result(req_msg):
            gid = getattr(req_msg, "goal_id", None)
            res_msg = self._get_result_res_cls()
            _set_status(res_msg, 0)
            try:
                if hasattr(res_msg, "goal_id") and gid is not None:
                    setattr(res_msg, "goal_id", gid)
            except Exception:
                pass
            key = _uuid_bytes(gid) if gid is not None else None
            if key in self._goals:
                gh, result_obj = self._goals[key]
                try:
                    if hasattr(res_msg, "result"):
                        setattr(res_msg, "result", result_obj)
                except Exception:
                    pass
            self._get_result_res_pub.publish(res_msg)

        def _on_cancel(req_msg):
            res = self._cancel_res_cls()
            _set_status(res, CancelResponse.ACCEPT.value)
            self._cancel_res_pub.publish(res)

        self._send_goal_sub = Subscription(
            self._participant,
            self._make_topic(send_goal_req_topic, RegisteredType(resolve_generated_type(self._send_goal_req_cls)[1]).register()),
            qos,
            _on_send_goal,
            send_goal_req_ctor,
            enqueue_cb=node._enqueue_callback,
        )
        self._get_result_sub = Subscription(
            self._participant,
            self._make_topic(get_result_req_topic, RegisteredType(resolve_generated_type(self._get_result_req_cls)[1]).register()),
            qos,
            _on_get_result,
            get_result_req_ctor,
            enqueue_cb=node._enqueue_callback,
        )
        self._cancel_sub = Subscription(
            self._participant,
            self._make_topic(cancel_req_topic, RegisteredType(resolve_generated_type(self._cancel_req_cls)[1]).register()),
            qos,
            _on_cancel,
            cancel_req_ctor,
            enqueue_cb=node._enqueue_callback,
        )

    def _make_topic(self, name: str, type_name: str):
        topic_obj, _ = get_or_create_topic(self._participant, name, type_name)
        return topic_obj

    def _run_execute(self, goal_handle: _GoalHandle):
        try:
            res = self._execute_callback(goal_handle)
            if res is not None:
                goal_handle.succeed(res)
        except Exception:
            return

    def destroy(self):
        self._send_goal_sub.destroy()
        self._get_result_sub.destroy()
        self._cancel_sub.destroy()
        self._send_goal_res_pub.destroy()
        self._get_result_res_pub.destroy()
        self._feedback_pub.destroy()
        self._status_pub.destroy()
        self._cancel_res_pub.destroy()


class ActionClient:
    """ActionClient using ROS2-like action topics."""

    def __init__(
        self,
        node,
        action_type,
        action_name: str,
        *,
        callback_group=None,
    ):
        self._participant = get_participant()
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

        base = action_name.strip("/")
        self._send_goal_req_topic = f"{base}/_action/send_goal/_request"
        self._send_goal_res_topic = f"{base}/_action/send_goal/_response"
        self._get_result_req_topic = f"{base}/_action/get_result/_request"
        self._get_result_res_topic = f"{base}/_action/get_result/_response"
        self._feedback_topic = f"{base}/_action/feedback"
        self._status_topic = f"{base}/_action/status"
        self._cancel_req_topic = f"{base}/_action/cancel_goal/_request"
        self._cancel_res_topic = f"{base}/_action/cancel_goal/_response"

        qos = QoSProfile()
        # pubs/subs
        self._send_goal_pub = Publisher(
            self._participant,
            self._make_topic(self._send_goal_req_topic, RegisteredType(resolve_generated_type(self._send_goal_req_cls)[1]).register()),
            qos,
            msg_ctor=resolve_generated_type(self._send_goal_req_cls)[1],
        )
        self._get_result_pub = Publisher(
            self._participant,
            self._make_topic(self._get_result_req_topic, RegisteredType(resolve_generated_type(self._get_result_req_cls)[1]).register()),
            qos,
            msg_ctor=resolve_generated_type(self._get_result_req_cls)[1],
        )
        self._cancel_pub = Publisher(
            self._participant,
            self._make_topic(self._cancel_req_topic, RegisteredType(resolve_generated_type(self._cancel_req_cls)[1]).register()),
            qos,
            msg_ctor=resolve_generated_type(self._cancel_req_cls)[1],
        )

        send_goal_res_ctor = resolve_generated_type(self._send_goal_res_cls)[1]
        get_result_res_ctor = resolve_generated_type(self._get_result_res_cls)[1]
        feedback_ctor = resolve_generated_type(self._feedback_msg_cls)[1]
        cancel_res_ctor = resolve_generated_type(self._cancel_res_cls)[1]

        self._cond = threading.Condition()
        self._result = None
        self._goal_id = None
        self._feedback_cb: Optional[Callable] = None

        def _on_send_goal_response(msg):
            with self._cond:
                self._accepted = getattr(msg, "accepted", True)
                self._cond.notify_all()

        def _on_result(msg):
            with self._cond:
                self._result = msg
                self._cond.notify_all()

        def _on_feedback(msg):
            cb = self._feedback_cb
            if cb:
                try:
                    fb = getattr(msg, "feedback", msg)
                    cb(fb)
                except Exception:
                    pass

        self._send_goal_res_sub = Subscription(
            self._participant,
            self._make_topic(self._send_goal_res_topic, RegisteredType(resolve_generated_type(self._send_goal_res_cls)[1]).register()),
            qos,
            _on_send_goal_response,
            send_goal_res_ctor,
            enqueue_cb=node._enqueue_callback,
        )
        self._result_sub = Subscription(
            self._participant,
            self._make_topic(self._get_result_res_topic, RegisteredType(resolve_generated_type(self._get_result_res_cls)[1]).register()),
            qos,
            _on_result,
            get_result_res_ctor,
            enqueue_cb=node._enqueue_callback,
        )
        self._feedback_sub = Subscription(
            self._participant,
            self._make_topic(self._feedback_topic, RegisteredType(resolve_generated_type(self._feedback_msg_cls)[1]).register()),
            qos,
            _on_feedback,
            feedback_ctor,
            enqueue_cb=node._enqueue_callback,
        )
        self._cancel_res_sub = Subscription(
            self._participant,
            self._make_topic(self._cancel_res_topic, RegisteredType(resolve_generated_type(self._cancel_res_cls)[1]).register()),
            qos,
            lambda msg: None,
            cancel_res_ctor,
            enqueue_cb=node._enqueue_callback,
        )

    def _make_topic(self, name: str, type_name: str):
        topic_obj, _ = get_or_create_topic(self._participant, name, type_name)
        return topic_obj

    def send_goal(self, goal, feedback_callback: Optional[Callable] = None):
        self._feedback_cb = feedback_callback
        self._goal_id = self._make_goal_id()
        req = self._send_goal_req_cls()
        try:
            setattr(req, "goal_id", self._goal_id)
            setattr(req, "goal", goal)
        except Exception:
            pass
        with self._cond:
            self._accepted = False
        self._send_goal_pub.publish(req)
        return True

    def wait_for_result(self, timeout: Optional[float] = None):
        with self._cond:
            if self._result is not None:
                return True
            if timeout is None:
                while self._result is None:
                    self._cond.wait()
                return True
            else:
                self._cond.wait(timeout=timeout)
                return self._result is not None

    def get_result(self):
        with self._cond:
            return self._result

    def cancel_goal(self):
        if self._goal_id is None:
            return False
        req = self._cancel_req_cls()
        try:
            setattr(req, "goal_id", self._goal_id)
        except Exception:
            pass
        self._cancel_pub.publish(req)
        return True

    def _make_goal_id(self):
        gid = self._goal_cls() if False else None
        uid = uuid.uuid4()
        msg = self._send_goal_req_cls()
        goal_id_field = getattr(msg, "goal_id", None)
        if goal_id_field is None:
            goal_id_field = type("goal_id", (), {})()
        _set_uuid(goal_id_field, uid)
        return goal_id_field

    def destroy(self):
        self._send_goal_res_sub.destroy()
        self._result_sub.destroy()
        self._feedback_sub.destroy()
        self._cancel_res_sub.destroy()
        self._send_goal_pub.destroy()
        self._get_result_pub.destroy()
        self._cancel_pub.destroy()

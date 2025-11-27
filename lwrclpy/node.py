import logging
import time
from typing import Optional

from .context import get_participant
from .parameters import Parameter, ParameterType, SetParametersResult, coerce_parameter
from .qos import QoSProfile
from .publisher import Publisher
from .subscription import Subscription
from .typesupport import RegisteredType
from .utils import (
    resolve_generated_type,
    get_or_create_topic,
    resolve_name,
    TOPIC_PREFIX,
)
from .client import Client
from .service import Service
from .clock import Clock
from .guard_condition import GuardCondition


# --- rclpy.Rate 相当（壁時計ベース） -----------------------------------------
class _WallRate:
    """ROS2 の rclpy.Rate に似せた壁時計ベースの Rate。
    - hz を指定（例: 10.0 なら 100ms 周期）
    - sleep() は次の周期まで待機し、ドリフトを補正
    - reset() で次周期基準を「現在」にリセット
    """

    __slots__ = ("_period_ns", "_next_ns")

    def __init__(self, hz: float):
        if hz <= 0:
            raise ValueError("rate hz must be > 0")
        self._period_ns = int(1e9 / float(hz))
        now = time.monotonic_ns()
        self._next_ns = now + self._period_ns

    def reset(self) -> None:
        """次の起床時刻を現在からにリセット。"""
        self._next_ns = time.monotonic_ns() + self._period_ns

    def sleep(self) -> None:
        """次の周期までスリープ。過剰遅延時は直ちに次周期を再設定して戻る（busy wait はしない）。"""
        now = time.monotonic_ns()
        # 既に次周期を過ぎている場合は、抜けるだけ（次基準を進めておく）
        if now >= self._next_ns:
            # どれだけ遅延していても「今から 1 周期後」を次基準にする（ドリフト補正）
            self._next_ns = now + self._period_ns
            return

        # 残り時間を sleep
        remaining_ns = self._next_ns - now
        time.sleep(remaining_ns / 1e9)

        # 次基準を 1 周期進める
        self._next_ns += self._period_ns


class _NodeLogger:
    """Minimal rclpy.get_logger() equivalent backed by Python logging."""

    _configured = False

    def __init__(self, name: str):
        if not _NodeLogger._configured and not logging.getLogger().handlers:
            logging.basicConfig(
                level=logging.INFO,
                format="[%(levelname)s] %(name)s: %(message)s",
            )
            _NodeLogger._configured = True
        self._logger = logging.getLogger(name)

    def debug(self, msg, *args, **kwargs):
        self._logger.debug(msg, *args, **kwargs)

    def info(self, msg, *args, **kwargs):
        self._logger.info(msg, *args, **kwargs)

    def warn(self, msg, *args, **kwargs):
        self._logger.warning(msg, *args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        self._logger.warning(msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        self._logger.error(msg, *args, **kwargs)

    def fatal(self, msg, *args, **kwargs):
        self._logger.fatal(msg, *args, **kwargs)

    def critical(self, msg, *args, **kwargs):
        self._logger.critical(msg, *args, **kwargs)

    def log(self, level, msg, *args, **kwargs):
        self._logger.log(level, msg, *args, **kwargs)


class Node:
    def __init__(
        self,
        name: str,
        namespace: str = "",
        *,
        allow_undeclared_parameters: bool = True,
        automatically_declare_parameters_from_overrides: bool = False,
        parameters: Optional[list[Parameter]] = None,
    ):
        self._name = name
        self._namespace = namespace if namespace.startswith("/") or namespace == "" else "/" + namespace
        self._pubsub_prefix = TOPIC_PREFIX  # ROS 2 DDS mapping: rt/<topic>
        self._service_prefix = ""  # rq/rr are applied in client/service helpers
        self._participant = get_participant()
        self._logger = _NodeLogger(self.get_fully_qualified_name())
        self._topics: dict[str, tuple[object, bool]] = {}  # resolved name -> (Topic, owned)
        self._publishers = []
        self._subscriptions = []
        self._timers = []
        self._guard_conditions = []
        self._callback_queue = []
        self._type_cache = {}  # key: message classの完全修飾名 / module名
        self._parameters: dict[str, Parameter] = {}
        self._allow_undeclared_parameters = allow_undeclared_parameters
        self._auto_declare_from_overrides = automatically_declare_parameters_from_overrides
        self._clock = Clock()

        if parameters:
            self.declare_parameters("", [(p.name, p.value) if isinstance(p, Parameter) else p for p in parameters])

    def _cache_key(self, msg_cls):
        return f"{msg_cls.__module__}.{msg_cls.__name__}"

    def _resolve_topic_name(self, name: str) -> str:
        """Apply ROS 2 name resolution and DDS topic prefix (rt/)."""
        resolved = resolve_name(name, self._namespace, self._name).lstrip("/")
        if resolved.startswith(self._pubsub_prefix):
            return resolved
        return f"{self._pubsub_prefix}{resolved}" if self._pubsub_prefix else resolved

    # ------------------- rclpy 風の Rate / sleep -------------------
    def create_rate(self, hz: float) -> _WallRate:
        """rclpy.create_rate に相当（Node メソッド版）。壁時計ベース。"""
        return _WallRate(hz)

    # 好みで使えるエイリアス（rclpy.rate(...) 風）
    def rate(self, hz: float) -> _WallRate:
        return self.create_rate(hz)

    def sleep(self, seconds: float) -> None:
        """rclpy の簡易 sleep に相当（壁時計の time.sleep）。"""
        if seconds <= 0:
            return
        time.sleep(float(seconds))

    # ------------------- Logger / Parameters / Namespace -------------
    def get_logger(self):
        return self._logger

    def get_namespace(self) -> str:
        return self._namespace if self._namespace else "/"

    def get_fully_qualified_name(self) -> str:
        ns = self.get_namespace().rstrip("/")
        return f"{ns}/{self._name}" if ns else f"/{self._name}"

    def get_clock(self) -> Clock:
        return self._clock

    def declare_parameter(self, name: str, value=None):
        """Store a parameter locally (best-effort rclpy compatibility)."""
        if name in self._parameters:
            return self._parameters[name]
        param = Parameter(name, value)
        self._parameters[name] = param
        return param

    def declare_parameters(self, namespace: str, parameters):
        """Bulk declare with optional namespace prefix."""
        ns = namespace.rstrip("/") + "/" if namespace else ""
        declared = []
        for p in parameters:
            if isinstance(p, Parameter):
                name, value = p.name, p.value
            else:
                name, value = p
            declared.append(self.declare_parameter(ns + name, value))
        return declared

    def has_parameter(self, name: str) -> bool:
        return name in self._parameters

    def get_parameter(self, name: str) -> Parameter:
        if name in self._parameters:
            return self._parameters[name]
        if self._allow_undeclared_parameters:
            return Parameter(name, ParameterType.NOT_SET)
        raise KeyError(f"Parameter '{name}' is not declared")

    def get_parameters(self, names) -> list[Parameter]:
        return [self.get_parameter(n) for n in names]

    def set_parameters(self, parameters) -> list[SetParametersResult]:
        """Set parameters from Parameter objects or (name, value) tuples."""
        results = []
        for p in parameters:
            param = coerce_parameter(p)
            if param.name not in self._parameters:
                if self._auto_declare_from_overrides or self._allow_undeclared_parameters:
                    self._parameters[param.name] = Parameter(param.name, param.value)
                else:
                    results.append(SetParametersResult(False, f"Parameter '{param.name}' not declared"))
                    continue
            self._parameters[param.name] = param
            results.append(SetParametersResult(True, ""))
        return results

    def set_parameters_atomically(self, parameters) -> SetParametersResult:
        res = self.set_parameters(parameters)
        ok = all(r.successful for r in res)
        reason = next((r.reason for r in res if not r.successful), "")
        return SetParametersResult(ok, reason)

    # ------------------- Publisher / Subscription 等 -------------------
    def create_publisher(self, msg_type, topic: str, qos_profile: QoSProfile | int = 10, *, callback_group=None):
        qos = qos_profile if isinstance(qos_profile, QoSProfile) else QoSProfile(depth=int(qos_profile))
        # 型解決（モジュール or クラスの両対応）
        _mod, msg_cls, _pubsub_cls = resolve_generated_type(msg_type)
        key = self._cache_key(msg_cls)
        type_name = self._type_cache.get(key)
        if not type_name:
            ts = RegisteredType(msg_cls)
            type_name = ts.register()
            self._type_cache[key] = type_name
        resolved_topic = self._resolve_topic_name(topic)
        topic_obj, owned = self._create_topic(resolved_topic, type_name)
        self._topics[resolved_topic] = (topic_obj, owned)
        pub = Publisher(self._participant, topic_obj, qos, msg_ctor=msg_cls)
        self._publishers.append(pub)
        return pub

    def create_subscription(self, msg_type, topic: str, callback, qos_profile: QoSProfile | int = 10, *, callback_group=None):
        qos = qos_profile if isinstance(qos_profile, QoSProfile) else QoSProfile(depth=int(qos_profile))
        # 型解決（モジュール or クラスの両対応）
        _mod, msg_cls, _pubsub_cls = resolve_generated_type(msg_type)
        key = self._cache_key(msg_cls)
        type_name = self._type_cache.get(key)
        if not type_name:
            ts = RegisteredType(msg_cls)
            type_name = ts.register()
            self._type_cache[key] = type_name
        resolved_topic = self._resolve_topic_name(topic)
        topic_obj, owned = self._create_topic(resolved_topic, type_name)
        self._topics[resolved_topic] = (topic_obj, owned)
        # メッセージ生成
        msg_ctor = msg_cls
        sub = Subscription(self._participant, topic_obj, qos, callback, msg_ctor, self._enqueue_callback)
        self._subscriptions.append(sub)
        return sub

    def create_client(self, srv_type, srv_name: str, qos_profile: QoSProfile | int = 10, *, callback_group=None):
        qos = qos_profile if isinstance(qos_profile, QoSProfile) else QoSProfile(depth=int(qos_profile))
        resolved = resolve_name(srv_name, self._namespace, self._name)
        return Client(srv_type, resolved, qos, topic_prefix=self._service_prefix, enqueue_cb=self._enqueue_callback)

    def create_service(self, srv_type, srv_name: str, callback, qos_profile: QoSProfile | int = 10, *, callback_group=None):
        qos = qos_profile if isinstance(qos_profile, QoSProfile) else QoSProfile(depth=int(qos_profile))
        resolved = resolve_name(srv_name, self._namespace, self._name)
        return Service(srv_type, resolved, callback, qos, topic_prefix=self._service_prefix)

    def create_action_server(self, action_type, action_name: str, execute_callback, **kwargs):
        from .action import ActionServer
        return ActionServer(self, action_type, action_name, execute_callback, **kwargs)

    def create_action_client(self, action_type, action_name: str, **kwargs):
        from .action import ActionClient
        return ActionClient(self, action_type, action_name, **kwargs)

    def create_timer(self, period_sec: float, callback, *, callback_group=None, oneshot: bool = False):
        from .timer import create_timer
        # Enqueue timer callbacks into the node's callback queue
        t = create_timer(period_sec, callback, oneshot=oneshot, enqueue_cb=self._enqueue_callback)
        self._timers.append(t)
        return t

    def create_wall_timer(self, period_sec: float, callback):
        return self.create_timer(period_sec, callback)

    def create_guard_condition(self, callback, *, callback_group=None):
        gc = GuardCondition(callback, self._enqueue_callback)
        self._guard_conditions.append(gc)
        return gc

    def destroy_publisher(self, pub):
        try:
            pub.destroy()
        finally:
            if pub in self._publishers:
                self._publishers.remove(pub)

    def destroy_subscription(self, sub):
        try:
            sub.destroy()
        finally:
            if sub in self._subscriptions:
                self._subscriptions.remove(sub)

    def get_name(self):
        return self._name

    def destroy_node(self):
        for t in self._timers:
            try:
                t.cancel()
            except Exception:
                pass
        self._timers.clear()
        for pub in self._publishers:
            try:
                pub.destroy()
            except Exception:
                pass
        self._publishers.clear()
        for sub in self._subscriptions:
            try:
                sub.destroy()
            except Exception:
                pass
        self._subscriptions.clear()
        for gc in self._guard_conditions:
            try:
                gc.destroy()
            except Exception:
                pass
        self._guard_conditions.clear()
        for name, (tobj, owned) in list(self._topics.items()):
            try:
                if owned:
                    self._participant.delete_topic(tobj)
            except Exception:
                pass
        self._topics.clear()

    # ------------- internal helpers -----------------
    def _create_topic(self, name: str, type_name: str):
        # Reuse already created topics per name when possible to avoid duplicate registration
        existing = self._topics.get(name)
        if existing:
            return existing
        return get_or_create_topic(self._participant, name, type_name)

    # ---- executor enqueue/dequeue -----------------------------------
    def _enqueue_callback(self, cb, msg):
        self._callback_queue.append((cb, msg))

    def _drain_callbacks(self):
        queue = self._callback_queue
        self._callback_queue = []
        return queue

    def _pop_callback(self):
        if not self._callback_queue:
            return None
        return self._callback_queue.pop(0)

    def _has_pending_work(self) -> bool:
        """Internal: check if callbacks are queued or timers are still active."""
        if self._callback_queue:
            return True
        for t in self._timers:
            try:
                if not t.is_canceled():
                    return True
            except Exception:
                continue
        return False


# --- Module-level helpers to mirror rclpy API --------------------------------
def create_node(name: str, **kwargs) -> Node:
    return Node(name, **kwargs)


def create_rate(hz: float) -> _WallRate:
    return _WallRate(hz)


Rate = _WallRate

import time
from typing import Optional

import fastdds
from .context import get_participant
from .qos import QoSProfile
from .publisher import Publisher
from .subscription import Subscription
from .typesupport import RegisteredType
from .utils import resolve_generated_type


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


class Node:
    def __init__(self, name: str):
        self._name = name
        self._participant = get_participant()
        self._topics = {}
        self._publishers = []
        self._subscriptions = []
        self._timers = []
        self._type_cache = {}  # key: message classの完全修飾名 / module名

    def _cache_key(self, msg_cls):
        return f"{msg_cls.__module__}.{msg_cls.__name__}"

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

    # ------------------- Publisher / Subscription 等 -------------------
    def create_publisher(self, msg_type, topic: str, qos_profile: QoSProfile | int = 10):
        qos = qos_profile if isinstance(qos_profile, QoSProfile) else QoSProfile(depth=int(qos_profile))
        # 型解決（モジュール or クラスの両対応）
        _mod, msg_cls, _pubsub_cls = resolve_generated_type(msg_type)
        key = self._cache_key(msg_cls)
        type_name = self._type_cache.get(key)
        if not type_name:
            ts = RegisteredType(msg_cls)
            type_name = ts.register()
            self._type_cache[key] = type_name
        # トピック
        tq = fastdds.TopicQos()
        # add "rt/" prefix for real-time topic by default
        if not topic.startswith("rt/"):
            topic = "rt/" + topic
        self._participant.get_default_topic_qos(tq)
        topic_obj = self._participant.create_topic(topic, type_name, tq)
        self._topics[topic] = topic_obj
        pub = Publisher(self._participant, topic_obj, qos)
        self._publishers.append(pub)
        return pub

    def create_subscription(self, msg_type, topic: str, callback, qos_profile: QoSProfile | int = 10):
        qos = qos_profile if isinstance(qos_profile, QoSProfile) else QoSProfile(depth=int(qos_profile))
        # 型解決（モジュール or クラスの両対応）
        _mod, msg_cls, _pubsub_cls = resolve_generated_type(msg_type)
        key = self._cache_key(msg_cls)
        type_name = self._type_cache.get(key)
        if not type_name:
            ts = RegisteredType(msg_cls)
            type_name = ts.register()
            self._type_cache[key] = type_name
        # トピック
        tq = fastdds.TopicQos()
        # add "rt/" prefix for real-time topic by default
        if not topic.startswith("rt/"):
            topic = "rt/" + topic
        self._participant.get_default_topic_qos(tq)
        topic_obj = self._participant.create_topic(topic, type_name, tq)
        self._topics[topic] = topic_obj
        # メッセージ生成
        msg_ctor = msg_cls
        sub = Subscription(self._participant, topic_obj, qos, callback, msg_ctor)
        self._subscriptions.append(sub)
        return sub

    def create_timer(self, period_sec: float, callback):
        from .timer import create_timer
        t = create_timer(period_sec, callback)
        self._timers.append(t)
        return t

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
        for _, tobj in list(self._topics.items()):
            try:
                self._participant.delete_topic(tobj)
            except Exception:
                pass
        self._topics.clear()
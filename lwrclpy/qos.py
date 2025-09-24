from enum import Enum

try:
    import fastdds  # Fast DDS v3 の Python API を想定
except Exception as e:
    raise RuntimeError("fastdds Python bindings are not available") from e


# Python API では、QoS の kind はモジュール直下の定数で提供される
# 参考（v3系のPython API）:
#  - History:  fastdds.KEEP_LAST_HISTORY_QOS / fastdds.KEEP_ALL_HISTORY_QOS
#  - Reliability: fastdds.RELIABLE_RELIABILITY_QOS / fastdds.BEST_EFFORT_RELIABILITY_QOS
#  - Durability:  fastdds.VOLATILE_DURABILITY_QOS / fastdds.TRANSIENT_LOCAL_DURABILITY_QOS

# 存在チェック（ない場合は v3 API 未満／不整合）
REQUIRED = [
    "KEEP_LAST_HISTORY_QOS", "KEEP_ALL_HISTORY_QOS",
    "RELIABLE_RELIABILITY_QOS", "BEST_EFFORT_RELIABILITY_QOS",
    "VOLATILE_DURABILITY_QOS", "TRANSIENT_LOCAL_DURABILITY_QOS",
]
_missing = [n for n in REQUIRED if not hasattr(fastdds, n)]
if _missing:
    raise AttributeError(
        "Fast DDS Python API does not expose expected v3 symbols: "
        + ", ".join(_missing)
        + ". Reinstall Fast-DDS v3 + Fast-DDS-python (colcon) to match v3 Python API."
    )


class HistoryPolicy(Enum):
    KEEP_LAST = fastdds.KEEP_LAST_HISTORY_QOS
    KEEP_ALL = fastdds.KEEP_ALL_HISTORY_QOS


class ReliabilityPolicy(Enum):
    RELIABLE = fastdds.RELIABLE_RELIABILITY_QOS
    BEST_EFFORT = fastdds.BEST_EFFORT_RELIABILITY_QOS


class DurabilityPolicy(Enum):
    VOLATILE = fastdds.VOLATILE_DURABILITY_QOS
    TRANSIENT_LOCAL = fastdds.TRANSIENT_LOCAL_DURABILITY_QOS


class QoSProfile:
    """
    rclpy 風の QoSProfile（必要最小限）。
    Fast DDS v3 の DataWriterQos / DataReaderQos に適用。
    """
    def __init__(
        self,
        depth: int = 10,
        history: HistoryPolicy = HistoryPolicy.KEEP_LAST,
        reliability: ReliabilityPolicy = ReliabilityPolicy.RELIABLE,
        durability: DurabilityPolicy = DurabilityPolicy.VOLATILE,
    ):
        self.depth = int(depth)
        self.history = history
        self.reliability = reliability
        self.durability = durability

    def apply_to_writer(self, wq: "fastdds.DataWriterQos"):
        # History
        wq.history().kind = self.history.value
        # KEEP_LAST のときのみ意味を持つが、設定しても害はない
        try:
            wq.history().depth = self.depth
        except Exception:
            pass
        # Reliability / Durability
        wq.reliability().kind = self.reliability.value
        wq.durability().kind = self.durability.value

    def apply_to_reader(self, rq: "fastdds.DataReaderQos"):
        rq.history().kind = self.history.value
        try:
            rq.history().depth = self.depth
        except Exception:
            pass
        rq.reliability().kind = self.reliability.value
        rq.durability().kind = self.durability.value
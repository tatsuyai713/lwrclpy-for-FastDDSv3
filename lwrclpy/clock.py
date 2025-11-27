import time


class _TimePoint:
    def __init__(self, nanoseconds: int):
        self._nanoseconds = int(nanoseconds)

    @property
    def nanoseconds(self) -> int:
        return self._nanoseconds

    def to_msg(self):
        try:
            from builtin_interfaces.msg import Time  # type: ignore

            msg = Time()
            msg.sec = self._nanoseconds // 1_000_000_000
            msg.nanosec = self._nanoseconds % 1_000_000_000
            return msg
        except Exception:
            return None


class Clock:
    """Wall-clock based time provider similar to rclpy.clock.Clock."""

    def now(self) -> _TimePoint:
        return _TimePoint(time.time_ns())

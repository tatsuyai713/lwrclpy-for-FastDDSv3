class Duration:
    """Simple duration helper mirroring rclpy.duration.Duration for sample compatibility."""

    def __init__(self, *, seconds: float = 0.0, nanoseconds: int = 0):
        total_ns = int(nanoseconds)
        total_ns += int(seconds * 1_000_000_000)
        self._nanoseconds = total_ns

    @property
    def nanoseconds(self) -> int:
        return self._nanoseconds

    def seconds(self) -> float:
        return self._nanoseconds / 1_000_000_000

    def to_msg(self):
        try:
            from builtin_interfaces.msg import Duration as DurationMsg  # type: ignore

            msg = DurationMsg()
            msg.sec = self._nanoseconds // 1_000_000_000
            msg.nanosec = self._nanoseconds % 1_000_000_000
            return msg
        except Exception:
            return None

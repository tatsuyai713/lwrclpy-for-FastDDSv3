class CallbackGroup:
    """Compatibility stub for rclpy.callback_groups.CallbackGroup."""

    def __init__(self):
        pass


class MutuallyExclusiveCallbackGroup(CallbackGroup):
    """Allows only one callback at a time (no-op stub)."""


class ReentrantCallbackGroup(CallbackGroup):
    """Allows callbacks to re-enter (no-op stub)."""


__all__ = ["CallbackGroup", "MutuallyExclusiveCallbackGroup", "ReentrantCallbackGroup"]

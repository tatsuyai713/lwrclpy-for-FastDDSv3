import threading
import time


class _RepeatingTimer:
    """Lightweight repeating/oneshot timer similar to rclpy.Timer."""

    def __init__(self, period_sec: float, callback, *, oneshot: bool = False, enqueue_cb=None):
        self._period = float(period_sec)
        self._callback = callback
        self._oneshot = oneshot
        self._enqueue_cb = enqueue_cb
        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._run, daemon=True)
        self._next_t = time.monotonic() + self._period
        self._last_call: float | None = None

    def start(self):
        self._thr.start()

    def _run(self):
        while not self._stop.is_set():
            now = time.monotonic()
            sleep = self._next_t - now
            if sleep > 0:
                self._stop.wait(sleep)
            if self._stop.is_set():
                break
            try:
                self._last_call = time.monotonic()
                if self._enqueue_cb is not None:
                    # Enqueue the user callback to be run by the executor
                    self._enqueue_cb(self._callback, None)
                else:
                    self._callback()
            finally:
                if self._oneshot:
                    self._stop.set()
                    break
                self._next_t = (self._next_t if self._last_call is None else self._last_call) + self._period

    def cancel(self):
        """Request stop and join from external threads; skip join when self-canceling."""
        self._stop.set()
        # Avoid joining the current thread (raises RuntimeError)
        if threading.current_thread() is not self._thr:
            self._thr.join(timeout=1)

    def reset(self):
        """Reset next wake-up to now + period."""
        self._next_t = time.monotonic() + self._period

    def is_canceled(self) -> bool:
        return self._stop.is_set()

    def time_until_next_call(self) -> float:
        return max(0.0, self._next_t - time.monotonic())


def create_timer(period_sec: float, callback, *, oneshot: bool = False, enqueue_cb=None):
    """
    Create a repeating or oneshot timer.
    If enqueue_cb is provided, callbacks are queued (cb, msg) style instead of invoked in the timer thread.
    """
    t = _RepeatingTimer(period_sec, callback, oneshot=oneshot, enqueue_cb=enqueue_cb)
    t.start()
    return t

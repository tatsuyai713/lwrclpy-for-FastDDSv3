import asyncio
import threading
from typing import Any, Callable, Optional


class Future:
    """Minimal Future compatible with rclpy.task.Future."""

    def __init__(self):
        self._event = threading.Event()
        self._result: Any = None
        self._exception: BaseException | None = None
        self._done = False
        self._callbacks: list[Callable[["Future"], None]] = []
        self._lock = threading.Lock()

    def done(self) -> bool:
        return self._done

    def result(self, timeout: Optional[float] = None):
        if not self._event.wait(timeout):
            raise TimeoutError("Future result not ready")
        if self._exception:
            raise self._exception
        return self._result

    def exception(self) -> BaseException | None:
        if not self._done:
            return None
        return self._exception

    def set_result(self, value) -> None:
        with self._lock:
            if self._done:
                return
            self._result = value
            self._done = True
        self._event.set()
        self._run_callbacks()

    def set_exception(self, exc: BaseException) -> None:
        with self._lock:
            if self._done:
                return
            self._exception = exc
            self._done = True
        self._event.set()
        self._run_callbacks()

    def add_done_callback(self, callback: Callable[["Future"], None]) -> None:
        with self._lock:
            if self._done:
                run_now = True
            else:
                self._callbacks.append(callback)
                run_now = False
        if run_now:
            try:
                callback(self)
            except Exception:
                pass

    def _run_callbacks(self):
        callbacks = []
        with self._lock:
            callbacks = list(self._callbacks)
            self._callbacks.clear()
        for cb in callbacks:
            try:
                cb(self)
            except Exception:
                continue

    def cancel(self) -> bool:
        # Simple implementation: mark as canceled via exception
        if self._done:
            return False
        self.set_exception(asyncio.CancelledError())
        return True

    def cancelled(self) -> bool:
        return isinstance(self._exception, asyncio.CancelledError)

    async def _await_impl(self):
        while not self._done:
            await asyncio.sleep(0)
        if self._exception:
            raise self._exception
        return self._result

    def __await__(self):
        return self._await_impl().__await__()

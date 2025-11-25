import asyncio
import inspect
import threading
import time
from typing import Iterable
from .context import ok


class ExternalShutdownException(RuntimeError):
    """Raised when spin exits because the context was shutdown externally."""


class Executor:
    """Base executor compatible with rclpy's Executor surface."""

    def __init__(self):
        self._nodes = []
        self._stopped = False

    def add_node(self, node):
        if node not in self._nodes:
            self._nodes.append(node)

    def remove_node(self, node):
        if node in self._nodes:
            self._nodes.remove(node)

    def spin(self):
        while ok() and not self._stopped:
            self.spin_once(0.01)
        if not ok() and not self._stopped:
            raise ExternalShutdownException()

    def spin_once(self, timeout_sec: float | None = None):
        try:
            handler, _group, _node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except StopIteration:
            return
        try:
            handler()
        except Exception:
            pass

    def spin_some(self, timeout_sec: float | None = None):
        if not ok() or self._stopped:
            return
        self._process_all_ready(self._nodes)
        if timeout_sec:
            time.sleep(min(timeout_sec, 0.001))

    def shutdown(self):
        self._stopped = True

    def wait_for_ready_callbacks(self, timeout_sec: float | None = None):
        item = _pop_any_callback(self._nodes, timeout_sec, self._stopped)
        if not item:
            raise StopIteration()
        cb, msg, node = item
        return (lambda: _invoke_callback(cb, msg), None, node)

    def _process_all_ready(self, nodes: Iterable):
        for node in list(nodes):
            while True:
                try:
                    item = node._pop_callback()
                except Exception:
                    item = None
                if not item:
                    break
                cb, msg = item
                _invoke_callback(cb, msg)


class SingleThreadedExecutor(Executor):
    """Runs callbacks sequentially in the calling thread."""

    def __init__(self):
        super().__init__()


class MultiThreadedExecutor(Executor):
    """Processes callbacks across a thread pool."""

    def __init__(self, num_threads: int | None = None):
        super().__init__()
        self._threads: list[threading.Thread] = []
        self._num_threads = num_threads

    def spin(self):
        if self._stopped:
            return
        thread_count = self._num_threads or max(1, len(self._nodes))
        if thread_count <= 0:
            thread_count = 1
        for _ in range(thread_count):
            t = threading.Thread(target=self._worker, daemon=True)
            t.start()
            self._threads.append(t)
        try:
            while ok() and not self._stopped:
                if not any(getattr(n, "_has_pending_work", lambda: True)() for n in list(self._nodes)):
                    break
                time.sleep(0.01)
            if not ok() and not self._stopped:
                raise ExternalShutdownException()
        finally:
            self.shutdown()

    def _worker(self):
        while ok() and not self._stopped:
            item = _pop_any_callback(self._nodes, 0.01, self._stopped)
            if item:
                cb, msg, _node = item
                _invoke_callback(cb, msg)

    def shutdown(self):
        self._stopped = True
        for t in self._threads:
            t.join(timeout=0.1)
        self._threads.clear()


def spin(node, executor: Executor | None = None):
    if executor is None:
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.remove_node(node)
            executor.shutdown()
    else:
        added = False
        if node not in getattr(executor, "_nodes", []):
            executor.add_node(node)
            added = True
        try:
            executor.spin()
        finally:
            if added:
                executor.remove_node(node)


def spin_once(node, timeout_sec: float | None = None):
    _run_callbacks_for_node(node)
    if timeout_sec:
        time.sleep(min(timeout_sec, 0.001))


def spin_some(node, timeout_sec: float | None = None):
    _run_callbacks_for_node(node)
    if timeout_sec:
        time.sleep(min(timeout_sec, 0.001))


def spin_until_future_complete(node, future, timeout_sec: float | None = None, *, executor: Executor | None = None):
    start = time.monotonic()
    own_executor = executor is None
    exec_obj = executor or SingleThreadedExecutor()
    added = False
    if node not in getattr(exec_obj, "_nodes", []):
        exec_obj.add_node(node)
        added = True
    try:
        while ok():
            if future.done():
                return True
            exec_obj.spin_once(0.01)
            if timeout_sec is not None and (time.monotonic() - start) >= timeout_sec:
                return False
    finally:
        if added:
            exec_obj.remove_node(node)
        if own_executor:
            exec_obj.shutdown()
    return False


def _run_callbacks_for_node(node):
    try:
        callbacks = node._drain_callbacks()
    except Exception:
        callbacks = []
    for cb, msg in callbacks:
        _invoke_callback(cb, msg)


def _invoke_callback(cb, msg):
    try:
        result = cb(msg) if msg is not None else cb()
        if inspect.iscoroutine(result):
            asyncio.run(result)
    except Exception:
        pass


def _pop_any_callback(nodes: Iterable, timeout_sec: float | None, stopped: bool):
    start = time.monotonic()
    while ok() and not stopped:
        for node in list(nodes):
            try:
                item = node._pop_callback()
            except Exception:
                item = None
            if item:
                cb, msg = item
                return (cb, msg, node)
        if timeout_sec is None:
            time.sleep(0.001)
            continue
        elapsed = time.monotonic() - start
        if elapsed >= max(timeout_sec, 0):
            return None
        time.sleep(0.001)
    return None

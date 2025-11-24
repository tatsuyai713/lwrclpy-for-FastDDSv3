import threading
import time
from typing import Iterable
from .context import ok, shutdown


class SingleThreadedExecutor:
    """Minimal executor that mirrors rclpy interface surface."""

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
            self.spin_some(0.01)

    def spin_once(self, timeout_sec: float | None = None):
        cb_pair = self._pop_any_callback(self._nodes, timeout_sec)
        if cb_pair:
            cb, msg = cb_pair
            try:
                cb(msg) if msg is not None else cb()
            except Exception:
                pass

    def spin_some(self, timeout_sec: float | None = None):
        if not ok() or self._stopped:
            return
        self._process_all_ready(self._nodes)
        if timeout_sec is not None and timeout_sec > 0:
            time.sleep(min(timeout_sec, 0.001))

    def shutdown(self):
        self._stopped = True
        try:
            shutdown()
        except Exception:
            pass

    def _process_ready(self, nodes: Iterable):
        for node in list(nodes):
            self._process_all_ready([node])

    def _process_all_ready(self, nodes: Iterable):
        for node in list(nodes):
            while True:
                item = None
                try:
                    item = node._pop_callback()
                except Exception:
                    item = None
                if not item:
                    break
                cb, msg = item
                try:
                    cb(msg) if msg is not None else cb()
                except Exception:
                    pass

    def _pop_any_callback(self, nodes: Iterable, timeout_sec: float | None):
        start = time.monotonic()
        while ok() and not self._stopped:
            for node in list(nodes):
                try:
                    item = node._pop_callback()
                except Exception:
                    item = None
                if item:
                    return item
            if timeout_sec is None:
                time.sleep(0.001)
                continue
            elapsed = time.monotonic() - start
            if elapsed >= max(timeout_sec, 0):
                return None
            time.sleep(0.001)
        return None


def spin(node):
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.spin()


def spin_once(node, timeout_sec: float | None = None):
    # Backward compat; delegates to spin_some
    spin_some(node, timeout_sec)


def spin_some(node, timeout_sec: float | None = None):
    _run_callbacks_for_node(node)
    if timeout_sec is not None and timeout_sec > 0:
        time.sleep(min(timeout_sec, 0.001))


def _run_callbacks_for_node(node):
    try:
        callbacks = node._drain_callbacks()
    except Exception:
        callbacks = []
    for cb, msg in callbacks:
        try:
            cb(msg) if msg is not None else cb()
        except Exception:
            pass


class MultiThreadedExecutor(SingleThreadedExecutor):
    """Run each node in its own thread by delegating to rclpy.spin(node)."""

    def __init__(self):
        super().__init__()
        self._threads: list[threading.Thread] = []

    def _spin_node(self, node):
        # Process callbacks for this node only
        while ok() and not self._stopped:
            self._process_all_ready([node])
            time.sleep(0.001)

    def spin(self):
        if self._stopped:
            return
        for _node in self._nodes:
            t = threading.Thread(target=self._spin_node, args=(_node,), daemon=True)
            t.start()
            self._threads.append(t)
        try:
            # Monitor for idle: stop when all nodes have no pending work (timers canceled and queues empty)
            while ok() and not self._stopped:
                if not any(getattr(n, "_has_pending_work", lambda: True)() for n in list(self._nodes)):
                    self._stopped = True
                    break
                time.sleep(0.01)
            for t in self._threads:
                t.join()
        finally:
            self._threads.clear()

    def spin_some(self, timeout_sec: float | None = None):
        # For multithreaded executor, spin_some processes callbacks once per node
        if not ok() or self._stopped:
            return
        self._process_ready(self._nodes)
        time.sleep(0 if timeout_sec is None else max(timeout_sec, 0))

    def shutdown(self):
        self._stopped = True
        for t in self._threads:
            t.join(timeout=0.1)
        self._threads.clear()
        try:
            shutdown()
        except Exception:
            pass

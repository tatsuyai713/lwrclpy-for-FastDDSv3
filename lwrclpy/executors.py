import time
from .context import ok

class SingleThreadedExecutor:
    def __init__(self):
        self._nodes = []

    def add_node(self, node):
        self._nodes.append(node)

    def spin(self):
        while ok():
            time.sleep(0.01)


def spin(node):
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.spin()


def spin_once(node, timeout_sec: float | None = None):
    time.sleep(0 if timeout_sec is None else max(timeout_sec, 0))
import threading
import time

class _RepeatingTimer:
    def __init__(self, period_sec: float, callback):
        self._period = period_sec
        self._callback = callback
        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thr.start()

    def _run(self):
        next_t = time.monotonic() + self._period
        while not self._stop.is_set():
            now = time.monotonic()
            sleep = next_t - now
            if sleep > 0:
                self._stop.wait(sleep)
            if self._stop.is_set():
                break
            try:
                self._callback()
            finally:
                next_t += self._period

    def cancel(self):
        self._stop.set()
        self._thr.join(timeout=1)


def create_timer(period_sec: float, callback):
    t = _RepeatingTimer(period_sec, callback)
    t.start()
    return t
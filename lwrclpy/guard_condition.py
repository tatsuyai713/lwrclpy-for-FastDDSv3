class GuardCondition:
    """Lightweight guard condition that enqueues a callback when triggered."""

    def __init__(self, callback, enqueue_cb):
        self._callback = callback
        self._enqueue_cb = enqueue_cb
        self._destroyed = False

    def trigger(self):
        if self._destroyed:
            return
        self._enqueue_cb(self._callback, None)

    def destroy(self):
        self._destroyed = True

# lwrclpy/subscription.py
# Zero-copy–friendly DataReader wrapper for Fast DDS v3.
# - Prefer DDS internal zero-copy (DataSharing) where available.
# - Bind listener either via set_listener() or at creation time (API differences).
# - Absorb v2/v3 return code differences.

from __future__ import annotations
import fastdds  # type: ignore
from .qos import QoSProfile


def _retcode_is_ok(rc) -> bool:
    """Return True if 'rc' represents RETCODE_OK (v2/v3 tolerant)."""
    ok_const = getattr(fastdds, "RETCODE_OK", 0)  # v3: module-level; v2: often 0-like
    try:
        if rc == ok_const:
            return True
    except Exception:
        pass
    try:
        return int(rc) == int(ok_const)
    except Exception:
        return bool(rc) is True


def _force_data_sharing_on_reader(rq: "fastdds.DataReaderQos") -> None:
    """Prefer/force data sharing on the reader QoS when the API exists."""
    try:
        if hasattr(rq, "data_sharing"):
            ds = rq.data_sharing()
            # Prefer explicit ON for internal zero-copy; fallback to automatic.
            if hasattr(ds, "on"):
                ds.on()  # Force data-sharing if supported
            elif hasattr(ds, "automatic"):
                ds.automatic()
    except Exception:
        # Silently ignore if this build doesn't expose the API
        pass


class _ReaderListener(fastdds.DataReaderListener):
    """Simple listener that converts 'on_data_available' into Python callback."""

    def __init__(self, on_msg, msg_ctor):
        super().__init__()
        self._on_msg = on_msg
        self._msg_ctor = msg_ctor

    def on_data_available(self, reader):
        info = fastdds.SampleInfo()
        data = self._msg_ctor()

        # Absorb ordering & signature differences.
        try:
            rc = reader.take_next_sample(data, info)
        except TypeError:
            rc = reader.take_next_sample(info, data)
        except Exception:
            return  # Avoid director double-fault

        if _retcode_is_ok(rc) and getattr(info, "valid_data", True):
            try:
                self._on_msg(data)
            except Exception:
                # Swallow user exceptions to avoid aborting DDS thread.
                return


class Subscription:
    """Subscription managing Subscriber/DataReader with zero-copy friendly QoS."""

    def __init__(self, participant, topic, qos: QoSProfile, callback, msg_ctor):
        self._participant = participant
        self._topic = topic
        self._callback = callback
        self._msg_ctor = msg_ctor

        # Create Subscriber
        sub_qos = fastdds.SubscriberQos()
        participant.get_default_subscriber_qos(sub_qos)
        self._subscriber = participant.create_subscriber(sub_qos)
        if self._subscriber is None:
            raise RuntimeError("Failed to create Subscriber")

        # Prepare Reader QoS (map from high-level QoSProfile first)
        rq = fastdds.DataReaderQos()
        self._subscriber.get_default_datareader_qos(rq)
        qos.apply_to_reader(rq)

        # >>> Zero-copy–friendly hint: prefer DataSharing if the API exists
        _force_data_sharing_on_reader(rq)
        # <<<

        # Listener
        self._listener = _ReaderListener(callback, msg_ctor)

        # Create DataReader and attach listener (API differs across bindings)
        reader = None
        try:
            # Newer API often supports passing the listener on creation.
            reader = self._subscriber.create_datareader(self._topic, rq, self._listener)
        except TypeError:
            # Fallback to creation without listener, then set it.
            reader = self._subscriber.create_datareader(self._topic, rq)
            try:
                reader.set_listener(self._listener)
            except AttributeError:
                # Some very old bindings may not expose set_listener at all.
                # In that case the reader will work without callbacks (polling would be needed).
                pass

        if reader is None:
            raise RuntimeError("Failed to create DataReader")
        self._reader = reader

    def destroy(self) -> None:
        """Tear down DataReader/Subscriber in the right order, swallowing errors."""
        if getattr(self, "_reader", None):
            try:
                self._subscriber.delete_datareader(self._reader)
            except Exception:
                pass
            self._reader = None
        if getattr(self, "_subscriber", None):
            try:
                self._participant.delete_subscriber(self._subscriber)
            except Exception:
                pass
            self._subscriber = None

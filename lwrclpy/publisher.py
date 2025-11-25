# lwrclpy/publisher.py
# Zero-copy–friendly DataWriter wrapper for Fast DDS v3.
# - Prefer DDS internal zero-copy (DataSharing) where available.
# - Keep compatibility with QoSProfile mapping.

from __future__ import annotations
import fastdds  # type: ignore
from .qos import QoSProfile
from .message_utils import clone_message
from .duration import Duration


def _force_data_sharing_on_writer(wq: "fastdds.DataWriterQos") -> None:
    """Prefer/force data sharing on the writer QoS when the API exists."""
    try:
        if hasattr(wq, "data_sharing"):
            ds = wq.data_sharing()
            # Prefer explicit ON (or automatic when ON is unavailable).
            if hasattr(ds, "on"):
                ds.on()
            elif hasattr(ds, "automatic"):
                ds.automatic()
    except Exception:
        # Ignore when API is absent
        pass


class Publisher:
    """Publisher managing Publisher/DataWriter with zero-copy friendly QoS."""

    def __init__(self, participant, topic, qos: QoSProfile, msg_ctor=None):
        self._participant = participant
        self._topic = topic
        self._msg_ctor = msg_ctor

        # Create Publisher
        pub_qos = fastdds.PublisherQos()
        participant.get_default_publisher_qos(pub_qos)
        self._publisher = participant.create_publisher(pub_qos)
        if self._publisher is None:
            raise RuntimeError("Failed to create Publisher")

        # Prepare Writer QoS (map from high-level QoSProfile first)
        wq = fastdds.DataWriterQos()
        self._publisher.get_default_datawriter_qos(wq)
        qos.apply_to_writer(wq)

        # >>> Zero-copy–friendly hint: prefer DataSharing if the API exists
        _force_data_sharing_on_writer(wq)
        # <<<

        # Create DataWriter
        self._writer = self._publisher.create_datawriter(self._topic, wq)
        if self._writer is None:
            raise RuntimeError("Failed to create DataWriter")

    def publish(self, msg) -> None:
        """Publish a message instance generated from the SWIG type."""
        to_send = msg
        try:
            target_ctor = self._msg_ctor if self._msg_ctor is not None and isinstance(msg, self._msg_ctor) else msg.__class__
            to_send = clone_message(msg, target_ctor)
        except Exception:
            to_send = msg  # fall back to original on failure
        self._writer.write(to_send)

    def wait_for_all_acked(self, timeout: Duration | float | int | None = None) -> bool:
        """Block until all samples are acknowledged or timeout occurs."""
        duration = fastdds.Duration_t()
        if timeout is None:
            duration.seconds = getattr(fastdds, "DURATION_INFINITE_SEC", 0x7fffffff)
            duration.nanosec = getattr(fastdds, "DURATION_INFINITE_NSEC", 0x7fffffff)
        else:
            if isinstance(timeout, Duration):
                total_ns = timeout.nanoseconds
            else:
                total_ns = int(float(timeout) * 1_000_000_000)
            if total_ns < 0:
                total_ns = 0
            duration.seconds = total_ns // 1_000_000_000
            duration.nanosec = total_ns % 1_000_000_000
        try:
            rc = self._writer.wait_for_acknowledgments(duration)
            return bool(rc) if isinstance(rc, bool) else True
        except Exception:
            return False

    def destroy(self) -> None:
        """Tear down DataWriter/Publisher in the right order, swallowing errors."""
        if getattr(self, "_writer", None):
            try:
                self._publisher.delete_datawriter(self._writer)
            except Exception:
                pass
            self._writer = None
        if getattr(self, "_publisher", None):
            try:
                self._participant.delete_publisher(self._publisher)
            except Exception:
                pass
            self._publisher = None

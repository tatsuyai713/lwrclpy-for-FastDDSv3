import fastdds
from .qos import QoSProfile

class Publisher:
    def __init__(self, participant, topic, qos: QoSProfile):
        pqos = fastdds.PublisherQos()
        participant.get_default_publisher_qos(pqos)
        self._participant = participant
        self._publisher = participant.create_publisher(pqos)
        wq = fastdds.DataWriterQos()
        self._publisher.get_default_datawriter_qos(wq)
        qos.apply_to_writer(wq)
        self._writer = self._publisher.create_datawriter(topic, wq)

    def publish(self, msg):
        self._writer.write(msg)

    def destroy(self):
        if self._writer:
            self._publisher.delete_datawriter(self._writer)
            self._writer = None
        if self._publisher:
            self._participant.delete_publisher(self._publisher)
            self._publisher = None
from typing import Iterable
from src.navigation.state import SensorPacket

class DatasetReplay:
    """
    Replay interface for injecting IO-VNBD sensor packets chronologically.
    """
    def __init__(self, data_source: Iterable[SensorPacket]):
        self._source = data_source
        self._iterator = iter(self._source)

    def next_packet(self) -> SensorPacket:
        """
        Yields the next chronological sensor packet.
        """
        try:
            return next(self._iterator)
        except StopIteration:
            return None

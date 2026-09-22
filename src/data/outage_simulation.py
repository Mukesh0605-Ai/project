import typing
import copy
from src.navigation.state import SensorPacket

class GNSSOutageSimulator:
    """
    Simulates GNSS outages / blackout tunnels over sensor packet streams.
    Masks GNSS measurements (position, speed, accuracy) during the outage interval
    while preserving ground truth metrics in metadata.
    """
    def __init__(self):
        pass

    def simulate_outage(self, packets: typing.Optional[typing.List[SensorPacket]] = None, start_time: float = 0.0, duration: float = 0.0, gnss_data: typing.Optional[typing.Any] = None) -> typing.List[SensorPacket]:
        stream = packets if packets is not None else (gnss_data if isinstance(gnss_data, list) else [])
        end_time = start_time + duration
        modified_packets = []

        for p in stream:
            p_copy = SensorPacket(
                timestamp=p.timestamp,
                accelerometer=p.accelerometer.copy() if p.accelerometer is not None else None,
                gyroscope=p.gyroscope.copy() if p.gyroscope is not None else None,
                magnetometer=p.magnetometer.copy() if p.magnetometer is not None else None,
                gnss_lat_lon_alt=p.gnss_lat_lon_alt.copy() if p.gnss_lat_lon_alt is not None else None,
                gnss_speed=p.gnss_speed,
                gnss_accuracy=p.gnss_accuracy,
                metadata=copy.deepcopy(p.metadata) if p.metadata else {}
            )

            # Preserve ground truth in metadata if not already populated
            if "ground_truth_speed" not in p_copy.metadata:
                p_copy.metadata["ground_truth_speed"] = p.gnss_speed
            if "ground_truth_enu" not in p_copy.metadata:
                p_copy.metadata["ground_truth_enu"] = p_copy.metadata.get("gnss_enu")

            # Mask GNSS if timestamp falls inside outage interval
            if start_time <= p.timestamp <= end_time:
                p_copy.gnss_lat_lon_alt = None
                p_copy.gnss_speed = None
                p_copy.gnss_accuracy = None
                p_copy.metadata["gnss_enu"] = None
                p_copy.metadata["is_outage"] = True
            else:
                p_copy.metadata["is_outage"] = False

            modified_packets.append(p_copy)

        return modified_packets


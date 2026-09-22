import abc
import numpy as np
import typing
from src.navigation.state import SensorPacket

class BaseSensorAdapter(abc.ABC):
    """
    Abstract Base Class for Sensor-Agnostic Adapters.
    Defines common interface for converting raw smartphone or external high-rate
    IMU/GNSS payloads into standardized SensorPackets, and providing decimation factors
    for multi-rate EKF (up to 200 Hz) and AI/Map Matching (10 Hz).
    """

    @abc.abstractmethod
    def get_imu_sampling_rate(self) -> float:
        """Returns target IMU sampling frequency in Hz."""
        pass

    @abc.abstractmethod
    def process_raw_sample(self, raw_data: typing.Any) -> SensorPacket:
        """Converts raw sensor payload into a standardized SensorPacket."""
        pass

    def get_ai_subsampling_factor(self) -> int:
        """
        Calculates downsampling decimation stride for 10 Hz AI sliding window.
        Example: 200 Hz IMU / 10 Hz AI = 20 stride.
        """
        rate = self.get_imu_sampling_rate()
        return max(1, int(round(rate / 10.0)))

    def get_map_matching_subsampling_factor(self) -> int:
        """
        Calculates decimation stride for 10 Hz HMM Map Matching execution.
        Example: 200 Hz IMU / 10 Hz Map = 20 stride.
        """
        rate = self.get_imu_sampling_rate()
        return max(1, int(round(rate / 10.0)))


class SmartphoneSensorAdapter(BaseSensorAdapter):
    """
    Adapter for standard Smartphone sensors (Android/iOS).
    Default IMU sampling rate: 100 Hz (dt = 0.01 s).
    Default GNSS sampling rate: 1 Hz.
    """

    def __init__(self, target_rate_hz: float = 100.0):
        self.target_rate_hz = target_rate_hz

    def get_imu_sampling_rate(self) -> float:
        return self.target_rate_hz

    def process_raw_sample(self, raw_data: typing.Any) -> SensorPacket:
        """
        Processes smartphone sensor payload dict or tuple.
        Expected format dict:
        {
            "timestamp": float,
            "accel": [ax, ay, az],
            "gyro": [gx, gy, gz],
            "mag": [mx, my, mz] (optional),
            "gnss": [lat, lon, alt, acc] (optional),
            "gnss_speed": float (optional),
            "gnss_accuracy": float (optional)
        }
        """
        if isinstance(raw_data, SensorPacket):
            return raw_data

        if isinstance(raw_data, dict):
            ts = float(raw_data.get("timestamp", 0.0))
            acc = np.array(raw_data.get("accel", [0.0, 0.0, 9.81]), dtype=float)
            gyr = np.array(raw_data.get("gyro", [0.0, 0.0, 0.0]), dtype=float)
            mag = np.array(raw_data["mag"], dtype=float) if "mag" in raw_data else None
            gnss_lat_lon = np.array(raw_data["gnss"], dtype=float) if "gnss" in raw_data else None
            gnss_speed = float(raw_data["gnss_speed"]) if "gnss_speed" in raw_data else None
            gnss_acc = float(raw_data["gnss_accuracy"]) if "gnss_accuracy" in raw_data else None
            meta = raw_data.get("metadata", {})

            return SensorPacket(
                timestamp=ts,
                accelerometer=acc,
                gyroscope=gyr,
                magnetometer=mag,
                gnss_lat_lon_alt=gnss_lat_lon,
                gnss_speed=gnss_speed,
                gnss_accuracy=gnss_acc,
                metadata=meta
            )

        raise ValueError(f"Unsupported smartphone raw data type: {type(raw_data)}")


class ExternalHighRateIMUAdapter(BaseSensorAdapter):
    """
    Adapter for External High-Rate IMUs (e.g. 200 Hz industrial MEMS / FOG simulation).
    Propagates state at 200 Hz while decimating AI inference & Map Matching to 10 Hz.
    """

    def __init__(self, target_rate_hz: float = 200.0):
        self.target_rate_hz = target_rate_hz

    def get_imu_sampling_rate(self) -> float:
        return self.target_rate_hz

    def process_raw_sample(self, raw_data: typing.Any) -> SensorPacket:
        """
        Processes external high-rate IMU payload (dict, tuple, or binary packet struct).
        """
        if isinstance(raw_data, SensorPacket):
            return raw_data

        if isinstance(raw_data, dict):
            ts = float(raw_data.get("timestamp", 0.0))
            acc = np.array(raw_data.get("accel", [0.0, 0.0, 9.81]), dtype=float)
            gyr = np.array(raw_data.get("gyro", [0.0, 0.0, 0.0]), dtype=float)
            mag = np.array(raw_data["mag"], dtype=float) if "mag" in raw_data else None
            gnss_lat_lon = np.array(raw_data["gnss"], dtype=float) if "gnss" in raw_data else None
            gnss_speed = float(raw_data["gnss_speed"]) if "gnss_speed" in raw_data else None
            gnss_acc = float(raw_data["gnss_accuracy"]) if "gnss_accuracy" in raw_data else None
            meta = raw_data.get("metadata", {})
            meta["source"] = "EXTERNAL_200HZ_IMU"

            return SensorPacket(
                timestamp=ts,
                accelerometer=acc,
                gyroscope=gyr,
                magnetometer=mag,
                gnss_lat_lon_alt=gnss_lat_lon,
                gnss_speed=gnss_speed,
                gnss_accuracy=gnss_acc,
                metadata=meta
            )

        if isinstance(raw_data, (tuple, list)):
            # Format: (timestamp, accel_3d, gyro_3d)
            ts = float(raw_data[0])
            acc = np.array(raw_data[1], dtype=float)
            gyr = np.array(raw_data[2], dtype=float)
            return SensorPacket(
                timestamp=ts,
                accelerometer=acc,
                gyroscope=gyr,
                metadata={"source": "EXTERNAL_200HZ_IMU"}
            )

        raise ValueError(f"Unsupported external high-rate raw data type: {type(raw_data)}")

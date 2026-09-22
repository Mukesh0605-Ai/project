import typing
import numpy as np
from dataclasses import dataclass, field
from src.navigation.state import SensorPacket

@dataclass
class SynchronizedStreamOutput:
    """Output container for dual-rate synchronized sensor streams."""
    high_rate_imu_stream: typing.List[SensorPacket]  # e.g., 100 Hz for EKF propagation
    uniform_ai_stream: typing.List[SensorPacket]      # e.g., 10 Hz for AI feature extraction

class SensorSynchronizer:
    """
    Dedicated Sensor Time Synchronization Module.
    Aligns asynchronous sensor streams (accelerometer, gyroscope, magnetometer, GNSS, odometry),
    handles sampling rate discrepancies, time jitter, missing samples, and duplicate timestamps.
    Enforces monotonic timestamps and generates dual synchronized streams (100Hz EKF + 10Hz AI)
    while strictly preserving hard GNSS outage intervals.
    """
    def __init__(self, 
                 imu_rate_hz: float = 100.0, 
                 ai_rate_hz: float = 10.0, 
                 max_gnss_gap_sec: float = 2.5):
        self.imu_rate_hz = imu_rate_hz
        self.ai_rate_hz = ai_rate_hz
        self.max_gnss_gap_sec = max_gnss_gap_sec

    def sanitize_and_sort_timestamps(self, packets: typing.List[SensorPacket]) -> typing.List[SensorPacket]:
        """
        Deduplicates identical timestamps and enforces strict monotonic timestamp ordering.
        """
        if not packets:
            return []

        # Sort chronologically by timestamp
        sorted_packets = sorted(packets, key=lambda p: p.timestamp)

        # Deduplicate identical timestamps (retain last packet for duplicate time)
        deduped = []
        for p in sorted_packets:
            if not deduped:
                deduped.append(p)
            else:
                if math_is_close(p.timestamp, deduped[-1].timestamp):
                    # Replace duplicate with latest packet
                    deduped[-1] = p
                elif p.timestamp > deduped[-1].timestamp:
                    deduped.append(p)

        return deduped

    def interpolate_vector(self, t_target: float, times: np.ndarray, vectors: np.ndarray) -> np.ndarray:
        """
        Performs 1D linear interpolation for continuous 3D vector signals (accel, gyro, mag).
        """
        if len(times) == 0:
            return np.zeros(3)
        if len(times) == 1:
            return vectors[0].copy()

        # Clamp to bounds
        if t_target <= times[0]:
            return vectors[0].copy()
        if t_target >= times[-1]:
            return vectors[-1].copy()

        idx = np.searchsorted(times, t_target)
        t0, t1 = times[idx - 1], times[idx]
        v0, v1 = vectors[idx - 1], vectors[idx]

        if t1 == t0:
            return v0.copy()

        alpha = (t_target - t0) / (t1 - t0)
        return v0 + alpha * (v1 - v0)

    def synchronize_stream(self, raw_packets: typing.List[SensorPacket]) -> SynchronizedStreamOutput:
        """
        Ingests a stream of raw asynchronous SensorPackets and generates synchronized dual-rate streams:
        1. High-rate IMU stream (100 Hz) for EKF propagation.
        2. Uniform AI stream (10 Hz) for sliding window velocity prediction.
        
        Preserves GNSS outages: During blackout gaps, GNSS remains None.
        """
        clean_packets = self.sanitize_and_sort_timestamps(raw_packets)
        if not clean_packets:
            return SynchronizedStreamOutput(high_rate_imu_stream=[], uniform_ai_stream=[])

        orig_times = np.array([p.timestamp for p in clean_packets])
        t_start = clean_packets[0].timestamp
        t_end = clean_packets[-1].timestamp

        # Extract continuous vectors for interpolation
        accel_vecs = np.array([p.accelerometer if p.accelerometer is not None else np.zeros(3) for p in clean_packets])
        gyro_vecs = np.array([p.gyroscope if p.gyroscope is not None else np.zeros(3) for p in clean_packets])
        mag_vecs = np.array([p.magnetometer if p.magnetometer is not None else np.zeros(3) for p in clean_packets])

        # 1. High-Rate IMU Stream Generation (100 Hz)
        dt_imu = 1.0 / self.imu_rate_hz
        high_rate_timestamps = np.arange(t_start, t_end + 1e-6, dt_imu)
        high_rate_packets = []

        for t_high in high_rate_timestamps:
            acc_interp = self.interpolate_vector(t_high, orig_times, accel_vecs)
            gyr_interp = self.interpolate_vector(t_high, orig_times, gyro_vecs)
            mag_interp = self.interpolate_vector(t_high, orig_times, mag_vecs)

            # Find nearest original packet for GNSS / metadata status
            idx_near = min(max(0, np.searchsorted(orig_times, t_high)), len(clean_packets) - 1)
            near_p = clean_packets[idx_near]

            # Outage Protection: If nearest packet is inside outage gap (GNSS is None), preserve None!
            has_gnss_fix = (near_p.gnss_lat_lon_alt is not None) and (abs(t_high - near_p.timestamp) < self.max_gnss_gap_sec)

            high_p = SensorPacket(
                timestamp=t_high,
                accelerometer=acc_interp,
                gyroscope=gyr_interp,
                magnetometer=mag_interp,
                gnss_lat_lon_alt=near_p.gnss_lat_lon_alt.copy() if has_gnss_fix else None,
                gnss_speed=near_p.gnss_speed if has_gnss_fix else None,
                gnss_accuracy=near_p.gnss_accuracy if has_gnss_fix else None,
                metadata=near_p.metadata.copy() if near_p.metadata else {}
            )
            high_rate_packets.append(high_p)

        # 2. Uniform AI Feature Stream Generation (10 Hz)
        dt_ai = 1.0 / self.ai_rate_hz
        ai_timestamps = np.arange(t_start, t_end + 1e-6, dt_ai)
        ai_packets = []

        for t_ai in ai_timestamps:
            acc_interp = self.interpolate_vector(t_ai, orig_times, accel_vecs)
            gyr_interp = self.interpolate_vector(t_ai, orig_times, gyro_vecs)
            mag_interp = self.interpolate_vector(t_ai, orig_times, mag_vecs)

            idx_near = min(max(0, np.searchsorted(orig_times, t_ai)), len(clean_packets) - 1)
            near_p = clean_packets[idx_near]

            has_gnss_fix = (near_p.gnss_lat_lon_alt is not None) and (abs(t_ai - near_p.timestamp) < self.max_gnss_gap_sec)

            ai_p = SensorPacket(
                timestamp=t_ai,
                accelerometer=acc_interp,
                gyroscope=gyr_interp,
                magnetometer=mag_interp,
                gnss_lat_lon_alt=near_p.gnss_lat_lon_alt.copy() if has_gnss_fix else None,
                gnss_speed=near_p.gnss_speed if has_gnss_fix else None,
                gnss_accuracy=near_p.gnss_accuracy if has_gnss_fix else None,
                metadata=near_p.metadata.copy() if near_p.metadata else {}
            )
            ai_packets.append(ai_p)

        return SynchronizedStreamOutput(
            high_rate_imu_stream=high_rate_packets,
            uniform_ai_stream=ai_packets
        )

def math_is_close(a: float, b: float, rel_tol: float = 1e-7, abs_tol: float = 1e-9) -> bool:
    """Helper function to test float equality."""
    return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

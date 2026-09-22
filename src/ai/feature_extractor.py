import numpy as np
import typing
from src.navigation.state import SensorPacket

class SIHFeatureExtractor:
    """
    SIH Feature Extractor & Explicit 10 Hz Resampler.
    Processes raw sensor streams (typically 100 Hz) into a 2-second window 
    containing 20 samples at 10 Hz with 19 rich inertial & orientation features.
    
    Features per sample (19 total):
      1-3:  Accelerometer [ax, ay, az] (m/s^2)
      4-6:  Gyroscope [gx, gy, gz] (rad/s)
      7-9:  Quality-Gated Magnetometer [mx, my, mz] (uT)
      10-12: Gravity-Compensated Acceleration [ax_g, ay_g, az_g] (m/s^2)
      13-15: Jerk [jx, jy, jz] (m/s^3)
      16-19: Spectral & Orientation Features:
             - Accel norm spectral power
             - Dominant spectral peak frequency estimate
             - Estimated Pitch angle (rad)
             - Estimated Roll angle (rad)
    """
    def __init__(self, target_hz: float = 10.0, window_duration_sec: float = 2.0):
        self.target_hz = target_hz
        self.window_duration_sec = window_duration_sec
        self.num_samples = int(target_hz * window_duration_sec)  # 20 samples
        self.expected_feature_dim = 19

    def extract_features_from_raw(self,
                                  accel: np.ndarray,
                                  gyro: np.ndarray,
                                  mag: typing.Optional[np.ndarray] = None,
                                  dt: float = 0.01) -> np.ndarray:
        """
        Extracts a single 19-dimensional feature vector from instantaneous IMU reading.
        """
        # 1. Base accel & gyro
        a = np.array(accel, dtype=np.float32)
        g = np.array(gyro, dtype=np.float32)

        # 2. Quality-Gated Magnetometer
        if mag is not None:
            m_raw = np.array(mag, dtype=np.float32)
            m_norm = float(np.linalg.norm(m_raw))
            # Gate: Earth field is ~20 to 70 uT. Outside this range => magnetic perturbation
            if 20.0 <= m_norm <= 70.0:
                m_gated = m_raw
            else:
                m_gated = m_raw * 0.1  # Down-weight disturbed magnetic field
        else:
            m_gated = np.zeros(3, dtype=np.float32)

        # 3. Gravity-Compensated Acceleration (approximate gravity along Z or pitch/roll)
        a_norm = float(np.linalg.norm(a))
        if a_norm > 1e-3:
            gravity_vector = np.array([0.0, 0.0, 9.81], dtype=np.float32)
            a_grav_comp = a - gravity_vector
        else:
            a_grav_comp = np.zeros(3, dtype=np.float32)

        # 4. Jerk (approximate using accel magnitude or instant rate)
        jerk = a_grav_comp / max(1e-4, dt)

        # 5. Spectral & Orientation Features
        pitch = float(np.arctan2(-a[0], np.sqrt(a[1]**2 + a[2]**2)))
        roll = float(np.arctan2(a[1], a[2]))
        spectral_power = float(a_norm**2)
        dominant_freq = float(np.linalg.norm(g) / (2.0 * np.pi))  # Gyro rotation frequency estimate

        spec_orient = np.array([spectral_power, dominant_freq, pitch, roll], dtype=np.float32)

        feature_vec = np.concatenate([a, g, m_gated, a_grav_comp, jerk, spec_orient])
        return feature_vec

    def resample_and_extract_window(self, 
                                    high_rate_buffer: typing.List[SensorPacket], 
                                    target_window_sec: float = 2.0) -> np.ndarray:
        """
        Resamples a high-rate IMU buffer (e.g. 100 Hz, ~200 packets) down to 
        exactly 20 samples at 10 Hz with 19 feature channels.
        Returns array of shape (20, 19).
        """
        if not high_rate_buffer:
            return np.zeros((self.num_samples, self.expected_feature_dim), dtype=np.float32)

        # Extract timestamps and feature vectors for all high-rate packets
        timestamps = np.array([p.timestamp for p in high_rate_buffer], dtype=np.float64)
        t_start = timestamps[0]
        t_end = timestamps[-1]

        # Extract features for all raw packets
        raw_features = []
        for i, p in enumerate(high_rate_buffer):
            dt = (timestamps[i] - timestamps[i-1]) if i > 0 else 0.01
            dt = max(1e-4, dt)
            acc = p.accelerometer if p.accelerometer is not None else np.zeros(3)
            gyr = p.gyroscope if p.gyroscope is not None else np.zeros(3)
            mag = p.magnetometer
            feat = self.extract_features_from_raw(acc, gyr, mag, dt=dt)
            raw_features.append(feat)

        raw_features = np.array(raw_features, dtype=np.float32)  # (N_raw, 19)

        if len(timestamps) < 2 or (t_end - t_start) <= 1e-4:
            # Duplicate or replicate single sample across 20 slots
            return np.tile(raw_features[0], (self.num_samples, 1))

        # Define 10 Hz target grid (20 uniform timestamps over 2-second window)
        target_grid = np.linspace(t_start, t_end, self.num_samples)

        # Interpolate each feature channel across the 10 Hz grid
        resampled_window = np.zeros((self.num_samples, self.expected_feature_dim), dtype=np.float32)
        for c in range(self.expected_feature_dim):
            resampled_window[:, c] = np.interp(target_grid, timestamps, raw_features[:, c])

        return resampled_window

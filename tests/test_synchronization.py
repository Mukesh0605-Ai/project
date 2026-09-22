import unittest
import numpy as np
from src.data.synchronization import SensorSynchronizer, SynchronizedStreamOutput
from src.navigation.state import SensorPacket, NavigationState
from src.navigation.engine import NavigationEngine

class TestSensorSynchronization(unittest.TestCase):

    def setUp(self):
        self.synchronizer = SensorSynchronizer(imu_rate_hz=100.0, ai_rate_hz=10.0)

    def test_deduplication_and_monotonic_sorting(self):
        # Create un-sorted packets with duplicate timestamps
        p1 = SensorPacket(timestamp=0.10, accelerometer=np.array([0.0, 0.0, 9.8]))
        p2 = SensorPacket(timestamp=0.05, accelerometer=np.array([0.1, 0.0, 9.8]))
        p3 = SensorPacket(timestamp=0.10, accelerometer=np.array([0.2, 0.0, 9.8])) # Duplicate time
        p4 = SensorPacket(timestamp=0.20, accelerometer=np.array([0.3, 0.0, 9.8]))

        raw = [p1, p2, p3, p4]
        clean = self.synchronizer.sanitize_and_sort_timestamps(raw)

        # Should be sorted chronologically and deduplicated
        self.assertEqual(len(clean), 3)
        self.assertAlmostEqual(clean[0].timestamp, 0.05)
        self.assertAlmostEqual(clean[1].timestamp, 0.10)
        self.assertAlmostEqual(clean[2].timestamp, 0.20)
        # Duplicate should take the latest packet values (0.2)
        self.assertEqual(clean[1].accelerometer[0], 0.2)

    def test_vector_linear_interpolation(self):
        times = np.array([0.0, 1.0])
        vectors = np.array([
            [0.0, 0.0, 0.0],
            [10.0, 20.0, 30.0]
        ])

        # Interpolate at t = 0.5
        v_interp = self.synchronizer.interpolate_vector(0.5, times, vectors)
        np.testing.assert_allclose(v_interp, [5.0, 10.0, 15.0])

    def test_dual_rate_streams_output(self):
        # Generate 1.0 second of raw packets
        raw = []
        for t in np.linspace(0.0, 1.0, 25): # 25 Hz asynchronous
            p = SensorPacket(
                timestamp=t,
                accelerometer=np.array([0.0, 0.0, 9.80665]),
                gyroscope=np.zeros(3),
                gnss_lat_lon_alt=np.array([28.61, 77.22, 200.0]),
                gnss_speed=10.0
            )
            raw.append(p)

        synced = self.synchronizer.synchronize_stream(raw)
        self.assertIsInstance(synced, SynchronizedStreamOutput)

        # 100 Hz high rate stream over 1.0 second should have ~100 packets
        self.assertGreaterEqual(len(synced.high_rate_imu_stream), 90)
        self.assertLessEqual(len(synced.high_rate_imu_stream), 110)

        # 10 Hz AI feature stream over 1.0 second should have ~10 packets
        self.assertGreaterEqual(len(synced.uniform_ai_stream), 9)
        self.assertLessEqual(len(synced.uniform_ai_stream), 12)

    def test_gnss_outage_preservation(self):
        # Create a stream with a GNSS blackout tunnel between t = 0.4s and t = 0.8s
        raw = []
        for t in np.linspace(0.0, 1.2, 30):
            is_outage = (0.4 <= t <= 0.8)
            p = SensorPacket(
                timestamp=t,
                accelerometer=np.array([0.0, 0.0, 9.8]),
                gyroscope=np.zeros(3),
                gnss_lat_lon_alt=None if is_outage else np.array([28.61, 77.22, 200.0]),
                gnss_speed=None if is_outage else 12.0
            )
            raw.append(p)

        synced = self.synchronizer.synchronize_stream(raw)

        # Verify that packets inside outage zone (t = 0.5s) retain None for GNSS (NOT interpolated)
        outage_synced_packets = [p for p in synced.high_rate_imu_stream if 0.45 <= p.timestamp <= 0.75]
        self.assertGreater(len(outage_synced_packets), 0)

        for p in outage_synced_packets:
            self.assertIsNone(p.gnss_lat_lon_alt, f"GNSS was wrongly interpolated at t={p.timestamp}")
            self.assertIsNone(p.gnss_speed, f"GNSS speed was wrongly interpolated at t={p.timestamp}")

        # Verify that packets outside outage zone retain valid GNSS
        normal_synced_packets = [p for p in synced.high_rate_imu_stream if p.timestamp < 0.35]
        self.assertGreater(len(normal_synced_packets), 0)
        for p in normal_synced_packets:
            self.assertIsNotNone(p.gnss_lat_lon_alt)

    def test_navigation_engine_asynchronous_integration(self):
        engine = NavigationEngine()
        init_state = NavigationState(
            timestamp=0.0,
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3)
        )
        engine.initialize(init_state)

        # Generate un-synchronized raw packets with jitter
        raw_asynchronous = []
        times = [0.0, 0.033, 0.071, 0.071, 0.105, 0.142, 0.198, 0.250] # Includes duplicate 0.071
        for t in times:
            p = SensorPacket(
                timestamp=t,
                accelerometer=np.array([0.02, 0.1, 9.81]),
                gyroscope=np.zeros(3)
            )
            raw_asynchronous.append(p)

        states = engine.process_asynchronous_packets(raw_asynchronous)
        self.assertGreater(len(states), 0)
        self.assertIsNotNone(engine.get_state())

if __name__ == '__main__':
    unittest.main()

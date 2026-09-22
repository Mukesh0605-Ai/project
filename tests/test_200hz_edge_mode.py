import unittest
import numpy as np
from src.sensors.adapters import (
    SmartphoneSensorAdapter,
    ExternalHighRateIMUAdapter
)
from src.navigation.state import SensorPacket, NavigationState
from src.navigation.engine import NavigationEngine
from src.data.synthetic_200hz import generate_synthetic_200hz_trajectory, run_200hz_benchmark

class Test200HzEdgeMode(unittest.TestCase):

    def test_sensor_adapter_interfaces(self):
        # 1. Smartphone Adapter (100 Hz)
        phone_adapter = SmartphoneSensorAdapter(100.0)
        self.assertEqual(phone_adapter.get_imu_sampling_rate(), 100.0)
        self.assertEqual(phone_adapter.get_ai_subsampling_factor(), 10)
        self.assertEqual(phone_adapter.get_map_matching_subsampling_factor(), 10)

        raw_phone = {
            "timestamp": 1.0,
            "accel": [0.1, 0.2, 9.81],
            "gyro": [0.01, -0.02, 0.0],
            "gnss": [12.9716, 77.5946, 920.0, 1.2],
            "gnss_speed": 45.0,
            "gnss_accuracy": 1.5
        }
        packet_phone = phone_adapter.process_raw_sample(raw_phone)
        self.assertIsInstance(packet_phone, SensorPacket)
        self.assertEqual(packet_phone.timestamp, 1.0)
        self.assertAlmostEqual(packet_phone.gnss_accuracy, 1.5)

        # 2. External High-Rate IMU Adapter (200 Hz)
        ext_adapter = ExternalHighRateIMUAdapter(200.0)
        self.assertEqual(ext_adapter.get_imu_sampling_rate(), 200.0)
        self.assertEqual(ext_adapter.get_ai_subsampling_factor(), 20)
        self.assertEqual(ext_adapter.get_map_matching_subsampling_factor(), 20)

        raw_ext_tuple = (1.5, [0.05, -0.1, 9.80], [0.0, 0.005, -0.01])
        packet_ext = ext_adapter.process_raw_sample(raw_ext_tuple)
        self.assertIsInstance(packet_ext, SensorPacket)
        self.assertEqual(packet_ext.timestamp, 1.5)
        self.assertEqual(packet_ext.metadata["source"], "EXTERNAL_200HZ_IMU")

    def test_engine_200hz_propagation_and_decimation(self):
        timestamps, packets, ground_truth = generate_synthetic_200hz_trajectory(duration=5.0, dt=0.005)
        self.assertEqual(len(packets), 1000) # 5 seconds * 200 Hz = 1000 packets

        adapter = ExternalHighRateIMUAdapter(200.0)
        engine = NavigationEngine(adapter=adapter)

        init_state = NavigationState(
            timestamp=timestamps[0],
            position=ground_truth[0].copy(),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3),
            navigation_mode="INITIALIZING"
        )
        engine.initialize(init_state)

        states = []
        for p in packets:
            st = engine.process_packet(p)
            states.append(st)
            self.assertFalse(np.isnan(st.position).any())
            self.assertFalse(np.isnan(st.velocity).any())

        self.assertEqual(len(states), 1000)
        # Check metadata attributes set by adapter
        self.assertEqual(states[-1].metadata["adapter_type"], "ExternalHighRateIMUAdapter")
        self.assertEqual(states[-1].metadata["imu_sampling_rate_hz"], 200.0)

    def test_200hz_throughput_benchmark_run(self):
        res = run_200hz_benchmark()
        self.assertEqual(res["status"], "SUCCESS")
        self.assertEqual(res["imu_sampling_rate_hz"], 200.0)
        self.assertEqual(res["total_packets_processed"], 12000)
        self.assertGreater(res["throughput_packets_per_sec"], 200.0)  # Must achieve > 200 Hz real-time speed
        self.assertLess(res["latency_us_per_packet"], 5000.0)       # Must take < 5ms per packet

if __name__ == "__main__":
    unittest.main()

import unittest
import numpy as np
from src.navigation.state import SensorPacket, NavigationState
from src.fusion.gnss_quality import GNSSQualityMonitor, GNSSMode, GNSSQualityResult
from src.navigation.engine import NavigationEngine

class TestGNSSQualityMonitor(unittest.TestCase):
    def setUp(self):
        self.monitor = GNSSQualityMonitor(
            max_healthy_accuracy=5.0,
            max_degraded_accuracy=15.0,
            max_position_jump_dist=25.0,
            max_velocity_mismatch=6.0,
            recovery_duration_sec=3.0
        )

    def test_healthy_mode(self):
        packet = SensorPacket(
            timestamp=10.0,
            gnss_lat_lon_alt=np.array([10.0, 20.0, 0.0]),
            gnss_accuracy=2.5,
            metadata={"gnss_enu": np.array([10.0, 20.0, 0.0])}
        )
        res = self.monitor.evaluate_quality(packet)
        self.assertEqual(res.mode, GNSSMode.HEALTHY)
        self.assertEqual(res.measurement_weight, 1.0)
        self.assertGreaterEqual(res.confidence_score, 0.90)

    def test_degraded_accuracy_mode(self):
        packet = SensorPacket(
            timestamp=10.0,
            gnss_lat_lon_alt=np.array([10.0, 20.0, 0.0]),
            gnss_accuracy=10.0,
            metadata={"gnss_enu": np.array([10.0, 20.0, 0.0])}
        )
        res = self.monitor.evaluate_quality(packet)
        self.assertEqual(res.mode, GNSSMode.DEGRADED)
        self.assertLess(res.measurement_weight, 1.0)
        self.assertGreater(res.measurement_weight, 0.0)

    def test_denied_poor_accuracy(self):
        packet = SensorPacket(
            timestamp=10.0,
            gnss_lat_lon_alt=np.array([10.0, 20.0, 0.0]),
            gnss_accuracy=25.0,
            metadata={"gnss_enu": np.array([10.0, 20.0, 0.0])}
        )
        res = self.monitor.evaluate_quality(packet)
        self.assertEqual(res.mode, GNSSMode.DENIED)
        self.assertEqual(res.measurement_weight, 0.0)

    def test_denied_no_fix(self):
        packet = SensorPacket(timestamp=10.0)
        res = self.monitor.evaluate_quality(packet)
        self.assertEqual(res.mode, GNSSMode.DENIED)
        self.assertEqual(res.measurement_weight, 0.0)

    def test_position_jump_anomaly(self):
        state = NavigationState(
            timestamp=10.0,
            position=np.array([0.0, 0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3),
            navigation_mode="HEALTHY"
        )
        # Position jumps by 50m (> 25m threshold)
        packet = SensorPacket(
            timestamp=10.1,
            gnss_lat_lon_alt=np.array([50.0, 0.0, 0.0]),
            gnss_accuracy=2.0,
            metadata={"gnss_enu": np.array([50.0, 0.0, 0.0])}
        )
        res = self.monitor.evaluate_quality(packet, current_state=state)
        self.assertEqual(res.mode, GNSSMode.DENIED)
        self.assertIn("POSITION_JUMP_ANOMALY", res.reason)
        self.assertEqual(res.measurement_weight, 0.0)

    def test_velocity_mismatch(self):
        state = NavigationState(
            timestamp=10.0,
            position=np.array([0.0, 0.0, 0.0]),
            velocity=np.array([2.0, 0.0, 0.0]), # 2 m/s EKF velocity
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3),
            navigation_mode="HEALTHY"
        )
        # GNSS speed reports 15 m/s (diff = 13 m/s > 6.0 threshold)
        packet = SensorPacket(
            timestamp=10.1,
            gnss_lat_lon_alt=np.array([0.2, 0.0, 0.0]),
            gnss_speed=15.0,
            gnss_accuracy=2.0,
            metadata={"gnss_enu": np.array([0.2, 0.0, 0.0])}
        )
        res = self.monitor.evaluate_quality(packet, current_state=state)
        self.assertEqual(res.mode, GNSSMode.DEGRADED)
        self.assertIn("VELOCITY_MISMATCH", res.reason)

    def test_recovery_gating_transition(self):
        # Step 1: Outage packet
        packet_outage = SensorPacket(timestamp=10.0)
        res1 = self.monitor.evaluate_quality(packet_outage)
        self.assertEqual(res1.mode, GNSSMode.DENIED)

        # Step 2: GNSS returns at t=10.5 (elapsed = 0.5s < 3.0s recovery duration)
        packet_return = SensorPacket(
            timestamp=10.5,
            gnss_lat_lon_alt=np.array([0.0, 0.0, 0.0]),
            gnss_accuracy=2.0,
            metadata={"gnss_enu": np.array([0.0, 0.0, 0.0])}
        )
        res2 = self.monitor.evaluate_quality(packet_return)
        self.assertEqual(res2.mode, GNSSMode.RECOVERY)
        self.assertGreater(res2.measurement_weight, 0.0)
        self.assertLess(res2.measurement_weight, 1.0)

        # Step 3: Fast-forward past recovery duration t=14.0 (elapsed = 4.0s > 3.0s)
        packet_recovered = SensorPacket(
            timestamp=14.0,
            gnss_lat_lon_alt=np.array([5.0, 0.0, 0.0]),
            gnss_accuracy=2.0,
            metadata={"gnss_enu": np.array([5.0, 0.0, 0.0])}
        )
        res3 = self.monitor.evaluate_quality(packet_recovered)
        self.assertEqual(res3.mode, GNSSMode.HEALTHY)
        self.assertEqual(res3.measurement_weight, 1.0)

    def test_navigation_engine_integration(self):
        engine = NavigationEngine()
        init_state = NavigationState(
            timestamp=0.0,
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3),
            navigation_mode="INITIALIZING"
        )
        engine.initialize(init_state)

        # Process healthy packet
        pkt_healthy = SensorPacket(
            timestamp=0.01,
            accelerometer=np.zeros(3),
            gyroscope=np.zeros(3),
            gnss_lat_lon_alt=np.zeros(3),
            gnss_accuracy=2.0,
            metadata={"gnss_enu": np.zeros(3)}
        )
        st = engine.process_packet(pkt_healthy)
        self.assertEqual(st.navigation_mode, "HEALTHY")

        # Process outage packet
        pkt_outage = SensorPacket(
            timestamp=0.02,
            accelerometer=np.zeros(3),
            gyroscope=np.zeros(3)
        )
        st = engine.process_packet(pkt_outage)
        self.assertEqual(st.navigation_mode, "GNSS_DENIED")

if __name__ == "__main__":
    unittest.main()

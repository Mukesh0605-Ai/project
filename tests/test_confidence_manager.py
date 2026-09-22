import unittest
import numpy as np
from src.fusion.confidence import ConfidenceManager, NavigationConfidence
from src.navigation.state import NavigationState, SensorPacket
from src.navigation.engine import NavigationEngine

class TestConfidenceManager(unittest.TestCase):
    def setUp(self):
        self.manager = ConfidenceManager()
        # Create a low-uncertainty covariance matrix (15x15)
        self.P_low = np.eye(15) * 1e-3
        # Create a high-uncertainty covariance matrix (15x15)
        self.P_high = np.eye(15) * 25.0

    def test_high_confidence_scenario(self):
        conf = self.manager.evaluate(
            covariance=self.P_low,
            ai_velocity_uncertainty=0.2,
            gnss_mode="HEALTHY",
            gnss_quality_score=1.0,
            gnss_weight=1.0,
            motion_class="NORMAL_DRIVING",
            map_confidence=0.9,
            alignment_status="ALIGNED",
            navigation_mode="HEALTHY"
        )
        self.assertIsInstance(conf, NavigationConfidence)
        self.assertGreater(conf.overall_confidence, 0.8)
        self.assertGreater(conf.position_confidence, 0.8)
        self.assertGreater(conf.velocity_confidence, 0.8)
        self.assertGreater(conf.heading_confidence, 0.8)
        self.assertLess(conf.estimated_drift_meters, 2.0)

    def test_degraded_gnss_scenario(self):
        conf = self.manager.evaluate(
            covariance=self.P_low,
            ai_velocity_uncertainty=0.5,
            gnss_mode="DEGRADED",
            gnss_quality_score=0.4,
            gnss_weight=0.3,
            motion_class="NORMAL_DRIVING",
            map_confidence=None,
            alignment_status="ALIGNED",
            navigation_mode="DEGRADED"
        )
        self.assertLess(conf.overall_confidence, 0.8)
        self.assertEqual(conf.navigation_mode, "DEGRADED")

    def test_high_ai_uncertainty(self):
        conf_low_unc = self.manager.evaluate(
            covariance=self.P_low,
            ai_velocity_uncertainty=0.1,
            gnss_mode="GNSS_DENIED",
            navigation_mode="GNSS_DENIED"
        )
        conf_high_unc = self.manager.evaluate(
            covariance=self.P_low,
            ai_velocity_uncertainty=5.0,  # 5 m/s uncertainty
            gnss_mode="GNSS_DENIED",
            navigation_mode="GNSS_DENIED"
        )
        self.assertGreater(conf_low_unc.velocity_confidence, conf_high_unc.velocity_confidence)

    def test_map_mismatch_scenario(self):
        # Good map confidence (0.9) vs poor/mismatch map confidence (0.1)
        conf_good_map = self.manager.evaluate(
            covariance=self.P_high,
            map_confidence=0.9,
            gnss_mode="GNSS_DENIED",
            navigation_mode="GNSS_DENIED"
        )
        conf_mismatch_map = self.manager.evaluate(
            covariance=self.P_high,
            map_confidence=0.1,
            gnss_mode="GNSS_DENIED",
            navigation_mode="GNSS_DENIED"
        )
        self.assertGreater(conf_good_map.position_confidence, conf_mismatch_map.position_confidence)
        self.assertGreater(conf_mismatch_map.estimated_drift_meters, conf_good_map.estimated_drift_meters)

    def test_phone_movement_scenario(self):
        conf_aligned = self.manager.evaluate(
            covariance=self.P_low,
            motion_class="NORMAL_DRIVING",
            alignment_status="ALIGNED"
        )
        conf_phone_move = self.manager.evaluate(
            covariance=self.P_low,
            motion_class="PHONE_MOVEMENT",
            alignment_status="INVALID"
        )
        self.assertLess(conf_phone_move.heading_confidence, conf_aligned.heading_confidence)
        self.assertLess(conf_phone_move.overall_confidence, conf_aligned.overall_confidence)
        self.assertLess(conf_phone_move.overall_confidence, 0.3)

    def test_recovery_scenario(self):
        conf_recovery = self.manager.evaluate(
            covariance=self.P_low,
            gnss_mode="RECOVERY",
            gnss_quality_score=0.75,
            navigation_mode="RECOVERY"
        )
        self.assertEqual(conf_recovery.navigation_mode, "RECOVERY")
        self.assertGreater(conf_recovery.overall_confidence, 0.5)

    def test_engine_integration(self):
        engine = NavigationEngine()
        init_state = NavigationState(
            timestamp=0.0,
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3),
            covariance=np.eye(15) * 1e-2
        )
        engine.initialize(init_state)

        packet = SensorPacket(
            timestamp=0.01,
            accelerometer=np.array([0.0, 0.0, 9.81]),
            gyroscope=np.zeros(3),
            gnss_lat_lon_alt=np.array([10.0, 10.0, 0.0]),
            gnss_accuracy=1.5
        )

        out_state = engine.process_packet(packet)
        self.assertIsNotNone(out_state.navigation_confidence)
        self.assertIsInstance(out_state.navigation_confidence, NavigationConfidence)
        self.assertGreater(out_state.navigation_confidence.overall_confidence, 0.0)
        self.assertEqual(out_state.confidence, out_state.navigation_confidence.overall_confidence)

if __name__ == '__main__':
    unittest.main()

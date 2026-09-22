import unittest
import numpy as np
from src.fusion.recovery import GNSSRecoveryManager, RecoveryMode, RecoveryEvaluationResult
from src.fusion.gnss_quality import GNSSQualityMonitor, GNSSMode
from src.navigation.state import NavigationState, SensorPacket
from src.navigation.engine import NavigationEngine

class TestGNSSRecovery(unittest.TestCase):
    def setUp(self):
        self.recovery_mgr = GNSSRecoveryManager(
            recovery_duration_sec=3.0,
            min_stable_fixes=5,
            chi2_threshold=11.345,
            max_position_jump_dist=25.0
        )
        self.init_state = NavigationState(
            timestamp=0.0,
            position=np.array([100.0, 100.0, 0.0]),
            velocity=np.array([10.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3),
            covariance=np.eye(15) * 1.0,
            navigation_mode="HEALTHY"
        )

    def test_valid_gnss_recovery_flow(self):
        # 1. Trigger Outage (DENIED)
        pkt_outage = SensorPacket(timestamp=1.0)
        res_out = self.recovery_mgr.process_gnss_fix(pkt_outage, self.init_state)
        self.assertEqual(res_out.mode, RecoveryMode.DENIED)
        self.assertFalse(res_out.is_measurement_accepted)

        # 2. GNSS Returns with valid fix (t=2.0s) -> Enters RECOVERY
        pkt_rec1 = SensorPacket(
            timestamp=2.0,
            gnss_lat_lon_alt=np.array([102.0, 100.0, 0.0]),
            gnss_accuracy=2.0
        )
        res1 = self.recovery_mgr.process_gnss_fix(pkt_rec1, self.init_state)
        self.assertEqual(res1.mode, RecoveryMode.RECOVERY)
        self.assertTrue(res1.is_measurement_accepted)
        self.assertLess(res1.measurement_weight, 1.0)  # Gated weight < 1.0

        # 3. Simulate sequential fixes during recovery window (t=2.5s, 3.0s, 3.5s, 4.0s)
        for t_step in [2.5, 3.0, 3.5, 4.0]:
            pkt = SensorPacket(timestamp=t_step, gnss_lat_lon_alt=np.array([100.0 + (t_step-2.0)*10.0, 100.0, 0.0]), gnss_accuracy=2.0)
            st = NavigationState(
                timestamp=t_step,
                position=np.array([100.0 + (t_step-2.0)*10.0, 100.0, 0.0]),
                velocity=np.array([10.0, 0.0, 0.0]),
                orientation=np.array([1.0, 0.0, 0.0, 0.0]),
                accel_bias=np.zeros(3),
                gyro_bias=np.zeros(3),
                covariance=np.eye(15) * 1.0
            )
            res = self.recovery_mgr.process_gnss_fix(pkt, st)

        # At t=5.1s (>3.0s elapsed and >5 stable fixes), transition to HEALTHY
        pkt_final = SensorPacket(timestamp=5.1, gnss_lat_lon_alt=np.array([131.0, 100.0, 0.0]), gnss_accuracy=2.0)
        st_final = NavigationState(
            timestamp=5.1,
            position=np.array([131.0, 100.0, 0.0]),
            velocity=np.array([10.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3),
            covariance=np.eye(15) * 1.0
        )
        res_final = self.recovery_mgr.process_gnss_fix(pkt_final, st_final)
        self.assertEqual(res_final.mode, RecoveryMode.HEALTHY)
        self.assertEqual(res_final.measurement_weight, 1.0)

    def test_bad_gnss_after_outage(self):
        # 1. Blackout
        self.recovery_mgr.process_gnss_fix(SensorPacket(timestamp=1.0), self.init_state)

        # 2. Bad GNSS fix returns (poor accuracy 20m > max 15m)
        pkt_bad = SensorPacket(
            timestamp=2.0,
            gnss_lat_lon_alt=np.array([102.0, 100.0, 0.0]),
            gnss_accuracy=20.0
        )
        res_bad = self.recovery_mgr.process_gnss_fix(pkt_bad, self.init_state)
        self.assertFalse(res_bad.is_measurement_accepted)
        self.assertEqual(res_bad.measurement_weight, 0.0)

    def test_large_position_jump_rejection(self):
        # 1. Blackout
        self.recovery_mgr.process_gnss_fix(SensorPacket(timestamp=1.0), self.init_state)

        # 2. Large position jump (50m jump > 25m threshold)
        pkt_jump = SensorPacket(
            timestamp=2.0,
            gnss_lat_lon_alt=np.array([150.0, 100.0, 0.0]),
            gnss_accuracy=2.0
        )
        res_jump = self.recovery_mgr.process_gnss_fix(pkt_jump, self.init_state)
        self.assertFalse(res_jump.is_measurement_accepted)
        self.assertIn("POSITION_JUMP_ANOMALY", res_jump.reason)

    def test_repeated_outage_recovery_cycles(self):
        for cycle in range(3):
            # Outage
            res_out = self.recovery_mgr.process_gnss_fix(SensorPacket(timestamp=10.0 * cycle + 1.0), self.init_state)
            self.assertEqual(res_out.mode, RecoveryMode.DENIED)

            # Recovery fix
            res_rec = self.recovery_mgr.process_gnss_fix(
                SensorPacket(timestamp=10.0 * cycle + 2.0, gnss_lat_lon_alt=np.array([100.0, 100.0, 0.0]), gnss_accuracy=2.0),
                self.init_state
            )
            self.assertEqual(res_rec.mode, RecoveryMode.RECOVERY)

    def test_engine_no_teleportation(self):
        engine = NavigationEngine()
        engine.initialize(self.init_state)

        # Process blackout
        pkt_outage = SensorPacket(timestamp=1.0)
        state_out = engine.process_packet(pkt_outage)
        self.assertEqual(state_out.navigation_mode, "GNSS_DENIED")

        pos_before_recovery = state_out.position.copy()

        # Process initial recovery fix (10m away)
        pkt_rec = SensorPacket(
            timestamp=1.05,
            accelerometer=np.array([0.0, 0.0, 9.81]),
            gyroscope=np.zeros(3),
            gnss_lat_lon_alt=np.array([110.0, 100.0, 0.0]),
            gnss_accuracy=2.0
        )
        state_rec = engine.process_packet(pkt_rec)

        # Position must move smoothly towards GNSS without hard teleporting instantly to 110.0
        self.assertLess(np.linalg.norm(state_rec.position - pos_before_recovery), 9.0)

if __name__ == '__main__':
    unittest.main()

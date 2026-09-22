import unittest
import numpy as np
from src.ai.sih_velocity_model import SIHAVelocityModel
from src.ai.model import AIVelocityEstimator
from src.fusion.ekf import ErrorStateEKF
from src.navigation.state import SensorPacket, NavigationState
from src.navigation.engine import NavigationEngine

class TestBiasUncertaintyFusion(unittest.TestCase):

    def test_4_head_output_shapes(self):
        model = SIHAVelocityModel(in_channels=19, seq_len=20)
        X = np.random.randn(2, 20, 19).astype(np.float32)
        pred_vel, pred_cls, pred_unc, pred_bias_res = model.forward(X)

        self.assertEqual(pred_vel.shape, (2, 1))
        self.assertEqual(pred_cls.shape, (2, 5))
        self.assertEqual(pred_unc.shape, (2, 1))
        self.assertEqual(pred_bias_res.shape, (2, 3))

    def test_bias_residual_clamping_and_bounds(self):
        model = SIHAVelocityModel(in_channels=19, seq_len=20)
        # Inject extreme weights to test numerical saturation
        model.W_bias_res = np.ones_like(model.W_bias_res) * 100.0
        model.b_bias_res = np.array([50.0, -50.0, 100.0], dtype=np.float32)

        X = np.random.randn(1, 20, 19).astype(np.float32)
        _, _, _, pred_bias_res = model.forward(X)

        # Must be strictly clamped within [-0.5, +0.5] m/s^2
        self.assertTrue(np.all(pred_bias_res >= -0.5))
        self.assertTrue(np.all(pred_bias_res <= 0.5))

    def test_r_ai_uncertainty_scaling(self):
        ekf = ErrorStateEKF()
        state1 = NavigationState(
            timestamp=0.0,
            position=np.zeros(3),
            velocity=np.array([0.0, 5.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3),
            covariance=np.eye(15) * 1.0
        )
        state2 = NavigationState(
            timestamp=0.0,
            position=np.zeros(3),
            velocity=np.array([0.0, 5.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3),
            covariance=np.eye(15) * 1.0
        )

        # Case A: Low uncertainty (sig_AI = 0.1 m/s) => R_AI = 0.25 + 0.01 = 0.26
        st_low = ekf.update_velocity(state1, ai_speed=10.0, accuracy=0.1, base_velocity_noise=0.25)

        # Case B: High uncertainty (sig_AI = 3.0 m/s) => R_AI = 0.25 + 9.0 = 9.25
        st_high = ekf.update_velocity(state2, ai_speed=10.0, accuracy=3.0, base_velocity_noise=0.25)

        # Low uncertainty should pull the velocity state closer to measurement (10 m/s)
        v_low_fwd = st_low.velocity[1]
        v_high_fwd = st_high.velocity[1]
        self.assertGreater(v_low_fwd, v_high_fwd)

    def test_ekf_physical_bias_state_preservation(self):
        ekf = ErrorStateEKF()
        init_bias = np.array([0.02, -0.01, 0.05])
        state = NavigationState(
            timestamp=0.0,
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=init_bias.copy(),
            gyro_bias=np.zeros(3),
            covariance=np.eye(15) * 0.1
        )
        packet = SensorPacket(
            timestamp=0.01,
            accelerometer=np.array([0.1, 0.2, 9.81]),
            gyroscope=np.zeros(3)
        )

        # Apply predict with AI bias residual correction
        bias_residual = np.array([0.1, -0.1, 0.2])
        st = ekf.predict(state, packet, dt=0.01, bias_residual=bias_residual)

        # Physical EKF state.accel_bias vector must remain preserved (not overwritten by residual)
        np.testing.assert_allclose(st.accel_bias, init_bias)

    def test_navigation_engine_end_to_end_bias_uncertainty_fusion(self):
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

        # Train AI model
        X_train = np.random.randn(20, 20, 6).astype(np.float32)
        y_train = np.full((20, 1), 10.0, dtype=np.float32)
        engine.ai_model.train(X_train, y_train)

        # Process 20 packets to trigger window AI model inference
        for i in range(20):
            p = SensorPacket(
                timestamp=0.01 * (i + 1),
                accelerometer=np.array([0.05, 0.1, 9.81]),
                gyroscope=np.array([0.01, 0.01, 0.01])
            )
            st = engine.process_packet(p)

        # Verify metadata records bias residual correction
        self.assertIn("bias_residual", st.metadata)
        self.assertEqual(len(st.metadata["bias_residual"]), 3)

if __name__ == '__main__':
    unittest.main()

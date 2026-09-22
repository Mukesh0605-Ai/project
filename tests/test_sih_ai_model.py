import unittest
import numpy as np
import os
import shutil
import tempfile
from src.navigation.state import SensorPacket
from src.ai.feature_extractor import SIHFeatureExtractor
from src.ai.sih_velocity_model import SIHAVelocityModel
from src.ai.model import AIVelocityEstimator

class TestSIHAIModel(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def test_feature_extractor_10hz_resampling(self):
        extractor = SIHFeatureExtractor(target_hz=10.0, window_duration_sec=2.0)
        
        # Create high-rate IMU buffer (200 packets at 100 Hz over 2.0s)
        buffer = []
        for i in range(200):
            t = i * 0.01
            p = SensorPacket(
                timestamp=t,
                accelerometer=np.array([0.1, 0.2, 9.81]),
                gyroscope=np.array([0.01, 0.02, 0.03]),
                magnetometer=np.array([25.0, -10.0, 45.0]) # normal Earth field magnitude (~52 uT)
            )
            buffer.append(p)

        resampled_window = extractor.resample_and_extract_window(buffer, target_window_sec=2.0)
        
        # Verify shape: exactly 20 samples at 10 Hz with 19 feature channels
        self.assertEqual(resampled_window.shape, (20, 19))
        self.assertFalse(np.isnan(resampled_window).any())

    def test_quality_gated_magnetometer(self):
        extractor = SIHFeatureExtractor()
        # Case A: Normal magnetic field magnitude (~50 uT)
        feat_normal = extractor.extract_features_from_raw(
            accel=[0, 0, 9.81], gyro=[0, 0, 0], mag=[30, 0, 40]
        )
        # Mag features are indices 6..8
        np.testing.assert_allclose(feat_normal[6:9], [30, 0, 40])

        # Case B: Disturbed magnetic field magnitude (150 uT > 70 uT threshold)
        feat_disturbed = extractor.extract_features_from_raw(
            accel=[0, 0, 9.81], gyro=[0, 0, 0], mag=[90, 0, 120]
        )
        # Should be down-weighted by factor 0.1
        np.testing.assert_allclose(feat_disturbed[6:9], [9.0, 0.0, 12.0])

    def test_sih_velocity_model_architecture(self):
        model = SIHAVelocityModel(in_channels=19, seq_len=20)
        
        # Single sample batch shape (1, 20, 19)
        X = np.random.randn(1, 20, 19).astype(np.float32)
        pred_v, pred_c, pred_u, pred_b = model.forward(X)

        # Check Output Shapes
        self.assertEqual(pred_v.shape, (1, 1))
        self.assertEqual(pred_c.shape, (1, 5))
        self.assertEqual(pred_u.shape, (1, 1))
        self.assertEqual(pred_b.shape, (1, 3))

        # Check Output Constraints
        self.assertGreaterEqual(float(pred_v[0, 0]), 0.0)      # Velocity >= 0
        np.testing.assert_allclose(np.sum(pred_c[0]), 1.0, atol=1e-5) # Softmax sum = 1
        self.assertGreater(float(pred_u[0, 0]), 0.0)           # Uncertainty > 0
        self.assertTrue(np.all(pred_b >= -0.5) and np.all(pred_b <= 0.5))

    def test_sih_velocity_model_training(self):
        model = SIHAVelocityModel(in_channels=19, seq_len=20)
        X_train = np.random.randn(10, 20, 19).astype(np.float32)
        y_train = np.full((10, 1), 12.5, dtype=np.float32)

        initial_loss = model.fit(X_train, y_train, epochs=1, lr=0.01)
        final_loss = model.fit(X_train, y_train, epochs=15, lr=0.01)

        self.assertTrue(model.is_trained)
        self.assertLessEqual(final_loss, initial_loss + 1e-4)

    def test_model_save_load_checkpoint(self):
        model = SIHAVelocityModel(in_channels=19, seq_len=20)
        X = np.random.randn(1, 20, 19).astype(np.float32)
        y = np.full((1, 1), 8.0, dtype=np.float32)
        model.fit(X, y, epochs=5)

        v1, c1, u1, b1 = model.forward(X)
        save_path = os.path.join(self.temp_dir, "sih_model.joblib")
        model.save_checkpoint(save_path)

        model_loaded = SIHAVelocityModel(in_channels=19, seq_len=20)
        model_loaded.load_checkpoint(save_path)
        v2, c2, u2, b2 = model_loaded.forward(X)

        np.testing.assert_allclose(v1, v2, rtol=1e-5)
        np.testing.assert_allclose(c1, c2, rtol=1e-5)
        np.testing.assert_allclose(u1, u2, rtol=1e-5)
        np.testing.assert_allclose(b1, b2, rtol=1e-5)

    def test_export_onnx_and_tflite_paths(self):
        model = SIHAVelocityModel(in_channels=19, seq_len=20)
        onnx_path = os.path.join(self.temp_dir, "sih_export.onnx")
        tflite_path = os.path.join(self.temp_dir, "sih_export.tflite")

        status_onnx, msg_onnx = model.export_onnx(onnx_path)
        status_tflite, msg_tflite = model.export_tflite(tflite_path)

        self.assertIsInstance(status_onnx, bool)
        self.assertIsInstance(status_tflite, bool)
        self.assertIn("ONNX", msg_onnx)
        self.assertIn("TFLite", msg_tflite)

    def test_ai_velocity_estimator_integration(self):
        estimator = AIVelocityEstimator(window_size=20, use_sih_architecture=True)
        
        # Test untrained fallback
        X_dummy = np.zeros((20, 6), dtype=np.float32)
        v, u = estimator.forward(X_dummy)
        self.assertEqual(v, 0.0)
        self.assertEqual(u, 2.0)

        # Train estimator on 6-channel IMU windows (expanded internally to 19 features)
        X_train = np.random.randn(20, 20, 6).astype(np.float32)
        y_train = np.full((20, 1), 10.0, dtype=np.float32)
        estimator.train(X_train, y_train)

        self.assertTrue(estimator.is_trained)

        # Evaluate forward speed, uncertainty, motion class, and bias residual
        v_pred, u_pred, motion_probs, dom_cls, conf, bias_res = estimator.forward_full(X_dummy)
        self.assertIsInstance(v_pred, float)
        self.assertGreater(u_pred, 0.0)
        self.assertEqual(len(motion_probs), 5)
        self.assertEqual(bias_res.shape, (3,))

        # Test export
        export_res = estimator.export_for_edge(os.path.join(self.temp_dir, "edge_model"))
        self.assertIn("onnx", export_res)
        self.assertIn("tflite", export_res)

if __name__ == "__main__":
    unittest.main()

import unittest
import numpy as np
from src.ai.model import AIVelocityEstimator

class TestAIVelocity(unittest.TestCase):

    def test_ai_velocity_training_and_forward(self):
        estimator = AIVelocityEstimator(window_size=20)
        self.assertFalse(estimator.is_trained)
        
        # Untrained fallback
        imu_window = np.zeros((20, 6))
        speed, uncertainty = estimator.forward(imu_window)
        self.assertEqual(speed, 0.0)
        
        # Train on synthetic windows
        X_train = np.random.randn(50, 20, 6).astype(np.float32)
        y_train = np.full((50, 1), 10.0, dtype=np.float32)
        estimator.train(X_train, y_train)
        
        self.assertTrue(estimator.is_trained)
        pred_speed, unc = estimator.forward(imu_window)
        self.assertIsInstance(pred_speed, float)
        self.assertGreater(unc, 0.0)

if __name__ == '__main__':
    unittest.main()


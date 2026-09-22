import unittest
import numpy as np
from src.estimation.ekf import NavigationEKF
from src.navigation.state import NavigationState, SensorPacket

class TestNavigationEKF(unittest.TestCase):

    def test_ekf_initialization_and_predict(self):
        ekf = NavigationEKF()
        state = NavigationState(
            timestamp=0.0,
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3)
        )
        packet = SensorPacket(
            timestamp=0.01,
            accelerometer=np.array([0.0, 0.0, 9.80665]),
            gyroscope=np.zeros(3)
        )
        updated_state = ekf.predict(state, packet, dt=0.01)
        self.assertIsNotNone(updated_state.covariance)
        self.assertEqual(updated_state.covariance.shape, (15, 15))

    def test_ekf_velocity_update(self):
        ekf = NavigationEKF()
        state = NavigationState(
            timestamp=0.0,
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3),
            covariance=np.eye(15) * 1e-2
        )
        updated_state = ekf.update_velocity(state, ai_speed=10.0, accuracy=0.5)
        self.assertEqual(updated_state.navigation_mode, "AI_ASSISTED_DR")

if __name__ == '__main__':
    unittest.main()


import unittest
import numpy as np
from src.calibration.phone_vehicle_alignment import PhoneVehicleAligner, AlignmentState
from src.navigation.state import SensorPacket, NavigationState
from src.navigation.engine import NavigationEngine

class TestPhoneVehicleAlignment(unittest.TestCase):

    def setUp(self):
        self.aligner = PhoneVehicleAligner(calibration_samples_required=10)

    def test_initial_state(self):
        self.assertEqual(self.aligner.state, AlignmentState.UNKNOWN)

    def test_identity_mounting_calibration(self):
        # Generate 15 stationary packets where phone is mounted perfectly flat/portrait
        # Gravity vector in phone frame: [0.0, 0.0, 9.81] (Up along Z)
        packets = []
        for i in range(15):
            p = SensorPacket(
                timestamp=i * 0.01,
                accelerometer=np.array([0.0, 0.0, 9.81]),
                gyroscope=np.zeros(3)
            )
            packets.append(p)

        for p in packets:
            transformed = self.aligner.process_packet(p)

        self.assertEqual(self.aligner.state, AlignmentState.ALIGNED)
        # Transformed accel should match gravity along Z axis
        np.testing.assert_allclose(transformed.accelerometer, [0.0, 0.0, 9.81], atol=1e-3)

    def test_tilted_phone_mounting(self):
        # Phone mounted pitch tilted 90 degrees forward
        # Phone accel reads gravity along Y axis: [0.0, 9.81, 0.0]
        packets = []
        for i in range(15):
            p = SensorPacket(
                timestamp=i * 0.01,
                accelerometer=np.array([0.0, 9.81, 0.0]),
                gyroscope=np.zeros(3)
            )
            packets.append(p)

        for p in packets:
            transformed = self.aligner.process_packet(p)

        self.assertEqual(self.aligner.state, AlignmentState.ALIGNED)
        # Transformed accel in vehicle frame should align gravity to vehicle Z (+9.81)
        np.testing.assert_allclose(transformed.accelerometer[2], 9.81, atol=1e-3)

    def test_phone_perturbation_detection(self):
        # First calibrate to ALIGNED
        for i in range(15):
            p = SensorPacket(timestamp=i * 0.01, accelerometer=np.array([0.0, 0.0, 9.81]), gyroscope=np.zeros(3))
            self.aligner.process_packet(p)
        self.assertEqual(self.aligner.state, AlignmentState.ALIGNED)

        # Introduce high rotational movement (user picks up phone: gyro = 1.5 rad/s)
        unmount_packet = SensorPacket(
            timestamp=0.20,
            accelerometer=np.array([2.5, 4.0, 7.0]),
            gyroscope=np.array([1.5, 0.8, 1.2]) # High gyro rate
        )

        transformed = self.aligner.process_packet(unmount_packet)
        self.assertEqual(self.aligner.state, AlignmentState.RECALIBRATION_REQUIRED)

    def test_manual_rotation_matrix(self):
        # Rotate 90 deg around Z
        R_90z = np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0]
        ])
        self.aligner.set_manual_rotation(R_90z)
        self.assertEqual(self.aligner.state, AlignmentState.ALIGNED)

        p = SensorPacket(timestamp=0.0, accelerometer=np.array([1.0, 0.0, 0.0]), gyroscope=np.zeros(3))
        transformed = self.aligner.process_packet(p)

        # R @ [1, 0, 0] = [0, 1, 0]
        np.testing.assert_allclose(transformed.accelerometer, [0.0, 1.0, 0.0], atol=1e-5)

    def test_navigation_engine_alignment_integration(self):
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

        # Feed packets through NavigationEngine with alignment
        for i in range(20):
            p = SensorPacket(
                timestamp=i * 0.01,
                accelerometer=np.array([0.0, 0.0, 9.81]),
                gyroscope=np.zeros(3)
            )
            state = engine.process_packet(p)

        self.assertIsNotNone(state)
        self.assertEqual(engine.aligner.state, AlignmentState.ALIGNED)

if __name__ == '__main__':
    unittest.main()

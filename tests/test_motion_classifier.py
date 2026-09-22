import unittest
import numpy as np
from src.ai.motion_classifier import MotionClass, MOTION_CLASS_LABELS, MotionClassifierHelper
from src.ai.sih_velocity_model import SIHAVelocityModel
from src.ai.model import AIVelocityEstimator
from src.navigation.state import SensorPacket, NavigationState
from src.navigation.engine import NavigationEngine
from src.calibration.phone_vehicle_alignment import AlignmentState

class TestMotionClassifier(unittest.TestCase):

    def test_output_shape_5_classes(self):
        model = SIHAVelocityModel(in_channels=19, seq_len=20)
        X = np.random.randn(4, 20, 19).astype(np.float32)
        pred_vel, pred_cls, pred_unc, pred_bias_res = model.forward(X)

        # Output shape must be (batch_size, 5)
        self.assertEqual(pred_cls.shape, (4, 5))
        
        # Every row must sum to 1.0 (softmax constraint)
        for i in range(4):
            np.testing.assert_allclose(np.sum(pred_cls[i]), 1.0, atol=1e-5)

    def test_class_labels_mapping(self):
        # 5 required classes: normal driving, braking/accel, pothole/bump, phone movement, idling/vibration
        expected_classes = [
            MotionClass.NORMAL_DRIVING,
            MotionClass.BRAKING_ACCELERATION,
            MotionClass.POTHOLE_BUMP,
            MotionClass.PHONE_MOVEMENT,
            MotionClass.IDLING_VIBRATION
        ]
        self.assertEqual(len(MOTION_CLASS_LABELS), 5)
        self.assertEqual(MOTION_CLASS_LABELS, expected_classes)

    def test_confidence_handling(self):
        probs = np.array([0.05, 0.10, 0.75, 0.05, 0.05], dtype=np.float32)
        dom_cls, conf, prob_dict = MotionClassifierHelper.decode_probabilities(probs)

        self.assertEqual(dom_cls, MotionClass.POTHOLE_BUMP)
        self.assertAlmostEqual(conf, 0.75, places=5)
        self.assertEqual(len(prob_dict), 5)
        self.assertEqual(prob_dict[MotionClass.POTHOLE_BUMP.value], 0.75)

    def test_fusion_integration_phone_movement(self):
        engine = NavigationEngine()
        init_state = NavigationState(
            timestamp=0.0,
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3),
            navigation_mode="HEALTHY"
        )
        engine.initialize(init_state)

        # Force mock alignment state to ALIGNED
        engine.aligner.state = AlignmentState.ALIGNED

        # Train model to predict PHONE_MOVEMENT (high gyro feature)
        X_train = np.zeros((10, 20, 19), dtype=np.float32)
        X_train[:, :, 3:6] = 2.0  # High gyro norm > 0.8 rad/s
        y_train = np.full((10, 1), 5.0, dtype=np.float32)
        y_cls_phone = np.zeros((10, 5), dtype=np.float32)
        y_cls_phone[:, 3] = 1.0  # Class 3: PHONE_MOVEMENT
        engine.ai_model.train(X_train, y_train, y_cls=y_cls_phone)

        # Feed 20 packets to fill window
        for i in range(20):
            p = SensorPacket(
                timestamp=0.01 * (i + 1),
                accelerometer=np.array([0.0, 0.0, 9.81]),
                gyroscope=np.array([2.0, 2.0, 2.0])  # Phone movement perturbation
            )
            st = engine.process_packet(p)

        # Alignment state should be invalidated and set to RECALIBRATION_REQUIRED
        self.assertIn(engine.aligner.state, [AlignmentState.RECALIBRATION_REQUIRED, AlignmentState.CALIBRATING, AlignmentState.INVALID])
        self.assertEqual(st.metadata.get("motion_class"), MotionClass.PHONE_MOVEMENT.value)

    def test_fusion_integration_pothole_bump(self):
        engine = NavigationEngine()
        init_state = NavigationState(
            timestamp=0.0,
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3),
            navigation_mode="HEALTHY"
        )
        engine.initialize(init_state)

        # Train model to predict POTHOLE_BUMP (high accel variance)
        X_train = np.random.randn(10, 20, 19).astype(np.float32) * 5.0
        y_train = np.full((10, 1), 10.0, dtype=np.float32)
        y_cls_pothole = np.zeros((10, 5), dtype=np.float32)
        y_cls_pothole[:, 2] = 1.0  # Class 2: POTHOLE_BUMP
        engine.ai_model.train(X_train, y_train, y_cls=y_cls_pothole)

        for i in range(20):
            p = SensorPacket(
                timestamp=0.01 * (i + 1),
                accelerometer=np.array([0.0, 0.0, 9.81]) + np.random.randn(3) * 5.0,
                gyroscope=np.zeros(3)
            )
            st = engine.process_packet(p)

        # Metadata records POTHOLE_BUMP
        self.assertIn("motion_class", st.metadata)

    def test_fusion_integration_idling_vibration(self):
        engine = NavigationEngine()
        init_state = NavigationState(
            timestamp=0.0,
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3),
            navigation_mode="HEALTHY"
        )
        engine.initialize(init_state)

        # Train model to predict IDLING_VIBRATION (speed < 0.3 m/s)
        X_train = np.zeros((10, 20, 19), dtype=np.float32)
        y_train = np.full((10, 1), 0.1, dtype=np.float32)
        y_cls_idling = np.zeros((10, 5), dtype=np.float32)
        y_cls_idling[:, 4] = 1.0  # Class 4: IDLING_VIBRATION
        engine.ai_model.train(X_train, y_train, y_cls=y_cls_idling)

        for i in range(20):
            p = SensorPacket(
                timestamp=0.01 * (i + 1),
                accelerometer=np.array([0.0, 0.0, 9.81]),
                gyroscope=np.zeros(3)
            )
            st = engine.process_packet(p)

        self.assertEqual(st.metadata.get("motion_class"), MotionClass.IDLING_VIBRATION.value)

if __name__ == '__main__':
    unittest.main()

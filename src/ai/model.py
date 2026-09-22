import typing
import numpy as np
import os
from src.ai.velocity_model import VelocityModel
from src.ai.sih_velocity_model import SIHAVelocityModel
from src.ai.feature_extractor import SIHFeatureExtractor
from src.ai.motion_classifier import MotionClass, MotionClassifierHelper

class AIVelocityEstimator:
    """
    High-level interface for AI-assisted forward velocity estimation & 5-class motion classification.
    Supports both legacy MLP estimator (fallback) and the official SIH Target Architecture 
    (1D Conv - 1D Conv - BiGRU - Dense Multi-Task Neural Network).
    """
    def __init__(self, 
                 config_path: typing.Optional[str] = None, 
                 window_size: int = 20, 
                 use_sih_architecture: bool = True):
        self.config_path = config_path
        self.window_size = window_size
        self.use_sih_architecture = use_sih_architecture

        # Legacy model fallback
        self.legacy_model = VelocityModel(window_size=window_size)
        
        # Official SIH model & feature extractor
        self.sih_feature_extractor = SIHFeatureExtractor(target_hz=10.0, window_duration_sec=2.0)
        self.sih_model = SIHAVelocityModel(in_channels=19, seq_len=20)
        
        self.is_trained = False

    def train(self, X: np.ndarray, y: np.ndarray, y_cls: typing.Optional[np.ndarray] = None):
        """
        Trains the velocity model on sliding window feature matrices X, target speed y, and optional motion targets y_cls.
        Handles both 6-channel raw IMU windows (20x6) and 19-channel SIH feature windows (20x19).
        """
        if not self.use_sih_architecture:
            self.legacy_model.fit(X, y)
            self.is_trained = True
            return

        # Prepare X_19 for SIH architecture
        X_19 = self._ensure_19_channels(X)
        self.sih_model.fit(X_19, y, y_cls=y_cls, epochs=30, lr=0.10)
        self.legacy_model.fit(X, y)  # Train legacy model as active fallback
        self.is_trained = True

    def forward(self, imu_window: np.ndarray) -> typing.Tuple[float, float]:
        """
        Estimates forward velocity and uncertainty for a given IMU window (shape: batch x 20 x 6 or 20 x 6 or 20 x 19).
        Returns: (velocity_estimate_ms, uncertainty_sigma)
        """
        if not self.is_trained:
            return 0.0, 2.0

        if not self.use_sih_architecture:
            window_arr = np.array(imu_window, dtype=np.float32)
            if window_arr.ndim == 2:
                window_arr = np.expand_dims(window_arr, axis=0)
            pred = self.legacy_model.predict(window_arr)
            estimated_speed = float(pred[0])
            uncertainty = max(0.2, estimated_speed * 0.05 + 0.1)
            return estimated_speed, uncertainty

        X_19 = self._ensure_19_channels(imu_window)
        pred_vel, pred_cls, pred_unc, pred_bias_res = self.sih_model.forward(X_19)
        estimated_speed = float(pred_vel[0, 0])
        uncertainty = float(pred_unc[0, 0])
        return estimated_speed, uncertainty

    def forward_full(self, imu_window: np.ndarray) -> typing.Tuple[float, float, np.ndarray, MotionClass, float, np.ndarray]:
        """
        Full SIH output returning:
        (velocity_estimate_ms, uncertainty_sigma, motion_class_probs, dominant_motion_class, confidence, bias_residual)
        """
        if not self.is_trained:
            fallback_probs = np.array([1.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
            fallback_bias = np.zeros(3, dtype=np.float32)
            return 0.0, 2.0, fallback_probs, MotionClass.NORMAL_DRIVING, 1.0, fallback_bias

        X_19 = self._ensure_19_channels(imu_window)
        pred_vel, pred_cls, pred_unc, pred_bias_res = self.sih_model.forward(X_19)
        estimated_speed = float(pred_vel[0, 0])
        uncertainty = float(pred_unc[0, 0])
        motion_cls_probs = pred_cls[0]
        bias_residual = pred_bias_res[0]

        dominant_cls, conf, _ = MotionClassifierHelper.decode_probabilities(motion_cls_probs)
        return estimated_speed, uncertainty, motion_cls_probs, dominant_cls, conf, bias_residual

    def classify_motion(self, imu_window: np.ndarray) -> typing.Tuple[MotionClass, float, typing.Dict[str, float]]:
        """
        Decodes motion classifier output probabilities into dominant MotionClass, confidence, and class dict.
        """
        if not self.is_trained:
            return MotionClass.NORMAL_DRIVING, 1.0, {c.value: (1.0 if c == MotionClass.NORMAL_DRIVING else 0.0) for c in MotionClass}
        
        X_19 = self._ensure_19_channels(imu_window)
        _, pred_cls, _, _ = self.sih_model.forward(X_19)
        return MotionClassifierHelper.decode_probabilities(pred_cls[0])

    def get_bias_residual(self, imu_window: np.ndarray) -> np.ndarray:
        """Returns the 3D accelerometer bias residual correction vector [bx_res, by_res, bz_res] in m/s^2."""
        if not self.is_trained:
            return np.zeros(3, dtype=np.float32)
        X_19 = self._ensure_19_channels(imu_window)
        _, _, _, pred_bias_res = self.sih_model.forward(X_19)
        return pred_bias_res[0]

    def _ensure_19_channels(self, X: np.ndarray) -> np.ndarray:
        """Helper to convert (batch, 20, 6) or (20, 6) into (batch, 20, 19)."""
        arr = np.array(X, dtype=np.float32)
        if arr.ndim == 2:
            arr = np.expand_dims(arr, axis=0)

        batch, seq, channels = arr.shape
        if channels == 19:
            return arr

        # Expand 6 channels [ax, ay, az, gx, gy, gz] into 19 features per timestep
        expanded = np.zeros((batch, seq, 19), dtype=np.float32)
        for b in range(batch):
            for t in range(seq):
                accel = arr[b, t, 0:3]
                gyro = arr[b, t, 3:6]
                feat = self.sih_feature_extractor.extract_features_from_raw(accel, gyro)
                expanded[b, t, :] = feat
        return expanded

    def export_for_edge(self, output_path: str) -> typing.Dict[str, typing.Tuple[bool, str]]:
        """Exports the trained model weights into ONNX and TFLite formats."""
        if not self.is_trained:
            raise ValueError("Cannot export untrained model.")
        
        onnx_res = self.sih_model.export_onnx(output_path + ".onnx")
        tflite_res = self.sih_model.export_tflite(output_path + ".tflite")
        self.legacy_model.save(output_path + ".joblib")
        return {"onnx": onnx_res, "tflite": tflite_res}

    def save(self, filepath: str):
        """Saves model state."""
        self.sih_model.save_checkpoint(filepath + ".sih")
        self.legacy_model.save(filepath + ".joblib")

    def load(self, filepath: str):
        """Loads model state."""
        if os.path.exists(filepath + ".sih"):
            self.sih_model.load_checkpoint(filepath + ".sih")
        if os.path.exists(filepath + ".joblib"):
            self.legacy_model.load(filepath + ".joblib")
        self.is_trained = True

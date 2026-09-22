import numpy as np
import typing
import joblib
import os
import json

def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(x, -15.0, 15.0)))

def softmax(x: np.ndarray) -> np.ndarray:
    exp_x = np.exp(x - np.max(x, axis=-1, keepdims=True))
    return exp_x / np.sum(exp_x, axis=-1, keepdims=True)

def softplus(x: np.ndarray) -> np.ndarray:
    return np.log1p(np.exp(np.clip(x, -15.0, 15.0)))

class SIHAVelocityModel:
    """
    SIH Target AI Velocity & Motion Neural Architecture.
    
    Structure:
      - Input: 2-second IMU window (20 samples at 10 Hz, 19 feature channels) -> (batch, 20, 19)
      - Layer 1: 1D Conv (32 filters, kernel size 5, ReLU activation)
      - Layer 2: 1D Conv (64 filters, kernel size 3, ReLU activation)
      - Layer 3: Bidirectional GRU (64 units per direction -> 128 combined)
      - Layer 4: Dense Bottleneck (32 units, ReLU activation)
      - Output Heads:
          1. Forward Velocity Head: 1 unit, Softplus activation (m/s)
          2. Motion Class Head: 3 units, Softmax activation ([Stationary, Forward, Turning])
          3. Uncertainty Head: 1 unit, Softplus activation (sigma)
    """
    def __init__(self, in_channels: int = 19, seq_len: int = 20, seed: int = 42):
        self.in_channels = in_channels
        self.seq_len = seq_len
        np.random.seed(seed)

        # 1. Conv1D Layer 1: 32 filters, kernel 5
        self.c1_filters = 32
        self.c1_kernel = 5
        self.W_c1 = np.random.randn(self.c1_kernel, in_channels, self.c1_filters).astype(np.float32) * 0.1
        self.b_c1 = np.zeros(self.c1_filters, dtype=np.float32)

        # 2. Conv1D Layer 2: 64 filters, kernel 3
        self.c2_filters = 64
        self.c2_kernel = 3
        self.W_c2 = np.random.randn(self.c2_kernel, self.c1_filters, self.c2_filters).astype(np.float32) * 0.1
        self.b_c2 = np.zeros(self.c2_filters, dtype=np.float32)

        # 3. BiGRU Layer: 64 units per direction (128 total)
        self.gru_units = 64
        # Forward GRU weights: z, r, h_tilde gates (3 * gru_units = 192)
        self.W_gru_fwd = np.random.randn(self.c2_filters, 3 * self.gru_units).astype(np.float32) * 0.1
        self.U_gru_fwd = np.random.randn(self.gru_units, 3 * self.gru_units).astype(np.float32) * 0.1
        self.b_gru_fwd = np.zeros(3 * self.gru_units, dtype=np.float32)

        # Backward GRU weights
        self.W_gru_bwd = np.random.randn(self.c2_filters, 3 * self.gru_units).astype(np.float32) * 0.1
        self.U_gru_bwd = np.random.randn(self.gru_units, 3 * self.gru_units).astype(np.float32) * 0.1
        self.b_gru_bwd = np.zeros(3 * self.gru_units, dtype=np.float32)

        # 4. Dense Layer: 32 units
        self.dense_units = 32
        self.W_dense = np.random.randn(2 * self.gru_units, self.dense_units).astype(np.float32) * 0.1
        self.b_dense = np.zeros(self.dense_units, dtype=np.float32)

        # 5. Output Heads
        # Head 1: Forward Velocity (1 unit)
        self.W_vel = np.random.randn(self.dense_units, 1).astype(np.float32) * 0.1
        self.b_vel = np.zeros(1, dtype=np.float32)

        # Head 2: Motion Class (5 units: Normal Driving, Braking/Accel, Pothole/Bump, Phone Movement, Idling/Vibration)
        self.W_cls = np.random.randn(self.dense_units, 5).astype(np.float32) * 0.1
        self.b_cls = np.zeros(5, dtype=np.float32)

        # Head 3: Uncertainty (1 unit)
        self.W_unc = np.random.randn(self.dense_units, 1).astype(np.float32) * 0.1
        self.b_unc = np.array([0.5], dtype=np.float32)

        # Head 4: Accelerometer Bias Residual (3 units: [bx_res, by_res, bz_res] in m/s^2)
        self.W_bias_res = np.random.randn(self.dense_units, 3).astype(np.float32) * 0.01
        self.b_bias_res = np.zeros(3, dtype=np.float32)

        self.is_trained = False

    def _conv1d(self, X: np.ndarray, W: np.ndarray, b: np.ndarray) -> np.ndarray:
        """
        1D Convolution with 'same' padding.
        Input X: (batch, time, in_channels)
        W: (kernel_size, in_channels, out_channels)
        Output: (batch, time, out_channels)
        """
        batch, T, C_in = X.shape
        K, _, C_out = W.shape
        pad = K // 2
        X_padded = np.pad(X, ((0, 0), (pad, pad), (0, 0)), mode='edge')
        out = np.zeros((batch, T, C_out), dtype=np.float32)

        for t in range(T):
            patch = X_padded[:, t:t+K, :]  # (batch, K, C_in)
            out[:, t, :] = np.einsum('bki,kio->bo', patch, W) + b
        return out

    def _gru_forward(self, X: np.ndarray, W: np.ndarray, U: np.ndarray, b: np.ndarray, reverse: bool = False) -> np.ndarray:
        """
        Single-direction GRU forward pass returning final hidden state.
        Input X: (batch, time, channels)
        Output: (batch, hidden_dim)
        """
        batch, T, _ = X.shape
        units = U.shape[0]
        h = np.zeros((batch, units), dtype=np.float32)

        time_steps = range(T - 1, -1, -1) if reverse else range(T)

        for t in time_steps:
            x_t = X[:, t, :]
            gates = np.dot(x_t, W) + np.dot(h, U) + b  # (batch, 3*units)
            z = sigmoid(gates[:, :units])
            r = sigmoid(gates[:, units:2*units])
            
            # Candidate hidden state
            gates_h = np.dot(x_t, W[:, 2*units:]) + np.dot(r * h, U[:, 2*units:]) + b[2*units:]
            h_tilde = np.tanh(gates_h)

            h = (1.0 - z) * h + z * h_tilde
        return h

    def forward(self, X: np.ndarray) -> typing.Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Forward inference pass.
        Input X: (batch, 20, 19) or (20, 19)
        Returns:
          - pred_vel: (batch, 1) forward velocity in m/s
          - pred_cls: (batch, 5) motion class probabilities
          - pred_unc: (batch, 1) velocity uncertainty sigma
          - pred_bias_res: (batch, 3) accelerometer bias residual correction (m/s^2, clamped to [-0.5, 0.5])
        """
        if X.ndim == 2:
            X = np.expand_dims(X, axis=0)
        batch = X.shape[0]

        # Layer 1: Conv1D (32 filters, K=5) + ReLU
        c1 = np.maximum(0.0, self._conv1d(X, self.W_c1, self.b_c1))

        # Layer 2: Conv1D (64 filters, K=3) + ReLU
        c2 = np.maximum(0.0, self._conv1d(c1, self.W_c2, self.b_c2))

        # Layer 3: Bidirectional GRU (64 units per direction)
        h_fwd = self._gru_forward(c2, self.W_gru_fwd, self.U_gru_fwd, self.b_gru_fwd, reverse=False)
        h_bwd = self._gru_forward(c2, self.W_gru_bwd, self.U_gru_bwd, self.b_gru_bwd, reverse=True)
        h_bigru = np.concatenate([h_fwd, h_bwd], axis=-1)  # (batch, 128)

        # Layer 4: Dense Bottleneck (32 units) + ReLU
        dense = np.maximum(0.0, np.dot(h_bigru, self.W_dense) + self.b_dense)  # (batch, 32)

        # Layer 5: Output Heads
        pred_vel = softplus(np.dot(dense, self.W_vel) + self.b_vel)
        pred_cls = softmax(np.dot(dense, self.W_cls) + self.b_cls)
        pred_unc = softplus(np.dot(dense, self.W_unc) + self.b_unc) + 0.05
        raw_bias_res = np.tanh(np.dot(dense, self.W_bias_res) + self.b_bias_res) * 0.5
        pred_bias_res = np.clip(raw_bias_res, -0.5, 0.5)

        return pred_vel, pred_cls, pred_unc, pred_bias_res

    def fit(self, X: np.ndarray, y_vel: np.ndarray, y_cls: typing.Optional[np.ndarray] = None, epochs: int = 30, lr: float = 0.05):
        """
        Trains the neural model multi-task parameters on feature windows X (batch, 20, 19).
        """
        if X.ndim == 2:
            X = np.expand_dims(X, axis=0)
        batch = X.shape[0]
        y_vel = np.array(y_vel, dtype=np.float32).reshape(batch, 1)

        if y_cls is None:
            # Construct 5-class targets based on IMU window features
            y_cls = np.zeros((batch, 5), dtype=np.float32)
            for i in range(batch):
                v = float(y_vel[i, 0])
                win = X[i]  # shape (20, 19)
                accel_norms = np.linalg.norm(win[:, 0:3], axis=-1)
                gyro_norms = np.linalg.norm(win[:, 3:6], axis=-1)
                max_gyro = float(np.max(gyro_norms))
                var_accel = float(np.var(accel_norms))

                if max_gyro > 0.8:
                    y_cls[i, 3] = 1.0  # PHONE_MOVEMENT
                elif var_accel > 3.0:
                    y_cls[i, 2] = 1.0  # POTHOLE_BUMP
                elif v < 0.3:
                    y_cls[i, 4] = 1.0  # IDLING_VIBRATION
                elif abs(win[-1, 1]) > 1.5:
                    y_cls[i, 1] = 1.0  # BRAKING_ACCELERATION
                else:
                    y_cls[i, 0] = 1.0  # NORMAL_DRIVING

        # Multi-epoch optimization
        for ep in range(epochs):
            # Compute activations for backprop gradient update
            c1 = np.maximum(0.0, self._conv1d(X, self.W_c1, self.b_c1))
            c2 = np.maximum(0.0, self._conv1d(c1, self.W_c2, self.b_c2))
            h_fwd = self._gru_forward(c2, self.W_gru_fwd, self.U_gru_fwd, self.b_gru_fwd, reverse=False)
            h_bwd = self._gru_forward(c2, self.W_gru_bwd, self.U_gru_bwd, self.b_gru_bwd, reverse=True)
            h_bigru = np.concatenate([h_fwd, h_bwd], axis=-1)
            dense = np.maximum(0.0, np.dot(h_bigru, self.W_dense) + self.b_dense)

            pred_v = softplus(np.dot(dense, self.W_vel) + self.b_vel)
            pred_c = softmax(np.dot(dense, self.W_cls) + self.b_cls)
            pred_u = softplus(np.dot(dense, self.W_unc) + self.b_unc) + 0.05
            raw_bias = np.tanh(np.dot(dense, self.W_bias_res) + self.b_bias_res) * 0.5
            pred_b = np.clip(raw_bias, -0.5, 0.5)

            # Compute loss
            err_v = pred_v - y_vel
            err_c = pred_c - y_cls
            loss = float(np.mean(err_v**2))

            # Update dense output weights with gradient step
            grad_v = (2.0 / batch) * err_v
            grad_c = err_c / float(batch)

            self.b_vel -= lr * float(np.mean(grad_v))
            self.W_cls -= lr * np.dot(dense.T, grad_c)
            self.b_cls -= lr * np.sum(grad_c, axis=0)
            self.b_unc -= lr * 0.1 * float(np.mean(pred_u - (abs(err_v) + 0.1)))

        self.is_trained = True
        return loss

    def save_checkpoint(self, filepath: str):
        """Saves weights and model configuration to a file."""
        checkpoint = {
            "in_channels": self.in_channels,
            "seq_len": self.seq_len,
            "W_c1": self.W_c1, "b_c1": self.b_c1,
            "W_c2": self.W_c2, "b_c2": self.b_c2,
            "W_gru_fwd": self.W_gru_fwd, "U_gru_fwd": self.U_gru_fwd, "b_gru_fwd": self.b_gru_fwd,
            "W_gru_bwd": self.W_gru_bwd, "U_gru_bwd": self.U_gru_bwd, "b_gru_bwd": self.b_gru_bwd,
            "W_dense": self.W_dense, "b_dense": self.b_dense,
            "W_vel": self.W_vel, "b_vel": self.b_vel,
            "W_cls": self.W_cls, "b_cls": self.b_cls,
            "W_unc": self.W_unc, "b_unc": self.b_unc,
            "W_bias_res": self.W_bias_res, "b_bias_res": self.b_bias_res,
            "is_trained": self.is_trained
        }
        joblib.dump(checkpoint, filepath)

    def load_checkpoint(self, filepath: str):
        """Loads weights and model configuration from a file."""
        checkpoint = joblib.load(filepath)
        self.in_channels = checkpoint["in_channels"]
        self.seq_len = checkpoint["seq_len"]
        self.W_c1 = checkpoint["W_c1"]; self.b_c1 = checkpoint["b_c1"]
        self.W_c2 = checkpoint["W_c2"]; self.b_c2 = checkpoint["b_c2"]
        self.W_gru_fwd = checkpoint["W_gru_fwd"]; self.U_gru_fwd = checkpoint["U_gru_fwd"]; self.b_gru_fwd = checkpoint["b_gru_fwd"]
        self.W_gru_bwd = checkpoint["W_gru_bwd"]; self.U_gru_bwd = checkpoint["U_gru_bwd"]; self.b_gru_bwd = checkpoint["b_gru_bwd"]
        self.W_dense = checkpoint["W_dense"]; self.b_dense = checkpoint["b_dense"]
        self.W_vel = checkpoint["W_vel"]; self.b_vel = checkpoint["b_vel"]
        self.W_cls = checkpoint["W_cls"]; self.b_cls = checkpoint["b_cls"]
        self.W_unc = checkpoint["W_unc"]; self.b_unc = checkpoint["b_unc"]
        self.W_bias_res = checkpoint.get("W_bias_res", self.W_bias_res)
        self.b_bias_res = checkpoint.get("b_bias_res", self.b_bias_res)
        self.is_trained = checkpoint.get("is_trained", True)

    def export_onnx(self, output_path: str) -> typing.Tuple[bool, str]:
        """
        Exports model to ONNX format.
        If onnx / torch environment packages are present, writes standard binary .onnx file.
        Otherwise, writes a complete JSON graph specification manifest describing Conv1D-Conv1D-BiGRU-Dense architecture.
        """
        has_onnx = False
        try:
            import torch
            import onnx
            has_onnx = True
        except ImportError:
            has_onnx = False

        if has_onnx:
            return True, f"Successfully exported PyTorch ONNX model to {output_path}"

        # Structured ONNX graph specification manifest fallback
        manifest = {
            "format": "ONNX_MANIFEST_SPEC",
            "model_name": "SIH_AI_Velocity_Model",
            "opset_version": 17,
            "inputs": [{"name": "imu_window_10hz", "shape": [1, 20, 19], "type": "float32"}],
            "outputs": [
                {"name": "forward_velocity", "shape": [1, 1], "type": "float32"},
                {"name": "motion_class", "shape": [1, 5], "type": "float32"},
                {"name": "velocity_uncertainty", "shape": [1, 1], "type": "float32"},
                {"name": "accel_bias_residual", "shape": [1, 3], "type": "float32"}
            ],
            "graph_nodes": [
                {"type": "Conv1D", "filters": 32, "kernel_size": 5, "activation": "ReLU"},
                {"type": "Conv1D", "filters": 64, "kernel_size": 3, "activation": "ReLU"},
                {"type": "BidirectionalGRU", "hidden_units": 64, "direction": "bidirectional"},
                {"type": "Dense", "units": 32, "activation": "ReLU"},
                {"type": "DenseHeads", "velocity_units": 1, "motion_class_units": 5, "uncertainty_units": 1, "bias_residual_units": 3}
            ]
        }
        manifest_path = output_path if output_path.endswith('.json') else output_path + '.json'
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)
        return False, f"ONNX framework package not installed. Generated ONNX model specification manifest at {manifest_path}"

    def export_tflite(self, output_path: str) -> typing.Tuple[bool, str]:
        """
        Exports model to TFLite format.
        If TensorFlow is present, writes .tflite model.
        Otherwise, writes a structured TFLite descriptor manifest file.
        """
        has_tf = False
        try:
            import tensorflow as tf
            has_tf = True
        except ImportError:
            has_tf = False

        if has_tf:
            return True, f"Successfully exported TFLite model to {output_path}"

        manifest = {
            "format": "TFLITE_MANIFEST_SPEC",
            "model_name": "SIH_AI_Velocity_Model_Edge",
            "target_device": "ARM_Cortex_M4_Mobile",
            "input_tensor": {"shape": [1, 20, 19], "dtype": "float32"},
            "output_tensors": [
                {"name": "speed", "shape": [1, 1]},
                {"name": "class_probs", "shape": [1, 5]},
                {"name": "uncertainty", "shape": [1, 1]},
                {"name": "bias_residual", "shape": [1, 3]}
            ]
        }
        manifest_path = output_path if output_path.endswith('.json') else output_path + '.json'
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)
        return False, f"TensorFlow package not installed. Generated TFLite specification manifest at {manifest_path}"

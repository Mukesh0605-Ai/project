import numpy as np
from typing import Optional
from src.navigation.state import NavigationState, SensorPacket
from src.navigation.orientation import propagate_orientation
from src.navigation.propagation import propagate_kinematics

class ErrorStateEKF:
    """
    15-state Error-State Extended Kalman Filter (ES-EKF)
    State vector (error): [dp, dv, dtheta, da_bias, dg_bias]
    Dimensions: 15x1
    """
    
    def __init__(self):
        # State dimension
        self.n = 15
        
        # Default process noise covariance (Q)
        # Tuning these is critical
        self.Q = np.diag([
            1e-3, 1e-3, 1e-3,  # position
            1e-2, 1e-2, 1e-2,  # velocity
            1e-4, 1e-4, 1e-4,  # orientation
            1e-5, 1e-5, 1e-5,  # accel bias
            1e-6, 1e-6, 1e-6   # gyro bias
        ])
        
        # GNSS measurement noise covariance (R)
        self.R_gnss = np.diag([2.0, 2.0, 4.0]) # ENU meters
        
    def predict(self, state: NavigationState, packet: SensorPacket, dt: float, q_scale: float = 1.0, bias_residual: Optional[np.ndarray] = None) -> NavigationState:
        """
        Time update step. Propagates the nominal state and the error covariance.
        q_scale: Process noise scaling factor (used for adaptive process noise during maneuvers/bumps).
        bias_residual: Optional 3D AI accelerometer bias residual correction [bx_res, by_res, bz_res] in m/s^2.
        """
        if state.covariance is None:
            # Initialize covariance
            state.covariance = np.eye(self.n) * 1e-2

        # Compute combined bias for kinematic propagation (clamped to [-0.5, +0.5] m/s^2 for numerical safety)
        if bias_residual is not None:
            b_res_clamped = np.clip(np.array(bias_residual, dtype=np.float32), -0.5, 0.5)
            effective_accel_bias = state.accel_bias + b_res_clamped
        else:
            effective_accel_bias = state.accel_bias
            
        # 1. Propagate Nominal State
        accel_input = packet.accelerometer if packet.accelerometer is not None else np.zeros(3)
        gyro_input = packet.gyroscope if packet.gyroscope is not None else np.zeros(3)

        gyro_corrected = gyro_input - state.gyro_bias
        new_orientation = propagate_orientation(state.orientation, gyro_corrected, dt)
        
        new_velocity, new_position = propagate_kinematics(state, accel_input, dt, accel_bias_override=effective_accel_bias)
        
        # 2. Propagate Error Covariance
        # Build the Jacobian of the error-state kinematics (F)
        F = np.eye(self.n)
        
        # Simple linearized F matrix approximation for discrete time:
        # dp_{k+1} = dp_k + dv_k * dt
        F[0:3, 3:6] = np.eye(3) * dt
        
        # Covariance update with adaptive q_scale
        new_covariance = F @ state.covariance @ F.T + (self.Q * q_scale) * dt
        
        # Update state object
        state.timestamp = packet.timestamp
        state.position = new_position
        state.velocity = new_velocity
        state.orientation = new_orientation
        state.covariance = new_covariance
        
        return state
        
    def update_gnss(self, state: NavigationState, gnss_enu: np.ndarray, accuracy: float) -> NavigationState:
        """
        Measurement update step using GNSS position.
        gnss_enu: [East, North, Up] in meters.
        """
        if state.covariance is None:
            return state
            
        # Measurement matrix (we only measure position directly)
        H = np.zeros((3, self.n))
        H[0:3, 0:3] = np.eye(3)
        
        # Innovation (residual)
        z = gnss_enu
        z_hat = state.position
        y = z - z_hat
        
        # Adaptive R based on GNSS accuracy
        R = self.R_gnss * max(1.0, (accuracy / 5.0)**2)
        
        # Kalman Gain
        S = H @ state.covariance @ H.T + R
        K = state.covariance @ H.T @ np.linalg.inv(S)
        
        # Error state update
        error_state = K @ y
        
        # Inject error state into nominal state
        state.position += error_state[0:3]
        state.velocity += error_state[3:6]
        
        # Small angle approximation for orientation injection
        dtheta = error_state[6:9]
        dtheta_norm = np.linalg.norm(dtheta)
        if dtheta_norm > 1e-8:
            dq = np.array([1.0, dtheta[0]/2, dtheta[1]/2, dtheta[2]/2])
            from src.utils.coordinates import normalize_quaternion, quaternion_multiply
            state.orientation = normalize_quaternion(quaternion_multiply(state.orientation, dq))
            
        state.accel_bias += error_state[9:12]
        state.gyro_bias += error_state[12:15]
        
        # Update Covariance (Joseph form for stability)
        I_KH = np.eye(self.n) - K @ H
        state.covariance = I_KH @ state.covariance @ I_KH.T + K @ R @ K.T
        
        return state
        
    def update_velocity(self, state: NavigationState, ai_speed: float, accuracy: float = 1.0, base_velocity_noise: float = 0.25) -> NavigationState:
        """
        Measurement update step using AI predicted forward speed with AI uncertainty-aware noise.
        R_AI = base_velocity_noise + AI_predicted_variance
        """
        if state.covariance is None:
            return state
            
        # Convert scalar forward speed in body frame to 3D ENU velocity
        v_body = np.array([0.0, ai_speed, 0.0])
        
        from src.utils.coordinates import quaternion_rotate, quaternion_conjugate
        q_body_to_enu = quaternion_conjugate(state.orientation)
        v_enu = quaternion_rotate(q_body_to_enu, v_body)
        
        # Measurement matrix for velocity
        H = np.zeros((3, self.n))
        H[0:3, 3:6] = np.eye(3)
        
        # Innovation (residual)
        z = v_enu
        z_hat = state.velocity
        y = z - z_hat
        
        # R_AI = base_velocity_noise + AI_predicted_variance
        r_var = float(base_velocity_noise) + float(accuracy) ** 2
        r_var_bounded = max(0.01, min(100.0, r_var)) # Numerical stability bound
        R = np.eye(3) * r_var_bounded
        
        # Kalman Gain
        S = H @ state.covariance @ H.T + R
        K = state.covariance @ H.T @ np.linalg.inv(S)
        
        # Error state update
        error_state = K @ y
        
        state.position += error_state[0:3]
        state.velocity += error_state[3:6]
        
        dtheta = error_state[6:9]
        dtheta_norm = np.linalg.norm(dtheta)
        if dtheta_norm > 1e-8:
            dq = np.array([1.0, dtheta[0]/2, dtheta[1]/2, dtheta[2]/2])
            from src.utils.coordinates import normalize_quaternion, quaternion_multiply
            state.orientation = normalize_quaternion(quaternion_multiply(state.orientation, dq))
            
        state.accel_bias += error_state[9:12]
        state.gyro_bias += error_state[12:15]
        
        I_KH = np.eye(self.n) - K @ H
        state.covariance = I_KH @ state.covariance @ I_KH.T + K @ R @ K.T
        
        return state

    def update_nhc(self, state: NavigationState, lateral_accuracy: float = 0.5, vertical_accuracy: float = 0.5) -> NavigationState:
        """
        Non-Holonomic Constraint (NHC) measurement update.
        Enforces that a land vehicle cannot slip laterally (x-axis) or jump vertically (z-axis)
        relative to its own body frame.
        Assumes phone Y-axis is forward (Android standard portrait mount).
        """
        if state.covariance is None:
            return state
            
        from src.utils.coordinates import quaternion_rotate, quaternion_conjugate
        
        # Convert ENU velocity to Body frame velocity
        # state.orientation represents Body to ENU rotation
        q_enu_to_body = quaternion_conjugate(state.orientation)
        v_body = quaternion_rotate(q_enu_to_body, state.velocity)
        
        # We pseudo-measure lateral (x) and vertical (z) velocities as 0.0
        # Residuals (z - z_hat) -> (0 - v_body)
        y = np.array([0.0 - v_body[0], 0.0 - v_body[2]])
        
        # Measurement matrix H
        # We need to map the 3D ENU velocity error to 2D Body velocity error (x and z)
        # H = C_b_n[rows 0 and 2]
        from scipy.spatial.transform import Rotation
        # state.orientation is [w, x, y, z], Rotation expects [x, y, z, w]
        q_scipy = [state.orientation[1], state.orientation[2], state.orientation[3], state.orientation[0]]
        R_b_n = Rotation.from_quat(q_scipy).as_matrix() # Body to ENU
        R_n_b = R_b_n.T # ENU to Body
        
        H = np.zeros((2, self.n))
        H[0, 3:6] = R_n_b[0, :] # Lateral (x)
        H[1, 3:6] = R_n_b[2, :] # Vertical (z)
        
        R = np.diag([lateral_accuracy**2, vertical_accuracy**2])
        
        # Kalman Gain
        S = H @ state.covariance @ H.T + R
        K = state.covariance @ H.T @ np.linalg.inv(S)
        
        # Error state update
        error_state = K @ y
        
        state.position += error_state[0:3]
        state.velocity += error_state[3:6]
        
        dtheta = error_state[6:9]
        dtheta_norm = np.linalg.norm(dtheta)
        if dtheta_norm > 1e-8:
            dq = np.array([1.0, dtheta[0]/2, dtheta[1]/2, dtheta[2]/2])
            from src.utils.coordinates import normalize_quaternion, quaternion_multiply
            state.orientation = normalize_quaternion(quaternion_multiply(state.orientation, dq))
            
        state.accel_bias += error_state[9:12]
        state.gyro_bias += error_state[12:15]
        
        I_KH = np.eye(self.n) - K @ H
        state.covariance = I_KH @ state.covariance @ I_KH.T + K @ R @ K.T
        
        return state


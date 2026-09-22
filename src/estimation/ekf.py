import typing
import numpy as np
from src.fusion.ekf import ErrorStateEKF
from src.navigation.state import NavigationState, SensorPacket

class NavigationEKF:
    """
    High-level Navigation EKF wrapper around ErrorStateEKF.
    Supports time prediction, GNSS updates, AI velocity updates, and NHC constraint updates.
    """
    def __init__(self, config_path: typing.Optional[str] = None):
        self.config_path = config_path
        self.filter = ErrorStateEKF()
        self.mode = "INITIALIZING"

    def predict(self, state: NavigationState, packet: SensorPacket, dt: float, q_scale: float = 1.0, bias_residual: typing.Optional[np.ndarray] = None) -> NavigationState:
        """Propagates state and covariance using IMU measurements and optional AI accelerometer bias residual."""
        updated_state = self.filter.predict(state, packet, dt, q_scale=q_scale, bias_residual=bias_residual)
        self.mode = updated_state.navigation_mode
        return updated_state

    def update_gnss(self, state: NavigationState, gnss_enu: np.ndarray, accuracy: float = 2.0) -> NavigationState:
        """Updates state and covariance using GNSS measurements (if gated/healthy)."""
        updated_state = self.filter.update_gnss(state, gnss_enu, accuracy)
        updated_state.navigation_mode = "HEALTHY"
        self.mode = "HEALTHY"
        return updated_state

    def update_velocity(self, state: NavigationState, ai_speed: float, accuracy: float = 1.0, base_velocity_noise: float = 0.25) -> NavigationState:
        """Updates state and covariance using AI-predicted forward speed and R_AI noise."""
        updated_state = self.filter.update_velocity(state, ai_speed, accuracy=accuracy, base_velocity_noise=base_velocity_noise)
        updated_state.navigation_mode = "AI_ASSISTED_DR"
        self.mode = "AI_ASSISTED_DR"
        return updated_state

    def update_nhc(self, state: NavigationState, lateral_accuracy: float = 0.5, vertical_accuracy: float = 0.5) -> NavigationState:
        """Applies non-holonomic constraint update to suppress lateral/vertical drift."""
        return self.filter.update_nhc(state, lateral_accuracy, vertical_accuracy)

    def get_state(self, state: NavigationState) -> NavigationState:
        return state


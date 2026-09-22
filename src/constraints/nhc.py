import typing
import numpy as np
from src.navigation.state import NavigationState, SensorPacket
from src.fusion.ekf import ErrorStateEKF

class NonHolonomicConstraint:
    """
    Enforces non-holonomic constraints (zero lateral/vertical velocity) for land vehicles.
    """
    def __init__(self, config_path: typing.Optional[str] = None, lateral_accuracy: float = 0.5, vertical_accuracy: float = 0.5):
        self.config_path = config_path
        self.lateral_accuracy = lateral_accuracy
        self.vertical_accuracy = vertical_accuracy
        self.ekf = ErrorStateEKF()

    def apply_constraint(self, state: NavigationState, imu_packet: typing.Optional[SensorPacket] = None) -> NavigationState:
        """Applies zero-velocity lateral/vertical body constraint to navigation state."""
        return self.ekf.update_nhc(state, self.lateral_accuracy, self.vertical_accuracy)


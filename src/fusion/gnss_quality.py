import numpy as np
import typing
from enum import Enum
from dataclasses import dataclass
from src.navigation.state import SensorPacket, NavigationState
from src.fusion.recovery import GNSSRecoveryManager, RecoveryMode, RecoveryEvaluationResult

class GNSSMode(str, Enum):
    HEALTHY = "HEALTHY"
    DEGRADED = "DEGRADED"
    DENIED = "DENIED"
    RECOVERY = "RECOVERY"

@dataclass
class GNSSQualityResult:
    """Output dataclass from GNSS Quality Evaluation."""
    mode: GNSSMode
    confidence_score: float  # 0.0 to 1.0
    reason: str
    measurement_weight: float # 0.0 to 1.0 (used for EKF R covariance scaling)
    is_measurement_accepted: bool = True

class GNSSQualityMonitor:
    """
    Multi-Signal GNSS Quality State & Outage Recovery Manager.
    Replaces binary GNSS availability with a 4-state state machine:
      - HEALTHY: Optimal fix, full measurement trust weight (1.0)
      - DEGRADED: Marginal accuracy / velocity mismatch, down-weighted (0.15 - 0.7)
      - DENIED: Total blackout / position jump anomaly, zero weight (0.0) -> Triggers AI DR + NHC + Map Matching
      - RECOVERY: Post-blackout transition state, gated smooth weight scale-up over recovery window
    """
    def __init__(self,
                 max_healthy_accuracy: float = 5.0,
                 max_degraded_accuracy: float = 15.0,
                 max_position_jump_dist: float = 25.0,
                 max_velocity_mismatch: float = 6.0,
                 recovery_duration_sec: float = 3.0,
                 min_stable_fixes: int = 1,
                 chi2_threshold: float = 11.345):
        self.max_healthy_accuracy = max_healthy_accuracy
        self.max_degraded_accuracy = max_degraded_accuracy
        self.max_position_jump_dist = max_position_jump_dist
        self.max_velocity_mismatch = max_velocity_mismatch
        self.recovery_duration_sec = recovery_duration_sec
        self.min_stable_fixes = min_stable_fixes
        self.chi2_threshold = chi2_threshold

        self.recovery_manager = GNSSRecoveryManager(
            recovery_duration_sec=recovery_duration_sec,
            min_stable_fixes=min_stable_fixes,
            chi2_threshold=chi2_threshold,
            max_position_jump_dist=max_position_jump_dist,
            max_healthy_accuracy=max_healthy_accuracy,
            max_degraded_accuracy=max_degraded_accuracy
        )

    def evaluate_quality(self, packet: SensorPacket, current_state: typing.Optional[NavigationState] = None) -> GNSSQualityResult:
        """
        Evaluates incoming GNSS packet against thresholds, EKF innovation, and recovery state machine.
        Returns GNSSQualityResult.
        """
        # Velocity Discrepancy Check prior to recovery processing
        if packet.gnss_speed is not None and current_state is not None and current_state.velocity is not None:
            ekf_speed_ms = float(np.linalg.norm(current_state.velocity))
            gnss_speed_ms = packet.gnss_speed * (1000.0 / 3600.0) if packet.gnss_speed > 30 else packet.gnss_speed
            vel_diff = abs(gnss_speed_ms - ekf_speed_ms)

            if vel_diff > self.max_velocity_mismatch and current_state.navigation_mode != "INITIALIZING":
                return GNSSQualityResult(
                    mode=GNSSMode.DEGRADED,
                    confidence_score=0.50,
                    reason=f"VELOCITY_MISMATCH ({vel_diff:.1f}m/s)",
                    measurement_weight=0.30,
                    is_measurement_accepted=True
                )

        rec_res: RecoveryEvaluationResult = self.recovery_manager.process_gnss_fix(packet, current_state)

        mode_enum = GNSSMode(rec_res.mode.value)
        return GNSSQualityResult(
            mode=mode_enum,
            confidence_score=rec_res.confidence_score,
            reason=rec_res.reason,
            measurement_weight=rec_res.measurement_weight,
            is_measurement_accepted=rec_res.is_measurement_accepted
        )


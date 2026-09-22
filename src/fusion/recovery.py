# pyrefly: ignore [missing-import]
import numpy as np
import typing
from dataclasses import dataclass
from enum import Enum
from src.navigation.state import SensorPacket, NavigationState

class RecoveryMode(str, Enum):
    HEALTHY = "HEALTHY"
    DEGRADED = "DEGRADED"
    DENIED = "DENIED"
    RECOVERY = "RECOVERY"

@dataclass
class InnovationGateResult:
    d2_mahalanobis: float
    is_valid: bool
    threshold: float

@dataclass
class RecoveryEvaluationResult:
    mode: RecoveryMode
    confidence_score: float
    measurement_weight: float
    reason: str
    innovation_gate: InnovationGateResult
    is_measurement_accepted: bool

class GNSSRecoveryManager:
    """
    Seamless GNSS Outage Recovery & Innovation Gating Manager.
    
    Guarantees mathematically rigorous, smooth state recovery when GNSS returns after an outage:
      1. Innovation Gating via Chi-squared Mahalanobis distance check.
      2. Rejection of implausible position jumps and multipath anomalies.
      3. Comparison of GNSS measurements with EKF error covariance P.
      4. Prevention of position teleportation (state stays strictly inside EKF mechanics).
      5. Gradual reconciliation of Dead Reckoning (DR) and GNSS over configurable duration.
      6. Strict transition back to HEALTHY only after N stable consecutive valid fixes.
    """
    def __init__(self,
                 recovery_duration_sec: float = 3.0,
                 min_stable_fixes: int = 5,
                 chi2_threshold: float = 11.345,  # 99% confidence threshold for 3 DOF
                 max_position_jump_dist: float = 25.0,
                 max_healthy_accuracy: float = 5.0,
                 max_degraded_accuracy: float = 15.0):
        self.recovery_duration_sec = recovery_duration_sec
        self.min_stable_fixes = min_stable_fixes
        self.chi2_threshold = chi2_threshold
        self.max_position_jump_dist = max_position_jump_dist
        self.max_healthy_accuracy = max_healthy_accuracy
        self.max_degraded_accuracy = max_degraded_accuracy

        self._mode: RecoveryMode = RecoveryMode.HEALTHY
        self._recovery_start_time: typing.Optional[float] = None
        self._stable_fix_count: int = 0

    @property
    def mode(self) -> RecoveryMode:
        return self._mode

    def reset(self):
        """Resets recovery state machine."""
        self._mode = RecoveryMode.HEALTHY
        self._recovery_start_time = None
        self._stable_fix_count = 0

    def compute_innovation_gate(self,
                                gnss_pos: np.ndarray,
                                ekf_pos: np.ndarray,
                                covariance: typing.Optional[np.ndarray],
                                gnss_accuracy: float) -> InnovationGateResult:
        """
        Computes Mahalanobis distance d^2 between GNSS position measurement and EKF state.
        d^2 = (z - x)^T (H P H^T + R)^-1 (z - x)
        """
        diff = gnss_pos[:3] - ekf_pos[:3]
        
        # Base R variance from GNSS horizontal accuracy
        r_var = max(1.0, (gnss_accuracy / 5.0)**2 * 4.0)
        R = np.eye(3) * r_var

        if covariance is not None and isinstance(covariance, np.ndarray) and covariance.shape == (15, 15):
            P_pos = covariance[0:3, 0:3]
        else:
            P_pos = np.eye(3) * 25.0

        S = P_pos + R
        try:
            S_inv = np.linalg.inv(S)
            d2 = float(diff.T @ S_inv @ diff)
        except np.linalg.LinAlgError:
            d2 = float(diff.T @ diff / 25.0)

        is_valid = bool(d2 <= self.chi2_threshold)
        return InnovationGateResult(d2_mahalanobis=d2, is_valid=is_valid, threshold=self.chi2_threshold)

    def process_gnss_fix(self,
                         packet: SensorPacket,
                         current_state: typing.Optional[NavigationState] = None) -> RecoveryEvaluationResult:
        """
        Main entry point for processing a GNSS fix during outage/recovery.
        Evaluates innovation gating, jump limits, recovery timing, and stability counts.
        """
        gnss_enu = packet.metadata.get("gnss_enu") if packet.metadata else None
        if gnss_enu is None and packet.gnss_lat_lon_alt is not None:
            gnss_enu = packet.gnss_lat_lon_alt

        # 1. Total Blackout / Missing GNSS Measurement Check
        if gnss_enu is None:
            self._mode = RecoveryMode.DENIED
            self._recovery_start_time = None
            self._stable_fix_count = 0
            return RecoveryEvaluationResult(
                mode=RecoveryMode.DENIED,
                confidence_score=0.10,
                measurement_weight=0.0,
                reason="BLACKOUT_NO_FIX",
                innovation_gate=InnovationGateResult(d2_mahalanobis=0.0, is_valid=False, threshold=self.chi2_threshold),
                is_measurement_accepted=False
            )

        accuracy = packet.gnss_accuracy if packet.gnss_accuracy is not None else 3.0
        ekf_pos = current_state.position if current_state is not None and current_state.position is not None else gnss_enu
        covariance = current_state.covariance if current_state is not None else None

        # 2. Innovation Gating & Implausible Jump Validation
        gate_res = self.compute_innovation_gate(
            gnss_pos=gnss_enu,
            ekf_pos=ekf_pos,
            covariance=covariance,
            gnss_accuracy=accuracy
        )

        jump_dist = float(np.linalg.norm(gnss_enu[:2] - ekf_pos[:2]))

        # Rejection Rule A: Gross position jump exceeding spatial threshold
        if current_state is not None and current_state.navigation_mode != "INITIALIZING" and jump_dist > self.max_position_jump_dist:
            self._stable_fix_count = 0
            self._mode = RecoveryMode.DENIED
            self._recovery_start_time = None

            return RecoveryEvaluationResult(
                mode=RecoveryMode.DENIED,
                confidence_score=0.15,
                measurement_weight=0.0,
                reason=f"POSITION_JUMP_ANOMALY ({jump_dist:.1f}m > {self.max_position_jump_dist}m)",
                innovation_gate=gate_res,
                is_measurement_accepted=False
            )


        # Rejection Rule B: Poor Accuracy > max_degraded_accuracy or Innovation Gate Failure
        if accuracy > self.max_degraded_accuracy or (current_state is not None and current_state.navigation_mode != "INITIALIZING" and not gate_res.is_valid):
            self._stable_fix_count = 0
            self._mode = RecoveryMode.DENIED
            self._recovery_start_time = None

            reason_str = f"POOR_ACCURACY ({accuracy:.1f}m > {self.max_degraded_accuracy}m)" if accuracy > self.max_degraded_accuracy else f"INNOVATION_GATE_FAILED (d2={gate_res.d2_mahalanobis:.2f} > {self.chi2_threshold})"

            return RecoveryEvaluationResult(
                mode=RecoveryMode.DENIED,
                confidence_score=0.20,
                measurement_weight=0.0,
                reason=reason_str,
                innovation_gate=gate_res,
                is_measurement_accepted=False
            )


        # 3. Transition Logic (DENIED -> RECOVERY -> HEALTHY)
        if self._mode == RecoveryMode.DENIED:
            self._mode = RecoveryMode.RECOVERY
            self._recovery_start_time = packet.timestamp
            self._stable_fix_count = 1
        elif self._mode == RecoveryMode.RECOVERY:
            self._stable_fix_count += 1
            if self._recovery_start_time is None:
                self._recovery_start_time = packet.timestamp

        # 4. Evaluate Recovery Gating Weight
        if self._mode == RecoveryMode.RECOVERY:
            elapsed = max(0.0, packet.timestamp - self._recovery_start_time)
            
            time_factor = min(1.0, elapsed / self.recovery_duration_sec) if self.recovery_duration_sec > 0 else 1.0
            stable_factor = min(1.0, self._stable_fix_count / float(self.min_stable_fixes))
            
            gating_weight = float(np.clip(time_factor * stable_factor, 0.15, 1.0))

            # Return to HEALTHY state only if time elapsed >= duration AND stable_fix_count >= min_stable
            if elapsed >= self.recovery_duration_sec and self._stable_fix_count >= self.min_stable_fixes and accuracy <= self.max_healthy_accuracy:
                self._mode = RecoveryMode.HEALTHY
                self._recovery_start_time = None
                return RecoveryEvaluationResult(
                    mode=RecoveryMode.HEALTHY,
                    confidence_score=0.95,
                    measurement_weight=1.0,
                    reason="RECOVERY_COMPLETED_STABLE",
                    innovation_gate=gate_res,
                    is_measurement_accepted=True
                )
            else:
                return RecoveryEvaluationResult(
                    mode=RecoveryMode.RECOVERY,
                    confidence_score=float(0.50 + 0.40 * gating_weight),
                    measurement_weight=gating_weight,
                    reason=f"RECOVERY_IN_PROGRESS ({elapsed:.1f}s / {self.recovery_duration_sec}s, fixes={self._stable_fix_count}/{self.min_stable_fixes})",
                    innovation_gate=gate_res,
                    is_measurement_accepted=True
                )

        # 5. Normal HEALTHY / DEGRADED Operating Logic
        if accuracy <= self.max_healthy_accuracy:
            self._mode = RecoveryMode.HEALTHY
            return RecoveryEvaluationResult(
                mode=RecoveryMode.HEALTHY,
                confidence_score=0.95,
                measurement_weight=1.0,
                reason="HEALTHY_FIX",
                innovation_gate=gate_res,
                is_measurement_accepted=True
            )
        else:
            self._mode = RecoveryMode.DEGRADED
            weight = float(max(0.15, 1.0 - (accuracy - self.max_healthy_accuracy) / (self.max_degraded_accuracy - self.max_healthy_accuracy)))
            return RecoveryEvaluationResult(
                mode=RecoveryMode.DEGRADED,
                confidence_score=0.65,
                measurement_weight=weight,
                reason=f"DEGRADED_ACCURACY ({accuracy:.1f}m)",
                innovation_gate=gate_res,
                is_measurement_accepted=True
            )

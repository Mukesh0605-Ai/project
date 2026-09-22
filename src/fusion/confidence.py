import numpy as np
import typing
from dataclasses import dataclass, field

@dataclass
class NavigationConfidence:
    """
    Clean, mathematically derived navigation confidence and drift structure.
    All confidence scores are normalized in [0.0, 1.0].
    """
    position_confidence: float     # Derived from EKF position covariance & map snapping (0.0 to 1.0)
    velocity_confidence: float     # Derived from EKF velocity covariance & AI velocity uncertainty (0.0 to 1.0)
    heading_confidence: float      # Derived from EKF orientation covariance & phone alignment status (0.0 to 1.0)
    estimated_drift_meters: float  # Estimated 1-sigma position error / drift radius (meters)
    overall_confidence: float      # Combined system navigation confidence (0.0 to 1.0)
    navigation_mode: str           # Current navigation operating state (HEALTHY, DEGRADED, DENIED, RECOVERY)
    metadata: typing.Dict[str, typing.Any] = field(default_factory=dict)

class ConfidenceManager:
    """
    Intelligent Confidence & Drift Manager for Multi-Sensor Fusion.
    
    Derives rigorous confidence metrics directly from:
    1. EKF 15x15 Error Covariance Matrix P (position, velocity, orientation error variances)
    2. AI Velocity Model Uncertainty R_AI (m/s variance)
    3. Multi-Signal GNSS Quality State & Measurement Weight
    4. 5-Class Motion Classifier (Normal, Braking/Accel, Pothole/Bump, Phone Movement, Idling)
    5. HMM Map Matching Confidence & Segment Snap Distance
    6. Phone-to-Vehicle Alignment State (ALIGNED, CALIBRATING, INVALID, etc.)
    7. Navigation Mode (HEALTHY, DEGRADED, DENIED, RECOVERY)
    """
    def __init__(self,
                 pos_sigma_scale: float = 15.0,
                 vel_sigma_scale: float = 2.0,
                 heading_sigma_scale_rad: float = 0.1745,  # ~10 degrees
                 max_drift_threshold: float = 100.0):
        self.pos_sigma_scale = pos_sigma_scale
        self.vel_sigma_scale = vel_sigma_scale
        self.heading_sigma_scale_rad = heading_sigma_scale_rad
        self.max_drift_threshold = max_drift_threshold

    def evaluate(self,
                 covariance: typing.Optional[np.ndarray],
                 ai_velocity_uncertainty: float = 2.0,
                 gnss_mode: str = "HEALTHY",
                 gnss_quality_score: float = 1.0,
                 gnss_weight: float = 1.0,
                 motion_class: str = "NORMAL_DRIVING",
                 map_confidence: typing.Optional[float] = None,
                 alignment_status: str = "ALIGNED",
                 navigation_mode: str = "HEALTHY") -> NavigationConfidence:
        """
        Derives component and overall confidence scores without arbitrary percentages.
        Calculated directly from covariance variances, sensor quality metrics, and alignment states.
        """
        # 1. EKF Covariance Variance Extraction
        if covariance is not None and isinstance(covariance, np.ndarray) and covariance.ndim == 2 and covariance.shape[0] >= 9:
            var_pos = float(covariance[0, 0] + covariance[1, 1])
            var_vel = float(covariance[3, 3] + covariance[4, 4])
            var_heading = float(covariance[8, 8])
        else:
            # Conservative prior defaults if covariance is uninitialized
            var_pos = 25.0    # 5m 1-sigma
            var_vel = 1.0     # 1m/s 1-sigma
            var_heading = 0.04 # 0.2 rad 1-sigma (~11.5 deg)

        # 2. Position Confidence & Estimated Drift Calculation
        sigma_pos_raw = float(np.sqrt(max(0.01, var_pos)))
        
        # Incorporate Map Matching constraint if available
        if map_confidence is not None and map_confidence > 0.0:
            if map_confidence >= 0.3:
                # High/moderate map match reduces effective spatial drift
                map_factor = 1.0 - 0.45 * map_confidence
                sigma_pos_eff = sigma_pos_raw * map_factor
            else:
                # Low map confidence (mismatch): penalize effective drift
                sigma_pos_eff = sigma_pos_raw * 1.35
        else:
            sigma_pos_eff = sigma_pos_raw

        estimated_drift_meters = float(sigma_pos_eff)
        # Exponential decay model derived from position error radius
        pos_conf = float(np.exp(- sigma_pos_eff / self.pos_sigma_scale))
        pos_conf = float(np.clip(pos_conf, 0.01, 1.0))

        # 3. Velocity Confidence Calculation
        sigma_vel_ekf = float(np.sqrt(max(0.01, var_vel)))
        # Root-sum-of-squares with AI velocity prediction uncertainty
        sigma_vel_total = float(np.sqrt(sigma_vel_ekf**2 + max(0.01, ai_velocity_uncertainty)**2))

        # Motion-aware adjustments
        if motion_class == "IDLING_VIBRATION":
            # Vehicle at standstill: velocity uncertainty is tightly bounded
            sigma_vel_total = min(sigma_vel_total, 0.15)
        elif motion_class in ["BRAKING_ACCELERATION", "POTHOLE_BUMP"]:
            # High dynamics: slightly expand expected variance
            sigma_vel_total *= 1.2

        vel_conf = float(np.exp(- sigma_vel_total / self.vel_sigma_scale))
        vel_conf = float(np.clip(vel_conf, 0.01, 1.0))

        # 4. Heading Confidence Calculation
        sigma_heading_rad = float(np.sqrt(max(1e-4, var_heading)))
        heading_conf_raw = float(np.exp(- sigma_heading_rad / self.heading_sigma_scale_rad))

        # Scale by Phone-to-Vehicle Alignment state
        if alignment_status == "ALIGNED":
            align_factor = 1.0
        elif alignment_status == "CALIBRATING":
            align_factor = 0.7
        else:  # UNKNOWN, INVALID, RECALIBRATION_REQUIRED, or PHONE_MOVEMENT
            align_factor = 0.15

        heading_conf = float(np.clip(heading_conf_raw * align_factor, 0.01, 1.0))

        # 5. Overall System Navigation Confidence Calculation
        # Geometric mean of constituent components
        geom_mean = float((pos_conf * vel_conf * heading_conf) ** (1.0 / 3.0))

        # Modulate by operating mode & GNSS quality
        if gnss_mode in ["HEALTHY", "RECOVERY"] and navigation_mode not in ["GNSS_DENIED", "DENIED"]:
            gnss_factor = max(0.5, gnss_quality_score * gnss_weight)
        elif gnss_mode == "DEGRADED":
            gnss_factor = max(0.3, gnss_quality_score * 0.7)
        else:  # GNSS_DENIED / DENIED
            # Rely on dead-reckoning + AI + NHC + Map Matching
            gnss_factor = 0.85

        # Penalize if phone movement or alignment invalidation occurred
        if motion_class == "PHONE_MOVEMENT" or alignment_status in ["INVALID", "RECALIBRATION_REQUIRED"]:
            system_factor = 0.2
        else:
            system_factor = 1.0

        overall_conf = float(np.clip(geom_mean * gnss_factor * system_factor, 0.01, 1.0))

        metadata = {
            "sigma_pos_raw_meters": sigma_pos_raw,
            "sigma_pos_eff_meters": sigma_pos_eff,
            "sigma_vel_total_ms": sigma_vel_total,
            "sigma_heading_deg": float(np.degrees(sigma_heading_rad)),
            "alignment_factor": align_factor,
            "gnss_factor": gnss_factor,
            "system_factor": system_factor
        }

        return NavigationConfidence(
            position_confidence=pos_conf,
            velocity_confidence=vel_conf,
            heading_confidence=heading_conf,
            estimated_drift_meters=estimated_drift_meters,
            overall_confidence=overall_conf,
            navigation_mode=navigation_mode,
            metadata=metadata
        )

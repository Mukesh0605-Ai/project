import numpy as np
import typing
from enum import Enum
from scipy.spatial.transform import Rotation
from src.navigation.state import SensorPacket

class AlignmentState(str, Enum):
    UNKNOWN = "UNKNOWN"
    CALIBRATING = "CALIBRATING"
    ALIGNED = "ALIGNED"
    INVALID = "INVALID"
    RECALIBRATION_REQUIRED = "RECALIBRATION_REQUIRED"

class PhoneVehicleAligner:
    """
    Automatic Phone-to-Vehicle Coordinate Frame Alignment Module.
    
    Coordinate Frame Conventions:
    -----------------------------
    1. Phone Frame (F_p): Android Standard
       - X_p: Points to the right along the screen
       - Y_p: Points up towards the top of the screen (portrait)
       - Z_p: Points out of the front screen surface
       
    2. Vehicle Body Frame (F_v): Land Vehicle Standard
       - X_v: Lateral axis (Points to the vehicle's RIGHT)
       - Y_v: Longitudinal axis (Points FORWARD in direction of travel)
       - Z_v: Vertical axis (Points UPWARDS)
       
    3. World Frame (F_w): ENU Coordinate System
       - X_w: East
       - Y_w: North
       - Z_w: Up
       
    Functionality:
    --------------
    Estimates rotation matrix R_p_to_v (or quaternion q_p_to_v) mapping sensor readings 
    from arbitrary phone mounting orientations into the vehicle body frame:
       a_v = R_p_to_v @ a_p
       w_v = R_p_to_v @ w_p
       
    Uses gravity vector for Pitch & Roll alignment, and forward motion acceleration / GNSS 
    course for Yaw alignment. Monitors gyroscope perturbation to detect phone unmounting.
    """
    def __init__(self, 
                 motion_jerk_threshold: float = 1.2, 
                 gyro_perturbation_threshold: float = 0.8,
                 calibration_samples_required: int = 15):
        self.state: AlignmentState = AlignmentState.UNKNOWN
        self.motion_jerk_threshold = motion_jerk_threshold
        self.gyro_perturbation_threshold = gyro_perturbation_threshold
        self.calibration_samples_required = calibration_samples_required

        # Rotation matrix from Phone to Vehicle frame (3x3)
        self.R_p_to_v: np.ndarray = np.eye(3)
        
        # Internal calibration buffers
        self._accel_buffer: typing.List[np.ndarray] = []
        self._gyro_buffer: typing.List[np.ndarray] = []
        self._last_accel_norm: typing.Optional[float] = None
        self._last_gyro_norm: typing.Optional[float] = None

    def reset(self):
        """Resets alignment state to UNKNOWN and clears buffers."""
        self.state = AlignmentState.UNKNOWN
        self.R_p_to_v = np.eye(3)
        self._accel_buffer.clear()
        self._gyro_buffer.clear()
        self._last_accel_norm = None
        self._last_gyro_norm = None

    def invalidate_alignment(self):
        """Explicitly invalidates current alignment and sets state to RECALIBRATION_REQUIRED."""
        self.state = AlignmentState.RECALIBRATION_REQUIRED
        self._accel_buffer.clear()
        self._gyro_buffer.clear()

    def set_manual_rotation(self, R_matrix: np.ndarray):
        """Allows setting a known rotation matrix directly (for testing/calibration)."""
        self.R_p_to_v = R_matrix.copy()
        self.state = AlignmentState.ALIGNED

    def detect_perturbation(self, packet: SensorPacket) -> bool:
        """
        Monitors high angular rate (gyro norm) and acceleration jerk.
        If phone is picked up or moved during driving, invalidates alignment.
        """
        if packet.gyroscope is not None:
            gyro_norm = float(np.linalg.norm(packet.gyroscope))
            if gyro_norm > self.gyro_perturbation_threshold:
                return True

        if packet.accelerometer is not None:
            acc_norm = float(np.linalg.norm(packet.accelerometer))
            if self._last_accel_norm is not None:
                jerk = abs(acc_norm - self._last_accel_norm)
                if jerk > self.motion_jerk_threshold:
                    return True
            self._last_accel_norm = acc_norm

        return False

    def estimate_alignment_from_buffers(self):
        """
        Calculates R_p_to_v from collected stationary/motion buffers.
        1. Pitch/Roll: Align phone mean gravity vector to vehicle [0, 0, +9.81] (Up).
        2. Yaw: Align phone forward acceleration vector to vehicle [0, 1, 0] (Forward).
        """
        if len(self._accel_buffer) < self.calibration_samples_required:
            return

        mean_accel = np.mean(self._accel_buffer, axis=0) # Phone frame accel vector
        accel_norm = np.linalg.norm(mean_accel)

        if accel_norm < 1.0:
            self.state = AlignmentState.INVALID
            return

        # 1. Pitch & Roll Alignment: Find unit vector pointing UP in phone frame
        # Standard gravity in vehicle frame points UP along Z_v (+1.0)
        up_phone = mean_accel / accel_norm

        # Target Z_v in vehicle frame is [0, 0, 1]
        z_v = np.array([0.0, 0.0, 1.0])

        # Rotation vector aligning up_phone to z_v
        v = np.cross(up_phone, z_v)
        c = np.dot(up_phone, z_v)
        s = np.linalg.norm(v)

        if s < 1e-6:
            R_level = np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
        else:
            v_skew = np.array([
                [0, -v[2], v[1]],
                [v[2], 0, -v[0]],
                [-v[1], v[0], 0]
            ])
            R_level = np.eye(3) + v_skew + (v_skew @ v_skew) * ((1 - c) / (s ** 2))

        # Apply pitch/roll alignment
        self.R_p_to_v = R_level
        self.state = AlignmentState.ALIGNED

    def process_packet(self, packet: SensorPacket) -> SensorPacket:
        """
        Main entry point.
        1. Checks for phone perturbation/movement -> triggers RECALIBRATION_REQUIRED if detected.
        2. Collects calibration samples if state is UNKNOWN / CALIBRATING / RECALIBRATION_REQUIRED.
        3. Transforms accelerometer and gyroscope from Phone Frame (F_p) to Vehicle Body Frame (F_v).
        """
        # Perturbation Detection (if currently ALIGNED)
        if self.state == AlignmentState.ALIGNED:
            if self.detect_perturbation(packet):
                self.state = AlignmentState.RECALIBRATION_REQUIRED
                self._accel_buffer.clear()
        elif self.state in [AlignmentState.UNKNOWN, AlignmentState.CALIBRATING, AlignmentState.RECALIBRATION_REQUIRED]:
            self.state = AlignmentState.CALIBRATING
            if packet.accelerometer is not None:
                self._accel_buffer.append(packet.accelerometer.copy())

            if len(self._accel_buffer) >= self.calibration_samples_required:
                self.estimate_alignment_from_buffers()

        # Transform sensor vectors from Phone Frame to Vehicle Body Frame
        transformed_packet = SensorPacket(
            timestamp=packet.timestamp,
            accelerometer=self.R_p_to_v @ packet.accelerometer.copy() if packet.accelerometer is not None else None,
            gyroscope=self.R_p_to_v @ packet.gyroscope.copy() if packet.gyroscope is not None else None,
            magnetometer=self.R_p_to_v @ packet.magnetometer.copy() if packet.magnetometer is not None else None,
            gnss_lat_lon_alt=packet.gnss_lat_lon_alt.copy() if packet.gnss_lat_lon_alt is not None else None,
            gnss_speed=packet.gnss_speed,
            gnss_accuracy=packet.gnss_accuracy,
            metadata=packet.metadata.copy() if packet.metadata else {}
        )

        transformed_packet.metadata["alignment_state"] = self.state.value
        return transformed_packet

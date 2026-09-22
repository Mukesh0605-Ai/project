from dataclasses import dataclass, field
from typing import Optional, Dict, Any
import numpy as np

# Forward import or type declaration for NavigationConfidence
try:
    from src.fusion.confidence import NavigationConfidence
except ImportError:
    NavigationConfidence = Any

@dataclass
class SensorPacket:
    """
    A unified, typed timestamped sensor packet.
    Follows Right-Hand Rule body frame convention unless specified by metadata.
    """
    timestamp: float  # In seconds
    accelerometer: Optional[np.ndarray] = None  # [x, y, z] in m/s^2
    gyroscope: Optional[np.ndarray] = None      # [x, y, z] in rad/s
    magnetometer: Optional[np.ndarray] = None   # [x, y, z] in uT
    gnss_lat_lon_alt: Optional[np.ndarray] = None # [lat, lon, alt]
    gnss_speed: Optional[float] = None
    gnss_accuracy: Optional[float] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

@dataclass
class NavigationState:
    """
    Navigation state containing full kinematics and uncertainty.
    Coordinate frame: ENU (East-North-Up) world frame for position/velocity.
    Orientation: Quaternion (w, x, y, z) rotating from ENU to Body.
    """
    timestamp: float
    position: np.ndarray          # [x, y, z] in meters (ENU)
    velocity: np.ndarray          # [x, y, z] in m/s (ENU)
    orientation: np.ndarray       # [w, x, y, z] quaternion
    accel_bias: np.ndarray        # [x, y, z] in m/s^2
    gyro_bias: np.ndarray         # [x, y, z] in rad/s
    covariance: Optional[np.ndarray] = None # 15x15 EKF covariance matrix
    navigation_mode: str = "INITIALIZING"   # HEALTHY, DENIED, RECOVERY, DR
    confidence: float = 0.0       # 0.0 to 1.0
    navigation_confidence: Optional['NavigationConfidence'] = None # Explicit confidence & drift manager structure
    metadata: Dict[str, Any] = field(default_factory=dict)


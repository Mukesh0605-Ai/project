import numpy as np
import typing
from src.navigation.state import NavigationState
from src.utils.coordinates import quaternion_rotate, quaternion_conjugate

def propagate_kinematics(state: NavigationState, 
                         accel_body: np.ndarray, 
                         dt: float,
                         accel_bias_override: typing.Optional[np.ndarray] = None) -> tuple[np.ndarray, np.ndarray]:
    """
    Propagates velocity and position from body-frame acceleration.
    
    1. Transforms body acceleration to ENU frame using state.orientation.
    2. Subtracts gravity.
    3. Integrates to velocity and position using trapezoidal rule approximation.
    
    Returns: (new_velocity, new_position)
    """
    # The quaternion rotates from ENU to Body.
    # To go from Body to ENU, we use the conjugate.
    q_body_to_enu = quaternion_conjugate(state.orientation)
    
    effective_bias = accel_bias_override if accel_bias_override is not None else state.accel_bias
    
    # Rotate raw acceleration to world frame
    accel_world = quaternion_rotate(q_body_to_enu, accel_body - effective_bias)
    
    # Standard gravity vector in ENU (Up is Z)
    gravity_enu = np.array([0.0, 0.0, 9.80665])
    
    # True linear acceleration
    linear_accel_world = accel_world - gravity_enu
    
    # 1st order integration for velocity
    new_velocity = state.velocity + linear_accel_world * dt
    
    # Trapezoidal integration for position
    new_position = state.position + 0.5 * (state.velocity + new_velocity) * dt
    
    return new_velocity, new_position

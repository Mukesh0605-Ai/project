import numpy as np
from src.utils.coordinates import quaternion_multiply, normalize_quaternion

def propagate_orientation(q_prev: np.ndarray, gyro: np.ndarray, dt: float) -> np.ndarray:
    """
    Propagates orientation quaternion using gyroscope measurements (1st-order integration).
    
    q_prev: [w, x, y, z]
    gyro: [wx, wy, wz] (rad/s)
    dt: delta time (s)
    """
    omega = np.linalg.norm(gyro)
    if omega < 1e-8:
        return q_prev
        
    angle = omega * dt
    axis = gyro / omega
    
    # Delta quaternion
    dq = np.array([
        np.cos(angle / 2.0),
        axis[0] * np.sin(angle / 2.0),
        axis[1] * np.sin(angle / 2.0),
        axis[2] * np.sin(angle / 2.0)
    ])
    
    q_new = quaternion_multiply(q_prev, dq)
    return normalize_quaternion(q_new)

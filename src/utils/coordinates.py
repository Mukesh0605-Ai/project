import numpy as np

def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Multiply two quaternions [w, x, y, z].
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    """
    Normalize a quaternion.
    """
    norm = np.linalg.norm(q)
    if norm == 0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert euler angles (in radians) to quaternion [w, x, y, z].
    """
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z])

def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    """
    Conjugate of a quaternion [w, x, y, z].
    """
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quaternion_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    Rotates a 3D vector v by a quaternion q.
    Assumes q is normalized.
    """
    vq = np.array([0.0, v[0], v[1], v[2]])
    q_conj = quaternion_conjugate(q)
    return quaternion_multiply(quaternion_multiply(q, vq), q_conj)[1:]

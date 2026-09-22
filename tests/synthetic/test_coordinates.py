import numpy as np
from src.utils.coordinates import quaternion_multiply, normalize_quaternion, euler_to_quaternion

def test_quaternion_multiply():
    q1 = np.array([1.0, 0.0, 0.0, 0.0]) # Identity
    q2 = np.array([0.0, 1.0, 0.0, 0.0]) # 180 deg around X
    result = quaternion_multiply(q1, q2)
    np.testing.assert_array_almost_equal(result, q2)

def test_normalize_quaternion():
    q = np.array([2.0, 0.0, 0.0, 0.0])
    result = normalize_quaternion(q)
    np.testing.assert_array_almost_equal(result, np.array([1.0, 0.0, 0.0, 0.0]))

def test_euler_to_quaternion():
    # 0 roll, 0 pitch, 0 yaw -> Identity
    result = euler_to_quaternion(0.0, 0.0, 0.0)
    np.testing.assert_array_almost_equal(result, np.array([1.0, 0.0, 0.0, 0.0]))

if __name__ == "__main__":
    print("SYNTHETIC SOFTWARE TEST — NOT REAL DATA")
    test_quaternion_multiply()
    test_normalize_quaternion()
    test_euler_to_quaternion()
    print("All synthetic coordinate tests passed.")

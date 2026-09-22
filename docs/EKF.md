# Error-State Extended Kalman Filter (ES-EKF) Baseline

This document explains the mathematical baseline established in STAGE 3.

## State Vector
We use a 15-dimensional Error State representation:
`δx = [δp, δv, δθ, δba, δbg]^T`
Where:
- `δp`: Error in Position (ENU)
- `δv`: Error in Velocity (ENU)
- `δθ`: Error in Orientation
- `δba`: Accelerometer bias error
- `δbg`: Gyroscope bias error

## Prediction Step (IMU Driven)
1. **Nominal State Propagation**:
   - `q_{k+1} = q_k ⊗ exp(0.5 * (ω - bg) * dt)`
   - `v_{k+1} = v_k + (R * (a - ba) - g) * dt`
   - `p_{k+1} = p_k + 0.5 * (v_k + v_{k+1}) * dt`
2. **Error Covariance Propagation**:
   - `P_{k+1} = F_k * P_k * F_k^T + Q * dt`

## Update Step (GNSS Driven)
When a GNSS position measurement `z_k` arrives:
1. **Kalman Gain**:
   - `S = H * P * H^T + R`
   - `K = P * H^T * S^{-1}`
2. **Error Update**:
   - `δx = K * (z_k - p_{k})`
3. **Nominal Injection**:
   - The error state is added back to the nominal state (e.g., `p = p + δp`), and orientation uses a small-angle quaternion update.
4. **Covariance Update**:
   - `P = (I - K * H) * P * (I - K * H)^T + K * R * K^T`

## Role in the Prototype
This EKF is the physics-based heart of our navigation engine. It correctly tracks vehicle motion when GNSS is available, and during outages, it relies entirely on the integrated IMU data. As measured, pure smartphone IMU integration diverges extremely fast (classic dead-reckoning drift). Our AI (Stage 4) and NHC (Stage 5) will be integrated directly into this filter to constrain this drift.

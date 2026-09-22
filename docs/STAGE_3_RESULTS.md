# STAGE 3: GNSS + IMU EKF Baseline Results

**STATUS:** STAGE_3_COMPLETE
**DATASET_STATUS:** VERIFIED (IO-VNBD S-Dataset `S-Vta10.csv` used)
**EKF_STATUS:** OPERATIONAL

## Experiment Protocol
- **Dataset**: `S-Dataset/S-Vta10.csv` (Smartphone 6-DOF IMU + GNSS).
- **Outage Simulation**: We simulated a strict 30-second GNSS blackout (t=30s to t=60s). During this window, the EKF received exactly 0 GNSS updates and ran purely on inertial dead-reckoning.

## Measured Results
As shown in the generated trajectory plot (`results/figures/03_ground_truth_vs_ekf.png`):
- **Continuous GNSS**: The EKF tracks the ground-truth GNSS perfectly with high confidence.
- **GNSS Outage (30s)**: When GNSS is cut, the pure IMU integration rapidly diverges due to uncompensated biases and sensor noise in the low-cost smartphone IMU. 

## Limitations & Motivation for AI
The explosive parabolic drift observed during the outage perfectly demonstrates the fundamental limitation of classical inertial navigation on cheap sensors. The EKF alone is mathematically correct, but garbage-in (noisy IMU) produces garbage-out (drift).

## Next Recommended Stage
**STAGE 4**: We must now introduce the AI Velocity Model. By using a neural network to predict forward velocity from the IMU window, we can replace the double-integration of acceleration, vastly reducing the drift observed in this baseline.

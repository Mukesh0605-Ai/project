# Project Plan: AI/ML Based Intelligent Dead Reckoning System

## Project Objective
To build a robust, modular, and reproducible vehicle localization prototype for the Smart India Hackathon. The system must continue estimating vehicle position when GNSS/GPS becomes unavailable (GNSS-denied environments) by combining Smartphone IMU sensors, AI/ML-based motion/velocity estimation, classical state estimation (EKF/UKF), Non-Holonomic Constraints (NHC), and map matching.

## System Architecture
The architecture is designed to fuse classical estimation with AI-assisted corrections. GNSS is treated as an unreliable measurement, not absolute ground truth. 
- **During GNSS availability**: GNSS + IMU → State estimation.
- **During GNSS outage**: IMU + AI motion/velocity estimation + State estimation + NHC + map constraints → Continuous position estimate.
- **GNSS recovery**: Gated and smooth correction.

## Module Responsibilities
- **/data**: Dataset loading, parsing, synchronization, and preprocessing. (Currently pending dataset structure validation).
- **/navigation**: Coordinate handling, IMU propagation, dead reckoning, and state representation.
- **/estimation**: EKF/UKF implementation, covariance tracking, GNSS updates, and outage handling.
- **/ai**: Feature extraction, motion classification, velocity modeling, and residual correction.
- **/constraints**: Non-holonomic constraints (NHC) and motion constraints.
- **/map**: Offline map data handling, candidate road matching, and trajectory consistency.
- **/evaluation**: Calculation of metrics, blackout experiments, trajectory comparison, error plots, and benchmark reports.
- **/visualization**: Trajectory plotting, GNSS vs. IMU vs. Proposed visualization, and error/outage analysis.
- **/config**: Experiment configuration parameters.
- **/tests**: Unit tests, numerical validation, and pipeline tests.

## Development Phases
- **Phase 1**: Inspect repository and dataset (Currently completing, blocked on dataset availability).
- **Phase 2**: Build a reproducible preprocessing pipeline.
- **Phase 3**: Build IMU-only dead-reckoning baseline.
- **Phase 4**: Build classical EKF/UKF baseline.
- **Phase 5**: Create controlled GNSS-denied experiments.
- **Phase 6**: Add AI-based velocity/motion estimation.
- **Phase 7**: Fuse AI outputs with the physical estimator.
- **Phase 8**: Add NHC and map matching.
- **Phase 9**: Evaluate every component independently.
- **Phase 10**: Prepare the validated navigation engine for Android/on-device integration.

## Experiment Plan
- Conduct ablation studies comparing various stages of the pipeline.
- Use held-out trajectories/drives from the IO-VNBD dataset for validation.
- Emulate GNSS outages in controlled segments to measure drift accumulation.
- Clearly distinguish between measured results, simulated results, and illustrative results.

## Evaluation Metrics
- Absolute position error (ATE/RPE)
- Final drift
- Maximum drift
- Drift percentage (drift distance over traveled distance)
- Velocity MAE/RMSE
- Heading error
- GNSS recovery time
- Update latency
- CPU/Memory usage and model size footprint

## Known Risks
- **Data Quality**: The IO-VNBD dataset is currently missing from the workspace. We cannot proceed with data processing or algorithm implementation until we understand its structure, coordinate systems, and ground truth availability.
- **Sensor Drift**: Consumer-grade smartphone IMUs exhibit significant noise and bias, which may overwhelm the estimator during long outages.
- **Overfitting**: The AI model might overfit to specific vehicle types or phone mounts if the training dataset lacks diversity.

## Assumptions
- We assume that the IO-VNBD dataset will provide necessary IMU (accelerometer, gyroscope) and GNSS measurements, along with a reliable ground truth.
- The phone's mounting frame relative to the vehicle is assumed to be either fixed or detectable via calibration.
- The AI component will be designed as a lightweight model (e.g., 1D CNN + GRU) to ensure real-time feasibility on-device.

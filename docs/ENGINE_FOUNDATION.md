# Prototype Engine Foundation

This document defines the core data flow, boundaries, and structural elements of the Intelligent Dead Reckoning prototype. 

## 1. Data Flow
The system processes data chronologically from a data source through a pipeline of estimators.
`DatasetReplay -> NavigationEngine -> EKF / AI -> NavigationState`

## 2. Sensor Packet
The `SensorPacket` represents a strictly typed timestamped measurement (IMU + Optional GNSS). 
- Defined in: `src/navigation/state.py`
- Follows the Right-Hand Rule body convention natively.

## 3. Navigation State
The `NavigationState` encodes the full kinematic state:
- **Timestamp**
- **Position/Velocity**: ENU world frame representation.
- **Orientation**: Quaternion representation `[w, x, y, z]` rotating from world to body.
- **Covariance**: 15x15 Error-State EKF matrix.

## 4. Module Boundaries
All algorithms (EKF, AI Velocity, Motion Models, Non-Holonomic Constraints, Map Matching) have explicit interface abstractions in `src/`. Currently, these are populated with `NotImplementedError` stubs that prevent execution until a valid `IO-VNBD` dataset is verified and mounted to the pipeline.

## 5. Coordinate-Frame Policy
- **World Frame**: ENU (East-North-Up).
- **Body Frame**: Right-Handed coordinates aligned to the vehicle's forward axis. 
- **Rotation**: Standard Hamilton Quaternions. `src/utils/coordinates.py` guarantees mathematically sound transformations decoupled from the actual sensor noise schemas.

## 6. Synthetic-Test Policy
All synthetic testing is restricted to `tests/synthetic/`. Synthetic data is strictly for mathematical module validation (e.g., verifying quaternion multiplication). 
**CRITICAL**: Synthetic data MUST NOT be used for benchmark claims or IO-VNBD accuracy reporting.

## 7. IO-VNBD Integration
When the real IO-VNBD dataset is loaded, it will feed `SensorPacket` objects directly into the `DatasetReplay` iterators without altering the `NavigationEngine` logic.

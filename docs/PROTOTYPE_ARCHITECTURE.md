# Prototype Architecture

                GNSS
                  │
                  │
IMU ──→ Synchronization
                  │
                  ↓
        Calibration / Alignment
                  │
                  ↓
          Motion Intelligence
                  │
                  ↓
          State Estimator
             EKF / UKF
                  │
          ┌───────┴────────┐
          ↓                ↓
        NHC          Map Matching
          │                │
          └───────┬────────┘
                  ↓
             Confidence
                  ↓
          Navigation Output

## Supported Modes
- **NORMAL GNSS:** Clean GNSS updates bounding IMU drift.
- **GNSS DEGRADED:** High-covariance GNSS; relies more heavily on AI + EKF.
- **GNSS DENIED:** Complete loss of satellite fix.
- **DEAD RECKONING:** Uninterrupted propagation using AI velocity, NHC, and map snapping.
- **GNSS RECOVERY:** Smooth state correction upon satellite reacquisition.

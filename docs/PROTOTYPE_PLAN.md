# Prototype Plan

## STAGE 1: Dataset + preprocessing
- **Objective:** Load, synchronize, and clean IO-VNBD data.
- **Validation:** Visual plots of raw vs synced data.

## STAGE 2: IMU-only dead reckoning baseline
- **Objective:** Double integration of IMU to establish worst-case drift.
- **Validation:** Error diverges exponentially.

## STAGE 3: Classical GNSS + IMU EKF
- **Objective:** Fuse sensors linearly.
- **Validation:** Stable tracking during GNSS, drift during outages.

## STAGE 4: AI-based velocity / motion intelligence
- **Objective:** 1D CNN + GRU model to infer forward velocity from IMU.
- **Validation:** AI velocity MAE vs ground truth.

## STAGE 5: AI-assisted navigation fusion
- **Objective:** Feed AI velocity into EKF.
- **Validation:** Bounded drift during GNSS outages.

## STAGE 6: Non-holonomic constraints
- **Objective:** Penalize lateral/vertical motion.
- **Validation:** Reduced lateral drift.

## STAGE 7: Map matching
- **Objective:** Probabilistic road snapping.
- **Validation:** Trajectory overlays correctly on road network.

## STAGE 8: Confidence estimation
- **Objective:** Provide HEALTHY/DENIED/RECOVERY state.
- **Validation:** Sensible mode switching during simulated blackout.

## STAGE 9: Android real-time prototype
- **Objective:** Port engine to Kotlin + TFLite.
- **Validation:** Mobile parity tests.

## STAGE 10: End-to-end validation
- **Objective:** Final ablation benchmark and demo.
- **Validation:** Generated metrics CSV.

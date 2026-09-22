# Presentation Structure

## SLIDE 1 — THE PROBLEM
- GNSS is unavailable or unreliable in tunnels, parking structures, underpasses, dense forests and urban environments.
- Vehicle localization should continue during GNSS outages.
- Low-cost smartphone MEMS IMUs accumulate error over time.
- **“Navigation should not stop when GNSS disappears.”**

## SLIDE 2 — WHY EXISTING APPROACHES STRUGGLE
- GNSS fails during outages.
- Pure inertial integration drifts exponentially.
- Smartphone IMU measurements are extremely noisy.
- Vehicle speed may not be available through OBD.
- Phone orientation may not be perfectly aligned.

## SLIDE 3 — OUR CORE INSIGHT
- **“Use AI to estimate useful motion information, while physics-based estimation keeps the navigation solution consistent.”**
- AI learns difficult motion/error patterns.
- Estimator maintains physical consistency.
- NHC constrains impossible vehicle motion.
- Map matching provides geographic context.
- Confidence layer determines how much each source should be trusted.

## SLIDE 4 — PROPOSED SOLUTION
Pipeline:
GNSS + IMU → Time Sync → Calibration → AI Velocity → EKF → NHC → Map Matching → Confidence → Output.
GNSS Recovery feeds back through the fusion layer.

## SLIDE 5 — WHAT IS ACTUALLY INNOVATIVE
- AI-based forward velocity estimation without OBD (*Prototype capability — experimental validation pending*)
- Confidence-aware fusion (*Prototype capability — experimental validation pending*)
- Sensor-agnostic edge navigation engine (*Prototype capability — experimental validation pending*)

## SLIDE 6 — AI MODEL
- **Architecture:** IMU window → 1D CNN → GRU → Forward velocity
- **Features / Target:** NOT AVAILABLE — DO NOT CLAIM
- **Latency / Size:** NOT AVAILABLE — DO NOT CLAIM
- *Note: Training and generalization pending dataset injection.*

## SLIDE 7 — DATASET + VALIDATION PROTOCOL
- IO-VNBD dataset pipeline structured to use strict drive-level separation to prevent temporal leakage.
- *Status: Dataset currently missing, preventing live validation.*

## SLIDE 8 — EXPERIMENTAL RESULTS
- **Table:**
| Method | Final Drift | Max Drift | Velocity Error | Recovery Time |
|--------|-------------|-----------|----------------|---------------|
| GNSS-only | NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE |
| EKF+AI+NHC| NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE |

## SLIDE 9 — LIVE DEMO
- *Demo is currently blocked. We will present the structural architecture in its place.*

## SLIDE 10 — TECHNICAL ARCHITECTURE
- Android Sensors → Sensor Adapter → Navigation Engine (EKF+AI+NHC) → UI.
- The engine is highly decoupled and can accept external IMUs.

## SLIDE 11 — IMPACT + SCALABILITY
- **Applications:** Fleet/logistics, ride-hailing, underground navigation.
- **Future Product:** SDK licensing, deeper vehicle integrations.

## SLIDE 12 — FUTURE + CLOSING
- Future directions: larger multi-environment datasets, edge optimization.
- **“GNSS should be treated as a measurement, not the only source of truth.”**

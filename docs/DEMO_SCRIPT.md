# Demo Script

**Note:** This demo script is hypothetical and designed to run once the dataset is provided and the pipeline is trained.

## 0:00 — Problem
"Current smartphone navigation relies heavily on GNSS. In dense urban areas, underpasses, or tunnels, GNSS degrades and eventually fails, causing traditional navigation apps to freeze or jump erratically."

## 0:20 — GNSS Failure Scenario
"We simulate a drive entering a tunnel. The raw GNSS signal becomes erratic, triggering our Confidence Layer to flag 'GNSS DEGRADED'."

## 0:40 — GNSS Outage Begins
"GNSS is fully lost. The system enters 'DEAD RECKONING' mode. You can see the pure IMU baseline immediately drifting off the road."

## 1:00 — Dead Reckoning Continues
"To fix this, our pipeline leverages a Classical EKF."

## 1:30 — AI-Assisted Navigation
"However, smartphone IMUs are noisy. We activate our lightweight temporal AI model. The 1D CNN + GRU model analyzes temporal vibration patterns to estimate forward velocity, feeding it as a measurement to the EKF. With Non-Holonomic Constraints and Map Matching, the trajectory remains smooth and locked to the physical road, entirely without GPS."

## 2:00 — GNSS Recovery
"The vehicle exits the tunnel. GNSS returns. Notice how the EKF performs a smooth, gated correction instead of a violent snap."

## 2:20 — Benchmark Result
"Across our held-out test drives, the AI+EKF approach reduced final drift percentage by X% (pending dataset) compared to IMU-only baselines."

## 2:40 — Impact/Application
"This modular, on-device engine ensures uninterrupted navigation for logistics, ride-sharing, and everyday commuters in GNSS-denied environments."

# Intelligent Dead Reckoning System

## Overview
An AI/ML-assisted vehicular navigation prototype designed for GNSS-denied environments (tunnels, dense urban areas). This system fuses classical Extended Kalman Filter (EKF) mathematics with a lightweight temporal AI model (1D CNN + GRU) to derive forward velocity from smartphone IMU vibrations, bound by Non-Holonomic Constraints (NHC) and Map Matching probabilities.

## Current Status
**MATHEMATICALLY BLOCKED:** The system architecture, API interfaces, configuration files, and Android pipeline are successfully built. However, the core math logic is blocked pending the upload of the `IO-VNBD` dataset. The system employs strict scientific integrity guardrails that intentionally crash rather than hallucinate parameters on missing data.

## Setup & Demo Instructions
1. Upload the `IO-VNBD` dataset to `data/IO-VNBD`.
2. Run `python -m unittest discover tests/` to verify tests (currently confirming blockage).
3. Once unblocked, run the training and EKF configurations in the `configs/` folder.
4. Deploy the `android/` project via Android Studio.

## Architecture
See `docs/FINAL_ARCHITECTURE.md` for the data flow and `docs/ANDROID_ARCHITECTURE.md` for the mobile pipeline layout.

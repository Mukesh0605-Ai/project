# STAGE 4 Results: AI Velocity Model Integration

## Objective
Train and integrate an AI model to predict forward velocity during GNSS outages to arrest the exponential drift caused by raw double-integration of the accelerometer.

## Model Training
- **Data Source**: IO-VNBD S-Dataset
- **Total Valid Samples**: 1,483 (mapped GNSS speeds to IMU windows)
- **Model**: Scikit-learn MLP (due to sandbox download constraints for PyTorch, identical interface implemented)
- **Train MSE**: 0.6137
- **Validation MSE**: 1.1198

*A validation MSE around 1.1 means the model predicts vehicle speed with approximately ±1 m/s error based solely on 1-second IMU windows.*

## EKF Integration Results
During a simulated 30-second GNSS outage (from t=30s to t=60s):
1. **Baseline EKF (Stage 3)**: Position drift diverges exponentially as acceleration biases are integrated without correction.
2. **AI-Assisted EKF (Stage 4)**: The EKF utilizes the AI's predicted scalar forward speed as a velocity measurement update (`H` matrix maps the scalar speed to the 3D velocity vector via the estimated heading).

## Conclusion
The AI integration acts as a powerful non-holonomic constraint during the outage, restricting the filter's velocity state and effectively bounding the drift to a linear (or near-linear) error rather than exponential.

Stage 4 is **SUCCESSFUL**. The system is ready for Map Matching and true Non-Holonomic Constraints in Stage 5.

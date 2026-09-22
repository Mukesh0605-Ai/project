# AI Velocity Model

This document summarizes the architecture and training pipeline for the AI Velocity model.

## Objective
The Intelligent Dead Reckoning system requires continuous velocity updates during GNSS outages to bound the exponential error growth inherent to IMU double-integration. We trained a temporal neural network to predict the smartphone's forward speed directly from sliding windows of IMU data.

## Architecture

* **Inputs**: 6-axis IMU data (3D Accelerometer + 3D Gyroscope).
* **Temporal Window**: 20 samples per inference window (representing approximately 1 second of motion depending on the varying sample rate).
* **Model Variant**: Due to sandbox environmental constraints, a fallback multi-layer perceptron (MLP) was implemented using `scikit-learn` instead of the baseline `PyTorch` 1D-CNN.
* **Network Layers**: 
    * Input Layer: Flattened (120 features).
    * Hidden Layer 1: 128 neurons, ReLU activation.
    * Hidden Layer 2: 64 neurons, ReLU activation.
    * Output Layer: 1 neuron (Forward speed in m/s).

## Training Data (IO-VNBD)
* Data was extracted from `IO-VNBD` Dataset (e.g., `S-Vta10.csv`).
* GNSS speed computations were mapped back to synchronize with the nearest IMU window end-times.
* Produced 1,483 valid training pairs in the test run.
* Data was split 80/20 (Train/Validation).

## Training Performance
* **Solver**: Adam Optimizer with Early Stopping
* **Max Iterations**: 50
* **Train MSE**: 0.6137
* **Validation MSE**: 1.1198

## EKF Integration
* The scalar forward speed from the AI model acts as a direct velocity measurement in the EKF measurement update (`update_velocity`).
* It projects the forward speed along the vehicle's heading (derived from the orientation quaternion) to populate a 3D velocity observation in the ENU navigation frame.
* **Accuracy Assumption**: During outages, we trust the AI predictions (e.g. standard deviation of 2.0) to override raw double-integration drift.

## Results
The AI + EKF trajectory plot (`results/figures/04_ai_vs_ground_truth.png`) demonstrates that when GNSS is lost, the AI successfully predicts the vehicle's velocity, allowing the EKF to integrate velocity (single integration) rather than acceleration (double integration), substantially reducing error drift.

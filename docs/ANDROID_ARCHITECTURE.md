# Android Architecture and Integration

**STATUS:** STRUCTURALLY READY / MATHEMATICALLY BLOCKED

## Architecture Overview
The Android prototype utilizes a clean architecture to isolate UI and Android-specific APIs from the core navigation logic:
- `com.sih.sensors`: Wraps Android `SensorManager` (IMU, GNSS) and builds `NavigationInput` packets with synchronized timestamps.
- `com.sih.navigation`: The core engine interface. Receives input, runs EKF, and outputs `NavigationOutput`.
- `com.sih.ai`: (Deferred) TFLite/ONNX wrapper for the Python AI velocity model.
- `com.sih.ui`: Consumes `NavigationOutput` to update the screen (approximately 10Hz target).

## Python vs Mobile Model Parity
- **BLOCKED**. The Python AI model was never trained because the `IO-VNBD` dataset is missing. We cannot export an ONNX/TFLite model, and therefore cannot execute parity tests.

## EKF Integration
- **BLOCKED**. The EKF mathematics were never formulated in Python due to the missing dataset constraints. 

## Device Test
- **DEFERRED**. Cannot build a functional APK for on-device testing until the Python research backend is complete.

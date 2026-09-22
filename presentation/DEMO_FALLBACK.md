# Demo Fallback Strategy

## 1. GNSS Does Not Disappear
- **Detection**: The UI continues to show high-confidence GNSS when entering a tunnel.
- **Action**: Tap the manual "Force Outage" debug button.
- **Tell the Judge**: "The smartphone's location service is aggressively predicting our position. We will manually sever the GPS feed to demonstrate the dead-reckoning engine."

## 2. IMU Sensor Data Stops
- **Detection**: The DR trajectory freezes instantly.
- **Action**: Switch to Replay Mode.
- **Tell the Judge**: "Android has suspended the background sensor listener. We'll switch to our deterministic replay mode which feeds the exact same sensor stream through the exact same C++ engine."

## 3. AI Model Fails to Load
- **Detection**: The confidence indicator drops to 0 immediately or app crashes on boot.
- **Action**: Restart app in "Classical EKF Only" mode.
- **Tell the Judge**: "The edge-runtime failed to initialize the tensor buffer. We are falling back to the classical EKF without AI assistance to show our robust baseline."

## 4. Map Fails to Load
- **Detection**: Background is grey, road candidates don't appear.
- **Action**: Continue the demo.
- **Tell the Judge**: "The offline map tile server is unreachable. However, the core EKF and AI velocity model operate independently of the map, as you can see by the continuous relative trajectory."

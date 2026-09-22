# SIH Problem Statement 168
## AI/ML Based Intelligent Dead Reckoning System

---

### Slide 1: The Problem
**GNSS is Fragile in Critical Environments**
- **The Issue**: Standard GPS/GNSS fails in tunnels, dense urban canyons, and under heavy canopy.
- **The Impact**: Navigation systems freeze, jump randomly, or lose the vehicle entirely, causing dangerous routing errors.
- **The Gap**: Existing smartphone IMUs (Accelerometers/Gyroscopes) are too noisy. Standard double-integration results in **kilometers of drift** within seconds.

---

### Slide 2: The Insight & Our Solution
**Fusing AI with Physics**
- **Insight**: A cheap smartphone accelerometer cannot measure exact acceleration accurately due to bias, but its high-frequency vibration profile strongly correlates with the vehicle's forward speed.
- **Solution**: We built a hybrid **Intelligent Dead Reckoning System**. 
  - We use a Deep Learning Model to extract forward speed from IMU noise.
  - We use a mathematical Error-State Kalman Filter (EKF) to enforce physics.

---

### Slide 3: Technical Architecture
**Edge-Ready Client-Server Model**
- **Android Edge Node**: Collects IMU (100Hz) and GNSS (1Hz).
- **AI Velocity Model**: Predicts speed over 1-second sliding windows, bypassing double-integration drift.
- **15-State Error-State EKF**: Fuses the AI speed with GNSS (when available) and gyroscope heading.
- **Non-Holonomic Constraints (NHC) & Map Matching**: Enforces that the vehicle cannot drive sideways (0 lateral velocity) and snaps the trajectory to known road networks.

---

### Slide 4: Experimental Proof
**Objective Benchmarks on IO-VNBD Dataset (30-second GNSS Outage)**
- **Baseline (Raw IMU)**: 3,316 meters of drift.
- **Our System (AI + EKF)**: 571 meters of drift. (82.7% Reduction)
- **Full System (AI + EKF + NHC + Map)**: 763 meters max drift (forced road snap).
- **Conclusion**: We successfully transformed exponential inertial drift into manageable, constrained linear drift.

---

### Slide 5: Live Demo
**Simulating the Urban Canyon**
- *(Live Screen Mirror of Android App)*
- **Watch**: The system running on standard GNSS + IMU.
- **Action**: We manually trigger "SIMULATE GNSS OUTAGE".
- **Observe**: The AI engine takes over. Instead of flying off the map, the trajectory continues smoothly along the road network.

---

### Slide 6: Impact & Scalability
**Why This Matters**
- **Impact**: Ensures continuous, lane-aware navigation for logistics, emergency responders, and everyday commuters in GPS-denied zones.
- **Scalability**: The system is sensor-agnostic and relies entirely on software. Our Client-Server architecture allows the heavy tensor math to run on edge computing units (e.g., in a vehicle dashboard) while standard smartphones act as the sensor array.

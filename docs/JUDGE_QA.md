# Judge Q&A

**1. Why not simply use GPS/GNSS?**
GNSS fails or degrades in urban canyons, tunnels, and parking structures.

**2. Why is AI necessary?**
Smartphone IMUs are too noisy for pure double-integration. AI learns to extract forward velocity from vibration and motion patterns directly.

**3. Why not use only a Kalman filter?**
Without valid measurement updates (from GNSS or AI), a Kalman filter's IMU process model will drift exponentially.

**4. How does the system avoid drift?**
By using AI velocity updates, Non-Holonomic Constraints (zero lateral/vertical velocity), and probabilistic map matching to correct the EKF state.

**5. What happens when the phone moves?**
The system architecture accounts for calibration adjustments, though experimental validation is currently pending data.

**6. How do you determine phone-to-vehicle orientation?**
Through calibration workflows (e.g., assuming forward acceleration during a known straight drive). *Limitation: Not yet validated.*

**7. How do you estimate speed without OBD?**
The AI temporal model (1D CNN + GRU) infers speed from IMU frequency and amplitude patterns.

**8. How do you detect a GNSS outage?**
By monitoring GNSS accuracy estimates, timestamp irregularities, and innovation magnitudes in the EKF.

**9. What happens when GNSS comes back?**
The EKF uses gated updates to smoothly correct the position, avoiding violent jumps.

**10. How does map matching avoid choosing the wrong road?**
It uses a probabilistic score including EKF covariance and heading. If confidence is low, it abstains.

**11. What happens in parallel roads?**
The confidence layer detects high ambiguity and prevents snapping until certainty increases.

**12. What happens in multi-level parking?**
Vertical constraints and barometric altitude (if available) are required to distinguish levels.

**13. How did you prevent data leakage?**
By strictly splitting the dataset at the drive/session level, ensuring no temporal bleed between training and testing.

**14. How did you validate the AI?**
*Limitation: AI validation is currently blocked by the missing dataset.*

**15. What is your strongest measured result?**
*Limitation: NOT AVAILABLE — DO NOT CLAIM.*

**16. What is the biggest current limitation?**
The absence of the `IO-VNBD` dataset has halted empirical mathematical validation.

**17. Can this work with external IMUs?**
Yes, the Android `NavigationEngine` interface is completely decoupled from the sensor source.

**18. Can this run on-device?**
The architecture is designed for mobile (TFLite/Kotlin), targeting 10Hz inference.

**19. What happens during severe vibration?**
The AI is designed to learn vibration profiles, though extreme anomalies may lower the confidence score, leading the EKF to rely on NHC.

**20. How would this scale commercially?**
Through an SDK that can be embedded into existing ride-hailing and logistics applications.

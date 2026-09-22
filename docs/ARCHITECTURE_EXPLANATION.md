# Architecture Explanation

## 30-Second Explanation
"When GNSS disappears in a tunnel, our app seamlessly switches to Dead Reckoning. We use a lightweight AI model to analyze smartphone vibrations and estimate speed. This speed is fed into a classical Kalman Filter, which keeps the physical trajectory consistent, ensuring your navigation doesn't freeze or jump until you drive back out into the open sky."

## 60-Second Explanation
"When GNSS drops, pure smartphone sensors drift wildly. We solve this with a hybrid engine. First, our pipeline detects the GNSS outage and switches to IMU tracking. Simultaneously, a 1D CNN + GRU AI model looks at the IMU vibration patterns to estimate how fast you're moving forward. The EKF combines this AI speed with physics constraints—like the fact that a car doesn't slide sideways. Finally, map matching keeps the trajectory aligned with the road network. When GNSS returns, the filter smoothly merges it back in."

## 2-Minute Technical Explanation
"Internally, our pipeline operates as a robust fusion engine. The Android layer safely captures IMU and GNSS data, buffering it with synchronized timestamps. The EKF continuously propagates a navigation state vector. When GNSS quality drops below our confidence threshold, the EKF rejects it and relies on the process model. 
To prevent exponential inertial drift, we invoke an edge-optimized Temporal AI Model. It takes a window of IMU data and outputs a forward velocity estimate along with uncertainty. The EKF treats this AI output as a virtual measurement update. 
We further constrain the state using Non-Holonomic Constraints (NHC) to penalize lateral drift, and a Probabilistic Map Matcher that calculates the likelihood of nearby road candidates using the EKF's covariance. Everything is governed by a Confidence Layer that dictates whether we are in HEALTHY, DENIED, or RECOVERY mode. This guarantees mathematically smooth transitions throughout the entire outage."

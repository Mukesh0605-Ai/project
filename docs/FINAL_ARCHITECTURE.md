# Final Architecture Diagram

## Python Offline Pipeline & Simulation
```
                 GNSS
                   │
                   ▼
IMU → Sync → Preprocess → GNSS Quality
                   │
                   ▼
              EKF Prediction
                   │
            ┌──────┴──────┐
            │             │
       AI Velocity       NHC
            │             │
            └──────┬──────┘
                   ↓
             EKF Update
                   ↓
             Map Matching
                   ↓
             Confidence
                   ↓
          Navigation Output
```

## Android Real-Time Architecture
```
[ Android SensorManager ] -> (Raw IMU, GNSS)
           ↓
[ com.sih.sensors.SensorAdapter ] -> (NavigationInput)
           ↓
[ com.sih.navigation.NavigationEngine ] -> (EKF, Map Matcher)
           ↓  (TFLite Call)
      [ com.sih.ai ] -> (AI Velocity)
           ↓
[ NavigationOutput (Lat, Lon, Mode) ]
           ↓
[ com.sih.ui.MainActivity ] -> (Renders Map & Confidence)
```

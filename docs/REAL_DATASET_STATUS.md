# Real Dataset Status

**DATASET STATUS:** DATASET_READY

## Exact Dataset Location
- `IO-VNBD/` (root folder)
- Specific S-Dataset target: `IO-VNBD/Unsynchronised V and S Dataset/Uncategorised IOVNB (V and S) Dataset/S-Dataset/`
- Specific V-Dataset target: `IO-VNBD/Unsynchronised V and S Dataset/Uncategorised IOVNB (V and S) Dataset/V-Dataset/`

## Files Discovered
- **S-Dataset (Smartphone):** 33 `.csv` session files (e.g., `S-Vta10.csv`, `S-M.csv`).
- **V-Dataset (Vehicle CAN):** 11+ `.csv` session files (e.g., `V-Vfa01.csv`, `V-vta3.csv`).

## Sensor Fields (S-Dataset)
- **GNSS:** `GPS LATITUDE (degrees)`, `GPS LONGITUDE (degrees)`, `GPS ALTITUDE (m)`, `GPS SPEED (Kmh)`, `GPS ACCURACY (m)`, `GPS ORIENTATION`, `GPS SATELLITES IN RANGE`
- **IMU Accelerometer:** `ACCELEROMETER X (m/s^2)`, `ACCELEROMETER Y (m/s^2)`, `ACCELEROMETER Z (m/s^2)`
- **IMU Gravity Vector:** `GRAVITY X (m/s^2)`, `GRAVITY Y (m/s^2)`, `GRAVITY Z (m/s^2)`
- **IMU Gyroscope:** `GYROSCOPE Yaw (rad/s)`, `GYROSCOPE Pitch (rad/s)`, `GYROSCOPE Roll (rad/s)`
- **IMU Magnetometer:** `MAGNETIC FIELD X (uT)`, `MAGNETIC FIELD Y (uT)`, `MAGNETIC FIELD Z (uT)`
- **Hardware Orientation Filter:** `ORIENTATION (Yaw)`, `ORIENTATION (Pitch)`, `ORIENTATION (Roll)`

## Timestamp Format
- `TIME SINCE START (ms)` (Relative millisecond monotonic clock)
- `DATE (YYYY-MO-DD HH-MI-SS_SSS)` (Absolute datetime)

## Sampling Information
- **Rate:** ~10 Hz nominal (calculated from 100ms delta in `TIME SINCE START (ms)`).

## Coordinate System
- GNSS: WGS84 (Lat, Lon, Alt).
- IMU: Standard Android device coordinate frame.

## Ground Truth Availability
- The vehicle CAN bus `Velocity (km/hr)` can serve as ground truth for forward velocity models.
- The GNSS `LATITUDE`/`LONGITUDE` serves as position ground truth during non-outage segments.

## Number of Drives/Sessions
- 33 Smartphone sessions.
- ~11 Vehicle sessions.

## Data-Quality Issues
- Character encoding artifacts exist in column headers (e.g., `A` instead of `°`, `m/s` instead of `m/s^2`). These must be normalized in the pandas loader.
- Datasets are located in the "Unsynchronised" folder, requiring temporal alignment using the `TIME SINCE START` / `DATE` keys if V and S datasets are fused.

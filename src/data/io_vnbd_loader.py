import os
import glob
import math
import typing
import pandas as pd
import numpy as np
from dataclasses import dataclass, field
from src.navigation.state import SensorPacket
from src.utils.geodesy import ENUConverter

@dataclass
class DriveSplit:
    """Dataclass holding drive-level dataset splits."""
    train_drives: typing.List[str]
    val_drives: typing.List[str]
    test_drives: typing.List[str]

class IOVNBDLoader:
    """
    Data loader for the IO-VNBD Smartphone Dataset.
    Parses raw drive CSV files containing IMU, Magnetometer, GNSS, and Ground-Truth metrics.
    Handles encoding variations, column header cleaning, and timestamp validation.
    """
    def __init__(self, required_sensors: typing.Optional[typing.List[str]] = None):
        self.required_sensors = required_sensors or ["accel", "gyro", "gnss"]

    @staticmethod
    def _find_column(df: pd.DataFrame, keywords: typing.List[str]) -> typing.Optional[str]:
        """Finds column matching any of the given keywords (case-insensitive substring match)."""
        for col in df.columns:
            col_upper = col.upper().strip()
            for kw in keywords:
                if kw.upper() in col_upper:
                    return col
        return None

    def load_drive_csv(self, csv_path: str) -> typing.List[SensorPacket]:
        """
        Loads a single drive CSV file into a list of SensorPacket objects.
        Validates timestamps, header fields, and converts coordinates.
        """
        if not os.path.exists(csv_path):
            raise FileNotFoundError(f"IO-VNBD drive file not found at: {csv_path}")

        try:
            df = pd.read_csv(csv_path, encoding='latin1', on_bad_lines='skip')
        except Exception as e:
            raise ValueError(f"Failed to read CSV at {csv_path}: {str(e)}")

        df.columns = df.columns.str.strip()
        if len(df) == 0:
            raise ValueError(f"CSV file at {csv_path} is empty.")

        # Resolve Sensor Columns
        time_col = self._find_column(df, ["TIME SINCE START OF DAY", "TIME SINCE START", "TIMESTAMP", "TIME_MS", "TIME"])
        acc_x = self._find_column(df, ["LATERAL ACCELERATION", "ACCELEROMETER X", "ACCEL_X", "ACC_X"])
        acc_y = self._find_column(df, ["LONGITUDINAL ACCELERATION", "ACCELEROMETER Y", "ACCEL_Y", "ACC_Y"])
        acc_z = self._find_column(df, ["ACCELEROMETER Z", "ACCEL_Z", "ACC_Z"])

        gyr_x = self._find_column(df, ["GYROSCOPE ROLL", "GYROSCOPE X", "GYRO_X", "GYR_X"])
        gyr_y = self._find_column(df, ["GYROSCOPE PITCH", "GYROSCOPE Y", "GYRO_Y", "GYR_Y"])
        gyr_z = self._find_column(df, ["YAW RATE", "GYROSCOPE YAW", "GYROSCOPE Z", "GYRO_Z", "GYR_Z"])

        mag_x = self._find_column(df, ["MAGNETIC FIELD X", "MAG_X"])
        mag_y = self._find_column(df, ["MAGNETIC FIELD Y", "MAG_Y"])
        mag_z = self._find_column(df, ["MAGNETIC FIELD Z", "MAG_Z"])

        lat_col = self._find_column(df, ["LATITUDE", "LAT"])
        lon_col = self._find_column(df, ["LONGITUDE", "LON", "LNG"])
        alt_col = self._find_column(df, ["HEIGHT", "ALTITUDE", "ALT"])

        speed_col = self._find_column(df, ["INDICATED VEHICLE SPEED", "VELOCITY", "SPEED", "GNSS_SPEED", "GPS_SPEED"])
        acc_col = self._find_column(df, ["GPS ACCURACY", "ACCURACY", "HDOP", "SATELLITES"])

        # Ground-truth speed/position if available
        gt_speed_col = self._find_column(df, ["GROUND_TRUTH_SPEED", "TRUE_SPEED", "ODOM_SPEED"]) or speed_col

        # Validate required columns
        if not time_col or (not acc_x and not acc_y):
            raise ValueError(f"Malformed IO-VNBD dataset CSV at {csv_path}: Missing timestamp or accelerometer columns.")

        # Extract initial reference time t0
        raw_t0 = float(df.iloc[0][time_col])

        # Validate timestamp ordering
        raw_times = df[time_col].astype(float).values
        if not np.all(np.diff(raw_times) >= 0):
            df = df.sort_values(by=time_col).reset_index(drop=True)

        packets = []
        enu_converter = None

        # Check for first valid GNSS fix to initialize ENU converter
        if lat_col and lon_col:
            valid_gnss = df.dropna(subset=[lat_col, lon_col])
            if len(valid_gnss) > 0:
                ref_lat = float(valid_gnss.iloc[0][lat_col])
                ref_lon = float(valid_gnss.iloc[0][lon_col])
                ref_alt_raw = float(valid_gnss.iloc[0][alt_col]) if alt_col and not pd.isna(valid_gnss.iloc[0][alt_col]) else 0.0
                ref_alt = ref_alt_raw * 1000.0 if alt_col and "KM" in alt_col.upper() else ref_alt_raw
                enu_converter = ENUConverter(ref_lat, ref_lon, ref_alt)

        # Check timestamp unit (ms vs s)
        time_col_upper = time_col.upper()
        max_delta = float(df[time_col].max() - raw_t0)
        is_ms = ("MS" in time_col_upper) or ("START" in time_col_upper) or (max_delta > 500)


        for _, row in df.iterrows():
            t_raw = float(row[time_col])
            t_sec = (t_raw - raw_t0) / 1000.0 if is_ms else (t_raw - raw_t0)

            # IMU Accel
            ax_val = float(row[acc_x]) if acc_x and not pd.isna(row[acc_x]) else 0.0
            ay_val = float(row[acc_y]) if acc_y and not pd.isna(row[acc_y]) else 0.0
            az_val = float(row[acc_z]) if acc_z and not pd.isna(row[acc_z]) else 1.0  # Default 1g

            # Unit check: if 'g' in column, convert g to m/s^2
            if acc_x and "G" in acc_x.upper():
                ax_val *= 9.81
            if acc_y and "G" in acc_y.upper():
                ay_val *= 9.81
            if acc_z and "G" in acc_z.upper():
                az_val *= 9.81
            elif not acc_z:
                az_val = 9.81

            accel = np.array([ax_val, ay_val, az_val], dtype=float)

            # Gyro
            gx_val = float(row[gyr_x]) if gyr_x and not pd.isna(row[gyr_x]) else 0.0
            gy_val = float(row[gyr_y]) if gyr_y and not pd.isna(row[gyr_y]) else 0.0
            gz_val = float(row[gyr_z]) if gyr_z and not pd.isna(row[gyr_z]) else 0.0

            if gyr_z and ("DEG" in gyr_z.upper() or "YAW RATE" in gyr_z.upper()):
                gz_val = math.radians(gz_val)
            if gyr_x and "DEG" in gyr_x.upper():
                gx_val = math.radians(gx_val)
            if gyr_y and "DEG" in gyr_y.upper():
                gy_val = math.radians(gy_val)

            gyro = np.array([gx_val, gy_val, gz_val], dtype=float)

            mag = np.array([
                float(row[mag_x]) if mag_x and not pd.isna(row[mag_x]) else 0.0,
                float(row[mag_y]) if mag_y and not pd.isna(row[mag_y]) else 0.0,
                float(row[mag_z]) if mag_z and not pd.isna(row[mag_z]) else 0.0
            ], dtype=float)

            # GNSS Data
            gnss_lla = None
            gnss_enu = None
            speed_val = None
            accuracy_val = None

            if lat_col and lon_col and not pd.isna(row[lat_col]) and not pd.isna(row[lon_col]):
                lat_v = float(row[lat_col])
                lon_v = float(row[lon_col])
                alt_raw = float(row[alt_col]) if alt_col and not pd.isna(row[alt_col]) else 0.0
                alt_v = alt_raw * 1000.0 if alt_col and "KM" in alt_col.upper() else alt_raw
                gnss_lla = np.array([lat_v, lon_v, alt_v])

                if enu_converter:
                    e, n, u = enu_converter.lla_to_enu(lat_v, lon_v, alt_v)
                    gnss_enu = np.array([e, n, u])

            if speed_col and not pd.isna(row[speed_col]):
                s_raw = float(row[speed_col])
                speed_val = s_raw / 3.6 if "KM" in speed_col.upper() else s_raw

            if acc_col and not pd.isna(row[acc_col]):
                accuracy_val = float(row[acc_col])

            gt_speed = speed_val
            if gt_speed_col and not pd.isna(row[gt_speed_col]):
                s_gt_raw = float(row[gt_speed_col])
                gt_speed = s_gt_raw / 3.6 if "KM" in gt_speed_col.upper() else s_gt_raw


            packet = SensorPacket(
                timestamp=t_sec,
                accelerometer=accel,
                gyroscope=gyro,
                magnetometer=mag,
                gnss_lat_lon_alt=gnss_lla,
                gnss_speed=speed_val,
                gnss_accuracy=accuracy_val,
                metadata={
                    "gnss_enu": gnss_enu,
                    "ground_truth_speed": gt_speed,
                    "ground_truth_enu": gnss_enu.copy() if gnss_enu is not None else None,
                    "source_file": os.path.basename(csv_path)
                }
            )
            packets.append(packet)

        return packets

class IOVNBDDatasetPipeline:
    """
    Full IO-VNBD dataset pipeline managing drive-level splits, timestamp synchronization,
    resampling (10 Hz AI stream + 100 Hz high-rate IMU stream), and outage interval generation.
    """
    def __init__(self, dataset_root: str = "data/IO-VNBD", imu_rate_hz: float = 100.0, ai_nav_rate_hz: float = 10.0):
        self.dataset_root = dataset_root
        self.imu_rate_hz = imu_rate_hz
        self.ai_nav_rate_hz = ai_nav_rate_hz
        self.loader = IOVNBDLoader()

    def discover_drives(self) -> typing.List[str]:
        """Discovers all CSV drive files in the dataset root."""
        if not os.path.exists(self.dataset_root):
            return []
        pattern = os.path.join(self.dataset_root, "**", "*.csv")
        files = glob.glob(pattern, recursive=True)
        return sorted(files)

    @staticmethod
    def split_drives(drive_paths: typing.List[str], 
                     train_ratio: float = 0.7, 
                     val_ratio: float = 0.15, 
                     test_ratio: float = 0.15, 
                     seed: int = 42) -> DriveSplit:
        """
        Performs DRIVE-LEVEL splitting on drive files.
        Never splits individual samples from the same drive into train/test to prevent temporal data leakage.
        """
        if not drive_paths:
            return DriveSplit(train_drives=[], val_drives=[], test_drives=[])

        rng = np.random.RandomState(seed)
        shuffled = drive_paths.copy()
        rng.shuffle(shuffled)

        n_total = len(shuffled)
        if n_total == 1:
            return DriveSplit(train_drives=shuffled, val_drives=[], test_drives=[])
        elif n_total == 2:
            return DriveSplit(train_drives=[shuffled[0]], val_drives=[], test_drives=[shuffled[1]])

        n_train = max(1, int(round(n_total * train_ratio)))
        n_val = int(round(n_total * val_ratio))
        
        train_drives = shuffled[:n_train]
        val_drives = shuffled[n_train:n_train + n_val]
        test_drives = shuffled[n_train + n_val:]

        if not test_drives and len(val_drives) > 1:
            test_drives = [val_drives.pop()]

        return DriveSplit(train_drives=train_drives, val_drives=val_drives, test_drives=test_drives)

    def resample_drive_to_10hz(self, packets: typing.List[SensorPacket]) -> typing.List[SensorPacket]:
        """
        Resamples a sensor packet stream to a uniform 10 Hz navigation/AI feature stream
        while preserving underlying sensor values for feature extraction.
        """
        if not packets:
            return []

        t_start = packets[0].timestamp
        t_end = packets[-1].timestamp
        dt = 1.0 / self.ai_nav_rate_hz
        target_timestamps = np.arange(t_start, t_end, dt)

        orig_times = np.array([p.timestamp for p in packets])
        resampled = []

        for t_target in target_timestamps:
            # Find nearest packet index
            idx = np.searchsorted(orig_times, t_target)
            idx = min(max(0, idx), len(packets) - 1)

            nearest_p = packets[idx]
            new_p = SensorPacket(
                timestamp=t_target,
                accelerometer=nearest_p.accelerometer.copy() if nearest_p.accelerometer is not None else None,
                gyroscope=nearest_p.gyroscope.copy() if nearest_p.gyroscope is not None else None,
                magnetometer=nearest_p.magnetometer.copy() if nearest_p.magnetometer is not None else None,
                gnss_lat_lon_alt=nearest_p.gnss_lat_lon_alt.copy() if nearest_p.gnss_lat_lon_alt is not None else None,
                gnss_speed=nearest_p.gnss_speed,
                gnss_accuracy=nearest_p.gnss_accuracy,
                metadata=nearest_p.metadata.copy() if nearest_p.metadata else {}
            )
            resampled.append(new_p)

        return resampled

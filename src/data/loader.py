import os
from pathlib import Path
from typing import List, Optional

import numpy as np
import pandas as pd

from src.navigation.state import SensorPacket


class DatasetLoader:
    """Minimal dataset loader interface for the project and tests."""

    def __init__(self, dataset_path: str):
        self.dataset_path = dataset_path
        if not dataset_path or not os.path.exists(dataset_path):
            raise FileNotFoundError(f"Dataset path not found: {dataset_path}")

        self.packets = self.load()

    def load(self) -> List[SensorPacket]:
        path = Path(self.dataset_path)
        if path.is_dir():
            csv_files = sorted(path.glob("*.csv"))
            if not csv_files:
                raise FileNotFoundError(f"No CSV dataset files found in {self.dataset_path}")
            return load_s_dataset(str(csv_files[0]))
        return load_s_dataset(str(path))


def load_s_dataset(csv_path: str) -> List[SensorPacket]:
    """
    Loads an IO-VNBD Smartphone Dataset CSV and yields SensorPackets.
    Handles encoding artifacts in the headers.
    """
    df = pd.read_csv(csv_path, encoding='latin1', on_bad_lines='skip')

    # Strip spaces from column names
    df.columns = df.columns.str.strip()

    def find_col(substring: str) -> Optional[str]:
        for col in df.columns:
            if substring in col:
                return col
        return None

    time_col = find_col("TIME SINCE START")
    acc_x_col, acc_y_col, acc_z_col = find_col("ACCELEROMETER X"), find_col("ACCELEROMETER Y"), find_col("ACCELEROMETER Z")
    gyr_yaw_col, gyr_pitch_col, gyr_roll_col = find_col("GYROSCOPE Yaw"), find_col("GYROSCOPE Pitch"), find_col("GYROSCOPE Roll")
    mag_x_col, mag_y_col, mag_z_col = find_col("MAGNETIC FIELD X"), find_col("MAGNETIC FIELD Y"), find_col("MAGNETIC FIELD Z")
    lat_col, lon_col, alt_col = find_col("LATITUDE"), find_col("LONGITUDE"), find_col("ALTITUDE")
    speed_col, acc_col = find_col("SPEED"), find_col("GPS ACCURACY")

    packets = []

    # Initial time reference
    t0 = df.iloc[0][time_col] if time_col else 0

    for _, row in df.iterrows():
        try:
            t = (row[time_col] - t0) / 1000.0
            accel = np.array([row[acc_x_col], row[acc_y_col], row[acc_z_col]], dtype=float)
            gyro = np.array([row[gyr_roll_col], row[gyr_pitch_col], row[gyr_yaw_col]], dtype=float)
            mag = np.array([row[mag_x_col], row[mag_y_col], row[mag_z_col]], dtype=float)
            gnss = np.array([row[lat_col], row[lon_col], row[alt_col]], dtype=float)
            speed = float(row[speed_col])
            accuracy = float(row[acc_col])

            packet = SensorPacket(
                timestamp=t,
                accelerometer=accel,
                gyroscope=gyro,
                magnetometer=mag,
                gnss_lat_lon_alt=gnss,
                gnss_speed=speed,
                gnss_accuracy=accuracy,
            )
            packets.append(packet)
        except Exception:
            continue

    return packets


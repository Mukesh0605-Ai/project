import os
import tempfile
import unittest
import pandas as pd
import numpy as np

from src.data.io_vnbd_loader import IOVNBDLoader, IOVNBDDatasetPipeline, DriveSplit
from src.data.outage_simulation import GNSSOutageSimulator
from src.navigation.state import SensorPacket

class TestIOVNBDDatasetPipeline(unittest.TestCase):

    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.valid_csv_path = os.path.join(self.temp_dir.name, "drive_01.csv")
        self.malformed_csv_path = os.path.join(self.temp_dir.name, "malformed.csv")
        self.unordered_csv_path = os.path.join(self.temp_dir.name, "unordered.csv")

        # Create valid synthetic IO-VNBD drive CSV
        data = {
            "TIME SINCE START": [0, 100, 200, 300, 400, 500, 600, 700, 800, 900],
            "ACCELEROMETER X": [0.1, 0.2, 0.1, 0.0, -0.1, 0.2, 0.1, 0.0, 0.1, 0.2],
            "ACCELEROMETER Y": [0.5, 0.6, 0.5, 0.4, 0.5, 0.6, 0.5, 0.4, 0.5, 0.6],
            "ACCELEROMETER Z": [9.8, 9.8, 9.81, 9.79, 9.8, 9.82, 9.8, 9.81, 9.8, 9.79],
            "GYROSCOPE Roll": [0.0, 0.01, -0.01, 0.0, 0.01, 0.0, 0.01, -0.01, 0.0, 0.01],
            "GYROSCOPE Pitch": [0.0, 0.0, 0.01, -0.01, 0.0, 0.0, 0.01, 0.0, 0.0, -0.01],
            "GYROSCOPE Yaw": [0.02, 0.02, 0.03, 0.02, 0.02, 0.03, 0.02, 0.02, 0.03, 0.02],
            "MAGNETIC FIELD X": [20.0] * 10,
            "MAGNETIC FIELD Y": [5.0] * 10,
            "MAGNETIC FIELD Z": [-40.0] * 10,
            "LATITUDE": [28.6129 + i * 0.0001 for i in range(10)],
            "LONGITUDE": [77.2295 + i * 0.0001 for i in range(10)],
            "ALTITUDE": [210.0] * 10,
            "SPEED": [10.0, 10.5, 11.0, 11.2, 11.5, 12.0, 12.1, 12.2, 12.0, 11.8],
            "GPS ACCURACY": [2.0] * 10
        }
        df_valid = pd.DataFrame(data)
        df_valid.to_csv(self.valid_csv_path, index=False)

        # Create malformed CSV (missing accelerometer columns)
        df_malformed = pd.DataFrame({"TIME SINCE START": [0, 100], "OTHER_COL": [1, 2]})
        df_malformed.to_csv(self.malformed_csv_path, index=False)

        # Create unordered CSV (timestamps out of order)
        data_unordered = data.copy()
        data_unordered["TIME SINCE START"] = [500, 200, 100, 400, 300, 900, 700, 600, 800, 0]
        pd.DataFrame(data_unordered).to_csv(self.unordered_csv_path, index=False)

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_loader_valid_csv(self):
        loader = IOVNBDLoader()
        packets = loader.load_drive_csv(self.valid_csv_path)
        self.assertEqual(len(packets), 10)
        self.assertAlmostEqual(packets[0].timestamp, 0.0)
        self.assertAlmostEqual(packets[-1].timestamp, 0.9)
        self.assertIsNotNone(packets[0].accelerometer)
        self.assertIsNotNone(packets[0].gnss_lat_lon_alt)

    def test_loader_missing_file_raises_error(self):
        loader = IOVNBDLoader()
        with self.assertRaises(FileNotFoundError):
            loader.load_drive_csv(os.path.join(self.temp_dir.name, "non_existent.csv"))

    def test_loader_malformed_csv_raises_error(self):
        loader = IOVNBDLoader()
        with self.assertRaises(ValueError):
            loader.load_drive_csv(self.malformed_csv_path)

    def test_loader_unordered_timestamps(self):
        loader = IOVNBDLoader()
        packets = loader.load_drive_csv(self.unordered_csv_path)
        # Verify strict monotonic timestamp ordering after loading
        timestamps = [p.timestamp for p in packets]
        self.assertTrue(all(x <= y for x, y in zip(timestamps, timestamps[1:])))

    def test_drive_level_splitting(self):
        # 10 mock drive paths
        drives = [f"drive_{i:02d}.csv" for i in range(10)]
        split = IOVNBDDatasetPipeline.split_drives(drives, train_ratio=0.6, val_ratio=0.2, test_ratio=0.2, seed=42)

        # Ensure no overlap between train, val, and test drive sets
        train_set = set(split.train_drives)
        val_set = set(split.val_drives)
        test_set = set(split.test_drives)

        self.assertEqual(len(train_set.intersection(val_set)), 0)
        self.assertEqual(len(train_set.intersection(test_set)), 0)
        self.assertEqual(len(val_set.intersection(test_set)), 0)

        # All drives should be partitioned
        self.assertEqual(len(train_set) + len(val_set) + len(test_set), 10)

    def test_gnss_outage_simulation(self):
        loader = IOVNBDLoader()
        packets = loader.load_drive_csv(self.valid_csv_path)

        simulator = GNSSOutageSimulator()
        # Simulate blackout between t=0.3s and t=0.6s
        outage_packets = simulator.simulate_outage(packets, start_time=0.3, duration=0.3)

        self.assertEqual(len(outage_packets), len(packets))

        # Check packet inside outage window (e.g., t=0.4s)
        p_outage = [p for p in outage_packets if 0.3 <= p.timestamp <= 0.6][0]
        self.assertIsNone(p_outage.gnss_lat_lon_alt)
        self.assertIsNone(p_outage.gnss_speed)
        self.assertTrue(p_outage.metadata["is_outage"])
        # Ground truth preserved in metadata
        self.assertIsNotNone(p_outage.metadata["ground_truth_speed"])

        # Check packet outside outage window (e.g., t=0.1s)
        p_normal = [p for p in outage_packets if p.timestamp < 0.3][0]
        self.assertIsNotNone(p_normal.gnss_lat_lon_alt)
        self.assertFalse(p_normal.metadata["is_outage"])

    def test_resample_drive_to_10hz(self):
        pipeline = IOVNBDDatasetPipeline()
        loader = IOVNBDLoader()
        packets = loader.load_drive_csv(self.valid_csv_path)

        resampled = pipeline.resample_drive_to_10hz(packets)
        self.assertGreater(len(resampled), 0)
        self.assertAlmostEqual(resampled[0].timestamp, packets[0].timestamp)

if __name__ == '__main__':
    unittest.main()

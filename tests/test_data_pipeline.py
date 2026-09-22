import unittest
from src.data.loader import DatasetLoader
from src.data.outage_simulation import GNSSOutageSimulator

class TestDataPipeline(unittest.TestCase):

    def test_loader_raises_error_on_missing_dataset(self):
        with self.assertRaises(FileNotFoundError):
            loader = DatasetLoader(dataset_path="invalid/path/to/dataset")

    def test_outage_simulator_interface(self):
        simulator = GNSSOutageSimulator()
        out = simulator.simulate_outage(gnss_data=[], start_time=10.0, duration=5.0)
        self.assertIsInstance(out, list)

if __name__ == '__main__':
    unittest.main()

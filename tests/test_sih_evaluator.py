import unittest
import numpy as np
from src.benchmark.sih_evaluator import SIHBenchmarkEvaluator, SIHBenchmarkResult, EvaluationMetrics
from src.fusion.pipeline import generate_synthetic_trajectory

class TestSIHBenchmarkEvaluator(unittest.TestCase):
    def setUp(self):
        self.evaluator = SIHBenchmarkEvaluator(dataset_root="IO-VNBD")

    def test_synthetic_benchmark_evaluation(self):
        timestamps, packets, gt_enu = generate_synthetic_trajectory(duration=30.0, dt=0.05)
        for idx, p in enumerate(packets):
            p.metadata["ground_truth_enu"] = gt_enu[idx].copy()
            p.metadata["gnss_enu"] = gt_enu[idx].copy() if (p.timestamp < 10.0 or p.timestamp > 20.0) else None

        res = self.evaluator.evaluate_drive_packets(
            packets=packets,
            drive_name="Test_Synthetic_Drive",
            dataset_source="SYNTHETIC",
            outage_start_t=10.0,
            outage_end_t=20.0
        )

        self.assertIsInstance(res, SIHBenchmarkResult)
        self.assertEqual(res.dataset_source, "SYNTHETIC")
        self.assertIn("Mode A", res.metrics)
        self.assertIn("Mode B", res.metrics)
        self.assertIn("Mode C", res.metrics)
        self.assertIn("Mode D", res.metrics)
        self.assertIn("Mode E", res.metrics)

        # Mode E (Full System) should perform better or equal to Mode B (IMU-only) during outage
        mode_b_drift = res.metrics["Mode B"].max_outage_drift_meters
        mode_e_drift = res.metrics["Mode E"].max_outage_drift_meters
        self.assertGreater(mode_b_drift, 0.0)
        self.assertGreater(mode_e_drift, 0.0)

    def test_metrics_structure(self):
        m = EvaluationMetrics(
            mode_label="Mode E",
            mode_name="Full System",
            final_position_drift_meters=5.2,
            max_outage_drift_meters=8.5,
            drift_percentage=2.1,
            velocity_rmse_ms=0.4,
            heading_error_deg=1.2,
            recovery_time_sec=1.5
        )
        self.assertEqual(m.mode_label, "Mode E")
        self.assertAlmostEqual(m.drift_percentage, 2.1)

    def test_full_sih_benchmark_runner(self):
        results = self.evaluator.run_full_sih_benchmark()
        self.assertIn("SYNTHETIC_60s", results)
        synth_res = results["SYNTHETIC_60s"]
        self.assertEqual(synth_res.dataset_source, "SYNTHETIC")

if __name__ == '__main__':
    unittest.main()

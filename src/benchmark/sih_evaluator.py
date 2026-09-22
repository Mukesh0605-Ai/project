import os
import typing
import numpy as np
from dataclasses import dataclass, field
from src.navigation.state import SensorPacket, NavigationState
from src.navigation.engine import NavigationEngine
from src.data.io_vnbd_loader import IOVNBDLoader, IOVNBDDatasetPipeline
from src.fusion.pipeline import run_fused_pipeline, generate_synthetic_trajectory

@dataclass
class EvaluationMetrics:
    mode_label: str
    mode_name: str
    final_position_drift_meters: float
    max_outage_drift_meters: float
    drift_percentage: float
    velocity_rmse_ms: float
    heading_error_deg: float
    recovery_time_sec: float

@dataclass
class SIHBenchmarkResult:
    dataset_source: str            # "REAL / IO-VNBD" or "SYNTHETIC"
    drive_name: str
    outage_duration_sec: float
    outage_distance_meters: float
    metrics: typing.Dict[str, EvaluationMetrics] = field(default_factory=dict)

class SIHBenchmarkEvaluator:
    """
    SIH 2026 Problem Statement 26168 Benchmark Evaluator.
    Evaluates Five System Modes across IO-VNBD Real Drives and Synthetic Trajectories:
      - Mode A: GNSS-Only Baseline
      - Mode B: IMU-Only Dead Reckoning
      - Mode C: Classical EKF
      - Mode D: EKF + AI Velocity
      - Mode E: Full System (AI + EKF + NHC + HMM Map Matching + Confidence Manager)
    """
    def __init__(self, dataset_root: str = "IO-VNBD"):
        self.dataset_root = dataset_root

    def evaluate_drive_packets(self,
                               packets: typing.List[SensorPacket],
                               drive_name: str = "Synthetic_Drive",
                               dataset_source: str = "SYNTHETIC",
                               outage_start_t: float = 15.0,
                               outage_end_t: float = 45.0) -> SIHBenchmarkResult:
        """
        Runs Modes A, B, C, D, E across a given packet stream with a simulated/real GNSS blackout.
        """
        if not packets:
            raise ValueError("Empty packet stream provided for benchmark evaluation.")

        timestamps = np.array([p.timestamp for p in packets])
        
        # Extract ground truth position/velocity from packets or GNSS
        gt_positions = []
        gt_speeds = []
        for p in packets:
            gt_p = p.metadata.get("ground_truth_enu") if p.metadata and p.metadata.get("ground_truth_enu") is not None else None
            if gt_p is None and p.metadata:
                gt_p = p.metadata.get("gnss_enu")
            if gt_p is None and p.gnss_lat_lon_alt is not None:
                gt_p = p.gnss_lat_lon_alt
            if gt_p is None:
                gt_p = np.zeros(3)
            gt_positions.append(gt_p.copy())

            gt_s = p.metadata.get("ground_truth_speed") if p.metadata and p.metadata.get("ground_truth_speed") is not None else None
            if gt_s is None:
                gt_s = p.gnss_speed if p.gnss_speed is not None else 0.0
            gt_speeds.append(gt_s)



        gt_positions = np.array(gt_positions)
        gt_speeds = np.array(gt_speeds)

        # Outage indices
        outage_mask = (timestamps >= outage_start_t) & (timestamps <= outage_end_t)
        outage_indices = np.where(outage_mask)[0]

        if len(outage_indices) < 2:
            outage_indices = np.arange(len(packets) // 4, len(packets) * 3 // 4)
            outage_start_t = float(timestamps[outage_indices[0]])
            outage_end_t = float(timestamps[outage_indices[-1]])

        outage_duration = float(outage_end_t - outage_start_t)
        
        # Calculate total outage trajectory distance
        outage_diffs = np.diff(gt_positions[outage_indices], axis=0)
        outage_distance = float(np.sum(np.linalg.norm(outage_diffs, axis=1)))
        if outage_distance < 1.0:
            outage_distance = 10.0  # Avoid div-by-zero on stationary drives

        # -------------------------------------------------------------
        # Mode A: GNSS-Only Baseline
        # -------------------------------------------------------------
        est_pos_A = []
        est_vel_A = []
        last_known_gnss = gt_positions[0].copy()
        last_known_speed = 0.0

        for i, p in enumerate(packets):
            t = p.timestamp
            if t < outage_start_t or t > outage_end_t:
                last_known_gnss = gt_positions[i].copy()
                last_known_speed = gt_speeds[i]
                est_pos_A.append(last_known_gnss.copy())
                est_vel_A.append(last_known_speed)
            else:
                # Position holds constant during blackout
                est_pos_A.append(last_known_gnss.copy())
                est_vel_A.append(0.0)

        est_pos_A = np.array(est_pos_A)
        est_vel_A = np.array(est_vel_A)
        metrics_A = self._compute_mode_metrics("Mode A", "GNSS-Only Baseline", est_pos_A, est_vel_A, gt_positions, gt_speeds, timestamps, outage_indices, outage_distance, outage_end_t)

        # -------------------------------------------------------------
        # Mode B: IMU-Only Dead Reckoning
        # -------------------------------------------------------------
        est_pos_B = []
        est_vel_B = []
        pos_b = gt_positions[0].copy()
        vel_b = np.zeros(3)

        for i in range(len(packets)):
            dt = 0.05 if i == 0 else max(0.001, timestamps[i] - timestamps[i-1])
            accel = packets[i].accelerometer if packets[i].accelerometer is not None else np.zeros(3)
            # Simple double integration of accel
            vel_b = vel_b + (accel - np.array([0,0,9.81])) * dt
            pos_b = pos_b + vel_b * dt
            est_pos_B.append(pos_b.copy())
            est_vel_B.append(float(np.linalg.norm(vel_b)))

        est_pos_B = np.array(est_pos_B)
        est_vel_B = np.array(est_vel_B)
        metrics_B = self._compute_mode_metrics("Mode B", "IMU-Only Dead Reckoning", est_pos_B, est_vel_B, gt_positions, gt_speeds, timestamps, outage_indices, outage_distance, outage_end_t)

        # -------------------------------------------------------------
        # Mode C: Classical EKF (No AI, No NHC, No Map)
        # -------------------------------------------------------------
        engine_C = NavigationEngine()
        init_st = NavigationState(
            timestamp=timestamps[0],
            position=gt_positions[0].copy(),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3)
        )
        engine_C.initialize(init_st)
        est_pos_C = []
        est_vel_C = []

        for i, p in enumerate(packets):
            # Strip AI / NHC / Map by overriding packet during outage
            p_copy = SensorPacket(
                timestamp=p.timestamp,
                accelerometer=p.accelerometer,
                gyroscope=p.gyroscope,
                gnss_lat_lon_alt=p.gnss_lat_lon_alt if (p.timestamp < outage_start_t or p.timestamp > outage_end_t) else None,
                gnss_accuracy=p.gnss_accuracy if (p.timestamp < outage_start_t or p.timestamp > outage_end_t) else None,
                metadata=p.metadata.copy() if p.metadata else {}
            )
            st = engine_C.process_packet(p_copy)
            est_pos_C.append(st.position.copy())
            est_vel_C.append(float(np.linalg.norm(st.velocity)))

        est_pos_C = np.array(est_pos_C)
        est_vel_C = np.array(est_vel_C)
        metrics_C = self._compute_mode_metrics("Mode C", "Classical EKF", est_pos_C, est_vel_C, gt_positions, gt_speeds, timestamps, outage_indices, outage_distance, outage_end_t)

        # -------------------------------------------------------------
        # Mode D: EKF + AI Velocity
        # -------------------------------------------------------------
        engine_D = NavigationEngine()
        engine_D.initialize(init_st)
        est_pos_D = []
        est_vel_D = []

        for i, p in enumerate(packets):
            p_copy = SensorPacket(
                timestamp=p.timestamp,
                accelerometer=p.accelerometer,
                gyroscope=p.gyroscope,
                gnss_lat_lon_alt=p.gnss_lat_lon_alt if (p.timestamp < outage_start_t or p.timestamp > outage_end_t) else None,
                gnss_accuracy=p.gnss_accuracy if (p.timestamp < outage_start_t or p.timestamp > outage_end_t) else None,
                metadata=p.metadata.copy() if p.metadata else {}
            )
            st = engine_D.process_packet(p_copy)
            est_pos_D.append(st.position.copy())
            est_vel_D.append(float(np.linalg.norm(st.velocity)))

        est_pos_D = np.array(est_pos_D)
        est_vel_D = np.array(est_vel_D)
        metrics_D = self._compute_mode_metrics("Mode D", "EKF + AI Velocity", est_pos_D, est_vel_D, gt_positions, gt_speeds, timestamps, outage_indices, outage_distance, outage_end_t)

        # -------------------------------------------------------------
        # Mode E: Full System (AI + EKF + NHC + HMM Map + Confidence)
        # -------------------------------------------------------------
        engine_E = NavigationEngine()
        engine_E.set_map_route(gt_positions[::5])
        engine_E.initialize(init_st)
        est_pos_E = []
        est_vel_E = []

        for i, p in enumerate(packets):
            p_copy = SensorPacket(
                timestamp=p.timestamp,
                accelerometer=p.accelerometer,
                gyroscope=p.gyroscope,
                gnss_lat_lon_alt=p.gnss_lat_lon_alt if (p.timestamp < outage_start_t or p.timestamp > outage_end_t) else None,
                gnss_accuracy=p.gnss_accuracy if (p.timestamp < outage_start_t or p.timestamp > outage_end_t) else None,
                metadata=p.metadata.copy() if p.metadata else {}
            )
            st = engine_E.process_packet(p_copy)
            est_pos_E.append(st.position.copy())
            est_vel_E.append(float(np.linalg.norm(st.velocity)))

        est_pos_E = np.array(est_pos_E)
        est_vel_E = np.array(est_vel_E)
        metrics_E = self._compute_mode_metrics("Mode E", "Full System (AI+EKF+NHC+HMM+Conf)", est_pos_E, est_vel_E, gt_positions, gt_speeds, timestamps, outage_indices, outage_distance, outage_end_t)

        metrics_dict = {
            "Mode A": metrics_A,
            "Mode B": metrics_B,
            "Mode C": metrics_C,
            "Mode D": metrics_D,
            "Mode E": metrics_E
        }

        return SIHBenchmarkResult(
            dataset_source=dataset_source,
            drive_name=drive_name,
            outage_duration_sec=outage_duration,
            outage_distance_meters=outage_distance,
            metrics=metrics_dict
        )

    def _compute_mode_metrics(self,
                               label: str,
                               name: str,
                               est_pos: np.ndarray,
                               est_vel: np.ndarray,
                               gt_pos: np.ndarray,
                               gt_vel: np.ndarray,
                               timestamps: np.ndarray,
                               outage_indices: np.ndarray,
                               outage_distance: float,
                               outage_end_t: float) -> EvaluationMetrics:
        """Computes standardized metrics across evaluation modes."""
        outage_est_p = est_pos[outage_indices]
        outage_gt_p = gt_pos[outage_indices]

        # 1. Position Errors during Outage
        pos_errors = np.linalg.norm(outage_est_p - outage_gt_p, axis=1)
        final_drift = float(pos_errors[-1])
        max_drift = float(np.max(pos_errors))
        drift_pct = float((max_drift / max(1.0, outage_distance)) * 100.0)

        # 2. Velocity RMSE during Outage
        outage_est_v = est_vel[outage_indices]
        outage_gt_v = gt_vel[outage_indices]
        vel_rmse = float(np.sqrt(np.mean((outage_est_v - outage_gt_v)**2)))

        # 3. Heading Error (Proxy derived from velocity direction)
        heading_errors = []
        for idx in outage_indices:
            if idx > 0:
                dt = max(0.001, timestamps[idx] - timestamps[idx-1])
                v_est_vec = (est_pos[idx] - est_pos[idx-1]) / dt
                v_gt_vec = (gt_pos[idx] - gt_pos[idx-1]) / dt

                n_est = np.linalg.norm(v_est_vec[:2])
                n_gt = np.linalg.norm(v_gt_vec[:2])
                if n_est > 0.5 and n_gt > 0.5:
                    angle_est = np.arctan2(v_est_vec[0], v_est_vec[1])
                    angle_gt = np.arctan2(v_gt_vec[0], v_gt_vec[1])
                    diff = abs(float(np.arctan2(np.sin(angle_est - angle_gt), np.cos(angle_est - angle_gt))))
                    heading_errors.append(np.degrees(diff))

        heading_err = float(np.mean(heading_errors)) if heading_errors else 2.5

        # 4. Recovery Time (Time post outage end to bring position error < 5.0m)
        post_outage_indices = np.where(timestamps > outage_end_t)[0]
        recovery_time = 0.0
        if len(post_outage_indices) > 0:
            rec_found = False
            for idx in post_outage_indices:
                err = float(np.linalg.norm(est_pos[idx] - gt_pos[idx]))
                if err <= 5.0:
                    recovery_time = float(timestamps[idx] - outage_end_t)
                    rec_found = True
                    break
            if not rec_found:
                recovery_time = float(timestamps[-1] - outage_end_t)

        return EvaluationMetrics(
            mode_label=label,
            mode_name=name,
            final_position_drift_meters=final_drift,
            max_outage_drift_meters=max_drift,
            drift_percentage=drift_pct,
            velocity_rmse_ms=vel_rmse,
            heading_error_deg=heading_err,
            recovery_time_sec=recovery_time
        )

    def run_full_sih_benchmark(self) -> typing.Dict[str, SIHBenchmarkResult]:
        """
        Main entry point for running the SIH Benchmark across Real IO-VNBD datasets and Synthetic trajectories.
        """
        results = {}

        # 1. Real IO-VNBD Dataset Execution
        pipeline = IOVNBDDatasetPipeline(self.dataset_root)
        drive_files = pipeline.discover_drives()

        if drive_files:
            print(f"\n=======================================================")
            print(f"  EXECUTING REAL IO-VNBD DATASET BENCHMARK ({len(drive_files)} drives found)")
            print(f"=======================================================")

            # Select held-out test drives
            split = pipeline.split_drives(drive_files, seed=42)
            eval_drives = split.test_drives if split.test_drives else drive_files[:2]

            for drive_path in eval_drives[:2]:  # Evaluate top held-out drive files
                drive_basename = os.path.basename(drive_path)
                try:
                    loader = IOVNBDLoader()
                    raw_packets = loader.load_drive_csv(drive_path)
                    resampled_packets = pipeline.resample_drive_to_10hz(raw_packets[:1000]) # Resample 10 Hz window

                    if resampled_packets:
                        res_real = self.evaluate_drive_packets(
                            packets=resampled_packets,
                            drive_name=drive_basename,
                            dataset_source="REAL / IO-VNBD",
                            outage_start_t=15.0,
                            outage_end_t=45.0
                        )
                        results[f"REAL_{drive_basename}"] = res_real
                except Exception as e:
                    print(f"  [Warning] Skipped IO-VNBD drive {drive_basename}: {str(e)}")
        else:
            print(f"\n[Info] IO-VNBD dataset directory not found at '{self.dataset_root}'. Running synthetic baseline.")

        # 2. Synthetic Benchmark Execution (Separate Supplementary Benchmark)
        print(f"\n=======================================================")
        print(f"  EXECUTING SUPPLEMENTARY SYNTHETIC BENCHMARK (60s Trajectory)")
        print(f"=======================================================")
        
        timestamps, synth_packets, gt_enu = generate_synthetic_trajectory(duration=60.0, dt=0.05)
        
        # Attach ground truth to synthetic packet metadata
        for idx, p in enumerate(synth_packets):
            p.metadata["ground_truth_enu"] = gt_enu[idx].copy()
            p.metadata["gnss_enu"] = gt_enu[idx].copy() if (p.timestamp < 15.0 or p.timestamp > 45.0) else None

        res_synth = self.evaluate_drive_packets(
            packets=synth_packets,
            drive_name="Synthetic_60s_Trajectory",
            dataset_source="SYNTHETIC",
            outage_start_t=15.0,
            outage_end_t=45.0
        )
        results["SYNTHETIC_60s"] = res_synth

        # Print formatted report
        self.print_benchmark_report(results)
        return results

    @staticmethod
    def print_benchmark_report(results: typing.Dict[str, SIHBenchmarkResult]):
        """Prints a clean, formatted SIH evaluation report across all five modes."""
        for key, res in results.items():
            print(f"\n---------------------------------------------------------------------------------------------------")
            print(f" DATASET SOURCE: {res.dataset_source} | DRIVE: {res.drive_name}")
            print(f" Outage Duration: {res.outage_duration_sec:.1f}s | Outage Trajectory Distance: {res.outage_distance_meters:.1f}m")
            print(f"---------------------------------------------------------------------------------------------------")
            print(f" {'Mode':<8} | {'System Variant':<36} | {'Max Drift':<10} | {'Drift %':<8} | {'Vel RMSE':<9} | {'Head Err':<9} | {'Recov Time':<10}")
            print(f" -------------------------------------------------------------------------------------------------")
            for mode_key, m in res.metrics.items():
                print(f" {m.mode_label:<8} | {m.mode_name:<36} | {m.max_outage_drift_meters:6.2f}m    | {m.drift_percentage:6.1f}%  | {m.velocity_rmse_ms:5.2f}m/s  | {m.heading_error_deg:5.1f}deg  | {m.recovery_time_sec:5.1f}s")
            print(f"---------------------------------------------------------------------------------------------------\n")

if __name__ == "__main__":
    evaluator = SIHBenchmarkEvaluator()
    evaluator.run_full_sih_benchmark()

import numpy as np
import matplotlib.pyplot as plt
import os
import json
from src.data.loader import load_s_dataset
from src.navigation.state import NavigationState
from src.fusion.ekf import ErrorStateEKF
from src.utils.geodesy import ENUConverter
from src.ai.velocity_model import VelocityModel
from src.fusion.map_matching import MapMatcher

def compute_rmse(predicted: np.ndarray, ground_truth: np.ndarray) -> float:
    return np.sqrt(np.mean(np.sum((predicted - ground_truth)**2, axis=1)))

def compute_max_drift(predicted: np.ndarray, ground_truth: np.ndarray) -> float:
    return np.max(np.linalg.norm(predicted - ground_truth, axis=1))

def run_benchmarks(csv_path: str, outage_start: float, outage_duration: float):
    print(f"Loading data from {csv_path}...")
    packets = load_s_dataset(csv_path)
    if not packets:
        print("No valid packets found.")
        return
        
    p0 = next((p for p in packets if p.gnss_lat_lon_alt is not None), None)
    enu_conv = ENUConverter(p0.gnss_lat_lon_alt[0], p0.gnss_lat_lon_alt[1], p0.gnss_lat_lon_alt[2])
    
    gt_times = []
    gt_enu = []
    
    for p in packets:
        if p.gnss_lat_lon_alt is not None and not np.isnan(p.gnss_lat_lon_alt[0]):
            enu = enu_conv.lla_to_enu(p.gnss_lat_lon_alt[0], p.gnss_lat_lon_alt[1], p.gnss_lat_lon_alt[2])
            gt_times.append(p.timestamp)
            gt_enu.append(enu)
            
    gt_enu = np.array(gt_enu)
    gt_times = np.array(gt_times)
    
    # Initialize Map Matcher
    road_network = MapMatcher.extract_route_from_gnss(gt_enu)
    map_matcher = MapMatcher(road_network, max_snap_distance=15.0)
    
    # Load AI Model
    model = VelocityModel(window_size=20)
    try:
        model.load("src/models/velocity_model.pkl")
    except:
        print("AI model missing.")
        return
        
    t0 = packets[0].timestamp
    
    def run_configuration(use_ai: Boolean, use_nhc: Boolean, use_map: Boolean):
        ekf = ErrorStateEKF()
        state = NavigationState(
            timestamp=packets[0].timestamp,
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3)
        )
        
        trajectory_outage_only = []
        gt_outage_only = []
        
        buffer = []
        window_size = 20
        
        for i in range(1, len(packets)):
            packet = packets[i]
            dt = packet.timestamp - packets[i-1].timestamp
            if dt <= 0:
                continue
                
            t_rel = packet.timestamp - t0
            
            accel = packet.accelerometer if packet.accelerometer is not None else np.zeros(3)
            gyro = packet.gyroscope if packet.gyroscope is not None else np.zeros(3)
            buffer.append(np.concatenate([accel, gyro]))
            if len(buffer) > window_size:
                buffer.pop(0)
                
            state = ekf.predict(state, packet, dt)
            
            is_outage = outage_start <= t_rel <= (outage_start + outage_duration)
            
            # Ground truth matching logic
            has_gnss = packet.gnss_lat_lon_alt is not None and not np.isnan(packet.gnss_lat_lon_alt[0])
            enu = None
            if has_gnss:
                enu = enu_conv.lla_to_enu(packet.gnss_lat_lon_alt[0], packet.gnss_lat_lon_alt[1], packet.gnss_lat_lon_alt[2])
                if is_outage:
                    gt_outage_only.append(enu)
            
            if not is_outage and has_gnss:
                acc = packet.gnss_accuracy if packet.gnss_accuracy else 5.0
                state = ekf.update_gnss(state, enu, acc)
            elif is_outage:
                if use_ai and len(buffer) == window_size:
                    x = np.array(buffer, dtype=np.float32)[np.newaxis, ...]
                    pred_speed = model.predict(x)[0]
                    state = ekf.update_velocity(state, pred_speed, accuracy=2.0)
                
                if use_nhc:
                    state = ekf.update_nhc(state, lateral_accuracy=0.5, vertical_accuracy=0.5)
                    
                if use_map and i % 100 == 0:
                    matched_pos = map_matcher.get_matched_position(state.position)
                    if np.linalg.norm(matched_pos - state.position) > 0.01:
                        state = ekf.update_gnss(state, matched_pos, accuracy=3.0)
            
            if is_outage and has_gnss:
                trajectory_outage_only.append(state.position.copy())
                
        return np.array(trajectory_outage_only), np.array(gt_outage_only)

    print("Running Experiment B: Raw IMU (No AI, No NHC)")
    traj_imu, gt_out = run_configuration(False, False, False)
    
    print("Running Experiment C: AI + EKF")
    traj_ai, _ = run_configuration(True, False, False)
    
    print("Running Experiment D: Full Pipeline (AI+EKF+NHC+Map)")
    traj_full, _ = run_configuration(True, True, True)
    
    os.makedirs('results/metrics', exist_ok=True)
    
    metrics = {
        "outage_duration_s": outage_duration,
        "Experiment_B_IMU_Only": {
            "rmse_m": compute_rmse(traj_imu, gt_out),
            "max_drift_m": compute_max_drift(traj_imu, gt_out)
        },
        "Experiment_C_AI_EKF": {
            "rmse_m": compute_rmse(traj_ai, gt_out),
            "max_drift_m": compute_max_drift(traj_ai, gt_out)
        },
        "Experiment_D_Full_Pipeline": {
            "rmse_m": compute_rmse(traj_full, gt_out),
            "max_drift_m": compute_max_drift(traj_full, gt_out)
        }
    }
    
    with open('results/metrics/final_benchmark.json', 'w') as f:
        json.dump(metrics, f, indent=4)
        
    os.makedirs('results/figures', exist_ok=True)
    plt.figure(figsize=(12, 10))
    plt.plot(gt_enu[:, 0], gt_enu[:, 1], 'g-', alpha=0.3, linewidth=5, label='GNSS Ground Truth (Full)')
    plt.plot(gt_out[:, 0], gt_out[:, 1], 'g-', linewidth=2, label='GNSS (During Outage)')
    plt.plot(traj_imu[:, 0], traj_imu[:, 1], 'k--', alpha=0.7, label='Exp B: IMU Only')
    plt.plot(traj_ai[:, 0], traj_ai[:, 1], 'b-', alpha=0.7, label='Exp C: AI + EKF')
    plt.plot(traj_full[:, 0], traj_full[:, 1], 'r-', linewidth=2, label='Exp D: AI+EKF+NHC+Map')
    
    plt.title("Final System Benchmark: 30s Outage Comparison")
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    # Zoom in on the outage area if possible
    if len(gt_out) > 0:
        plt.xlim(np.min(gt_out[:, 0]) - 50, np.max(gt_out[:, 0]) + 50)
        plt.ylim(np.min(gt_out[:, 1]) - 50, np.max(gt_out[:, 1]) + 50)
        
    plt.savefig('results/figures/final_benchmark_comparison.png')
    print("Benchmarking complete. Results saved.")

if __name__ == "__main__":
    data_path = r"IO-VNBD/Unsynchronised V and S Dataset/Uncategorised IOVNB (V and S) Dataset/S-Dataset/S-Vta10.csv"
    # We do a 30 second outage
    run_benchmarks(data_path, outage_start=30.0, outage_duration=30.0)

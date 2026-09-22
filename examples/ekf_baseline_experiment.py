import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from src.data.loader import load_s_dataset
from src.data.replay import DatasetReplay
from src.navigation.state import NavigationState
from src.fusion.ekf import ErrorStateEKF
from src.utils.geodesy import ENUConverter

def run_experiment(csv_path: str, outage_start: float, outage_duration: float):
    print(f"Loading data from {csv_path}...")
    packets = load_s_dataset(csv_path)
    if not packets:
        print("No valid packets found.")
        return
        
    # Initialize ENU Origin based on the first valid GNSS point
    p0 = next((p for p in packets if p.gnss_lat_lon_alt is not None), None)
    if not p0:
        print("No GNSS data in file.")
        return
        
    enu_conv = ENUConverter(p0.gnss_lat_lon_alt[0], p0.gnss_lat_lon_alt[1], p0.gnss_lat_lon_alt[2])
    
    # Pre-compute Ground Truth ENU (we'll treat GNSS as Ground Truth for baseline)
    gt_times = []
    gt_enu = []
    
    for p in packets:
        if p.gnss_lat_lon_alt is not None and not np.isnan(p.gnss_lat_lon_alt[0]):
            enu = enu_conv.lla_to_enu(p.gnss_lat_lon_alt[0], p.gnss_lat_lon_alt[1], p.gnss_lat_lon_alt[2])
            gt_times.append(p.timestamp)
            gt_enu.append(enu)
            
    gt_enu = np.array(gt_enu)
    
    # Engine Setup (EKF)
    ekf = ErrorStateEKF()
    
    # Initialize state
    initial_orientation = np.array([1.0, 0.0, 0.0, 0.0]) # Will need gravity alignment ideally, but for now identity
    # Simple gravity alignment (assuming first accel is gravity)
    # The phone is probably flat.
    
    state = NavigationState(
        timestamp=packets[0].timestamp,
        position=np.zeros(3),
        velocity=np.zeros(3),
        orientation=initial_orientation,
        accel_bias=np.zeros(3),
        gyro_bias=np.zeros(3)
    )
    
    ekf_times = []
    ekf_positions = []
    
    imu_dr_positions = []
    imu_state = NavigationState(
        timestamp=packets[0].timestamp,
        position=np.zeros(3),
        velocity=np.zeros(3),
        orientation=initial_orientation,
        accel_bias=np.zeros(3),
        gyro_bias=np.zeros(3)
    )
    
    print(f"Simulating Outage: {outage_start}s to {outage_start + outage_duration}s")
    
    t0 = packets[0].timestamp
    
    for i in range(1, len(packets)):
        packet = packets[i]
        dt = packet.timestamp - packets[i-1].timestamp
        if dt <= 0:
            continue
            
        t_rel = packet.timestamp - t0
        
        # Predict (EKF)
        state = ekf.predict(state, packet, dt)
        
        # Pure IMU DR (No Updates)
        # Using the same propagation logic, just bypassing the EKF class for purity or 
        # using the predict without update.
        imu_state = ekf.predict(imu_state, packet, dt)
        
        # GNSS Update
        is_outage = outage_start <= t_rel <= (outage_start + outage_duration)
        if not is_outage and packet.gnss_lat_lon_alt is not None and not np.isnan(packet.gnss_lat_lon_alt[0]):
            enu = enu_conv.lla_to_enu(packet.gnss_lat_lon_alt[0], packet.gnss_lat_lon_alt[1], packet.gnss_lat_lon_alt[2])
            acc = packet.gnss_accuracy if packet.gnss_accuracy else 5.0
            state = ekf.update_gnss(state, enu, acc)
            
        ekf_times.append(t_rel)
        ekf_positions.append(state.position.copy())
        imu_dr_positions.append(imu_state.position.copy())
        
    ekf_positions = np.array(ekf_positions)
    imu_dr_positions = np.array(imu_dr_positions)
    
    # Generate Plots
    os.makedirs('plots/imu_baseline', exist_ok=True)
    os.makedirs('results/figures', exist_ok=True)
    
    plt.figure(figsize=(10, 8))
    plt.plot(gt_enu[:, 0], gt_enu[:, 1], 'g-', label='GNSS (Ground Truth)')
    plt.plot(ekf_positions[:, 0], ekf_positions[:, 1], 'b-', label='EKF (GNSS + IMU)')
    plt.plot(imu_dr_positions[:, 0], imu_dr_positions[:, 1], 'r--', label='IMU Pure DR')
    
    plt.title("Trajectory Comparison")
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.savefig('results/figures/03_ground_truth_vs_ekf.png')
    print("Saved trajectory plot.")

if __name__ == "__main__":
    # Test on one of the downloaded IO-VNBD files
    data_path = r"IO-VNBD/Unsynchronised V and S Dataset/Uncategorised IOVNB (V and S) Dataset/S-Dataset/S-Vta10.csv"
    run_experiment(data_path, outage_start=30.0, outage_duration=30.0)

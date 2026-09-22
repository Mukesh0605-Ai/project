import numpy as np
import matplotlib.pyplot as plt
import os
from src.data.loader import load_s_dataset
from src.navigation.state import NavigationState
from src.fusion.ekf import ErrorStateEKF
from src.utils.geodesy import ENUConverter
from src.ai.velocity_model import VelocityModel
from src.fusion.map_matching import MapMatcher

def run_pipeline(csv_path: str, outage_start: float, outage_duration: float):
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
    
    # Initialize Map Matcher with simulated road network
    road_network = MapMatcher.extract_route_from_gnss(gt_enu)
    map_matcher = MapMatcher(road_network, max_snap_distance=15.0)
    print("Initialized simulated road map.")
    
    # Load AI Model
    model = VelocityModel(window_size=20)
    try:
        model.load("src/models/velocity_model.pkl")
        print("Loaded trained AI Velocity Model.")
    except Exception as e:
        print(f"Could not load AI model: {e}")
        return

    ekf = ErrorStateEKF()
    initial_orientation = np.array([1.0, 0.0, 0.0, 0.0])
    
    state = NavigationState(
        timestamp=packets[0].timestamp,
        position=np.zeros(3),
        velocity=np.zeros(3),
        orientation=initial_orientation,
        accel_bias=np.zeros(3),
        gyro_bias=np.zeros(3)
    )
    
    trajectory = []
    
    print(f"Simulating Outage: {outage_start}s to {outage_start + outage_duration}s")
    t0 = packets[0].timestamp
    
    window_size = 20
    buffer = []
    
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
        
        if not is_outage and packet.gnss_lat_lon_alt is not None and not np.isnan(packet.gnss_lat_lon_alt[0]):
            enu = enu_conv.lla_to_enu(packet.gnss_lat_lon_alt[0], packet.gnss_lat_lon_alt[1], packet.gnss_lat_lon_alt[2])
            acc = packet.gnss_accuracy if packet.gnss_accuracy else 5.0
            state = ekf.update_gnss(state, enu, acc)
        elif is_outage and len(buffer) == window_size:
            # 1. AI Velocity Update
            x = np.array(buffer, dtype=np.float32)[np.newaxis, ...]
            pred_speed = model.predict(x)[0]
            state = ekf.update_velocity(state, pred_speed, accuracy=2.0)
            
            # 2. Non-Holonomic Constraints (NHC)
            # Enforce zero lateral and vertical slip
            state = ekf.update_nhc(state, lateral_accuracy=0.5, vertical_accuracy=0.5)
            
            # 3. Map Matching
            # Periodically snap to road center (pseudo-GNSS update)
            if i % 100 == 0:  # Roughly 1Hz at 100Hz IMU
                matched_pos = map_matcher.get_matched_position(state.position)
                # Only apply if it actually snapped
                if np.linalg.norm(matched_pos - state.position) > 0.01:
                    # Provide matched position to EKF as a measurement
                    state = ekf.update_gnss(state, matched_pos, accuracy=3.0)
            
        trajectory.append(state.position.copy())
        
    trajectory = np.array(trajectory)
    
    os.makedirs('results/figures', exist_ok=True)
    plt.figure(figsize=(10, 8))
    plt.plot(gt_enu[:, 0], gt_enu[:, 1], 'g-', alpha=0.5, linewidth=3, label='Road / GNSS Ground Truth')
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'r-', linewidth=1.5, label='Fully Constrained EKF (AI + NHC + Map)')
    
    plt.title("Stage 5: Full Intelligent Dead Reckoning Pipeline")
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.savefig('results/figures/05_nhc_map_matching.png')
    print("Saved Stage 5 trajectory plot.")

if __name__ == "__main__":
    data_path = r"IO-VNBD/Unsynchronised V and S Dataset/Uncategorised IOVNB (V and S) Dataset/S-Dataset/S-Vta10.csv"
    run_pipeline(data_path, outage_start=30.0, outage_duration=30.0)

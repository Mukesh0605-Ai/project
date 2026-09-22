import numpy as np
import typing
from src.navigation.state import SensorPacket, NavigationState
from src.navigation.engine import NavigationEngine
from src.ai.features import create_windows
from src.ai.model import AIVelocityEstimator
from src.fusion.map_matching import MapMatcher

def generate_synthetic_trajectory(duration: float = 60.0, dt: float = 0.05) -> typing.Tuple[np.ndarray, typing.List[SensorPacket], np.ndarray]:
    """
    Generates a realistic synthetic vehicle trajectory in ENU coordinate frame (100 Hz IMU).
    Simulates forward acceleration, turning, and constant velocity cruising.
    
    Returns:
        timestamps: (N,) array
        packets: List of SensorPackets (IMU + GNSS)
        ground_truth_enu: (N, 3) ENU positions
    """
    num_steps = int(duration / dt)
    timestamps = np.linspace(0, duration, num_steps)
    
    # Ground truth trajectory generation
    ground_truth_enu = np.zeros((num_steps, 3))
    ground_truth_vel = np.zeros((num_steps, 3))
    
    # Kinematic profile: speed ramp to 12 m/s (~43 km/h), steady cruise, slight right curve
    forward_speed = np.zeros(num_steps)
    heading = np.zeros(num_steps)
    
    for i, t in enumerate(timestamps):
        if t < 10.0:
            # Acceleration phase
            forward_speed[i] = (t / 10.0) * 12.0
        elif t < 40.0:
            # Steady cruise with slight turn between t=20 and t=30
            forward_speed[i] = 12.0
            if t >= 20.0 and t <= 30.0:
                heading[i] = (t - 20.0) * (np.pi / 20.0)  # 45 deg turn total
            elif t > 30.0:
                heading[i] = np.pi / 4.0
        else:
            # Deceleration phase
            decel_t = (t - 40.0) / 20.0
            forward_speed[i] = max(0.0, 12.0 * (1.0 - decel_t))
            heading[i] = np.pi / 4.0

    # Integrate velocity to positions
    pos = np.zeros(3)
    for i in range(1, num_steps):
        dt_step = timestamps[i] - timestamps[i-1]
        v_east = forward_speed[i] * np.sin(heading[i])
        v_north = forward_speed[i] * np.cos(heading[i])
        ground_truth_vel[i] = np.array([v_east, v_north, 0.0])
        pos = pos + ground_truth_vel[i] * dt_step
        ground_truth_enu[i] = pos

    # Generate noisy sensor measurements
    packets = []
    accel_noise_std = 0.08
    gyro_noise_std = 0.005
    
    for i, t in enumerate(timestamps):
        # Derive linear acceleration in ENU
        if i == 0:
            acc_enu = np.zeros(3)
        else:
            acc_enu = (ground_truth_vel[i] - ground_truth_vel[i-1]) / dt
            
        # Add gravity component (Up = Z)
        acc_enu[2] += 9.80665
        
        # Yaw rate
        if i == 0:
            yaw_rate = 0.0
        else:
            yaw_rate = (heading[i] - heading[i-1]) / dt

        # Convert ENU accel & gyro to Body frame
        h = heading[i]
        cos_h, sin_h = np.cos(h), np.sin(h)
        acc_body_x = acc_enu[0] * cos_h - acc_enu[1] * sin_h
        acc_body_y = acc_enu[0] * sin_h + acc_enu[1] * cos_h
        acc_body_z = acc_enu[2]
        
        acc_body = np.array([acc_body_x, acc_body_y, acc_body_z]) + np.random.normal(0, accel_noise_std, 3)
        gyro_body = np.array([0.0, 0.0, yaw_rate]) + np.random.normal(0, gyro_noise_std, 3)
        
        # Simulated GNSS: Outage between t=15.0s and t=45.0s (30-second blackout tunnel)
        is_outage = (t >= 15.0 and t <= 45.0)
        
        gnss_enu = None
        gnss_speed = None
        gnss_accuracy = None
        
        if not is_outage:
            gnss_enu = ground_truth_enu[i] + np.random.normal(0, 1.0, 3)
            gnss_speed = forward_speed[i] * 3.6 # km/h
            gnss_accuracy = 2.0
            
        packet = SensorPacket(
            timestamp=t,
            accelerometer=acc_body,
            gyroscope=gyro_body,
            gnss_speed=gnss_speed,
            gnss_accuracy=gnss_accuracy,
            metadata={"gnss_enu": gnss_enu if not is_outage else None, "is_outage": is_outage}
        )
        packets.append(packet)

    return timestamps, packets, ground_truth_enu

def run_fused_pipeline() -> typing.Dict[str, typing.Any]:
    """
    Executes the complete fused navigation engine over synthetic data.
    Trains AI Velocity model on pre-outage segment and evaluates tracking during blackout.
    """
    np.random.seed(42)
    timestamps, packets, ground_truth_enu = generate_synthetic_trajectory(duration=60.0, dt=0.05)
    
    # 1. Prepare AI Velocity Estimator by training on pre-outage window features
    pre_outage_packets = [p for p in packets if p.timestamp < 15.0]
    X_train, y_train = create_windows(pre_outage_packets, window_size=20)
    
    ai_estimator = AIVelocityEstimator(window_size=20)
    if len(X_train) > 0:
        ai_estimator.train(X_train, y_train)
        
    # 2. Extract road network graph for Map Matcher (sparse ground truth sample)
    reference_road = MapMatcher.extract_route_from_gnss(ground_truth_enu)
    
    # 3. Initialize Navigation Engine
    engine = NavigationEngine(ai_model=ai_estimator, window_size=20)
    engine.set_map_route(reference_road)
    
    initial_state = NavigationState(
        timestamp=timestamps[0],
        position=ground_truth_enu[0].copy(),
        velocity=np.zeros(3),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        accel_bias=np.zeros(3),
        gyro_bias=np.zeros(3),
        navigation_mode="INITIALIZING"
    )
    engine.initialize(initial_state)
    
    # 4. Run Loop
    estimated_positions = []
    modes = []
    
    for packet in packets:
        state = engine.process_packet(packet)
        estimated_positions.append(state.position.copy())
        modes.append(state.navigation_mode)
        
    estimated_positions = np.array(estimated_positions)
    
    # 5. Evaluate position error during GNSS blackout (t=15 to t=45)
    outage_indices = np.where((timestamps >= 15.0) & (timestamps <= 45.0))[0]
    outage_gt = ground_truth_enu[outage_indices]
    outage_est = estimated_positions[outage_indices]
    
    pos_errors = np.linalg.norm(outage_gt - outage_est, axis=1)
    rmse_error = float(np.sqrt(np.mean(pos_errors**2)))
    max_drift = float(np.max(pos_errors))
    final_outage_drift = float(pos_errors[-1])
    
    results = {
        "status": "SUCCESS",
        "num_packets": len(packets),
        "outage_duration_sec": 30.0,
        "outage_rmse_meters": rmse_error,
        "outage_max_drift_meters": max_drift,
        "outage_final_drift_meters": final_outage_drift,
        "ai_model_trained": ai_estimator.is_trained,
        "reference_road_nodes": len(reference_road)
    }
    return results

if __name__ == "__main__":
    res = run_fused_pipeline()
    print("=== NAVIGATION FUSION PIPELINE BENCHMARK ===")
    for k, v in res.items():
        print(f"  {k}: {v}")

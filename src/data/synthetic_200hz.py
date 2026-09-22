import time
import typing
import numpy as np
from src.navigation.state import SensorPacket, NavigationState
from src.navigation.engine import NavigationEngine
from src.sensors.adapters import ExternalHighRateIMUAdapter

def generate_synthetic_200hz_trajectory(
    duration: float = 60.0,
    dt: float = 0.005
) -> typing.Tuple[np.ndarray, typing.List[SensorPacket], np.ndarray]:
    """
    Generates realistic 200 Hz synthetic vehicle trajectory data (dt = 0.005s).
    Simulates high-rate IMU motion (acceleration, turning, cruising) with 30-second GNSS blackout.
    
    Returns:
        timestamps: (N,) array
        packets: List of SensorPackets at 200 Hz
        ground_truth_enu: (N, 3) ENU positions
    """
    num_steps = int(duration / dt)
    timestamps = np.linspace(0, duration, num_steps)

    ground_truth_enu = np.zeros((num_steps, 3))
    ground_truth_vel = np.zeros((num_steps, 3))

    forward_speed = np.zeros(num_steps)
    heading = np.zeros(num_steps)

    for i, t in enumerate(timestamps):
        if t < 10.0:
            forward_speed[i] = (t / 10.0) * 15.0  # Ramp up to 15 m/s (~54 km/h)
        elif t < 40.0:
            forward_speed[i] = 15.0
            if 20.0 <= t <= 30.0:
                heading[i] = (t - 20.0) * (np.pi / 20.0)  # 45-degree smooth turn
            elif t > 30.0:
                heading[i] = np.pi / 4.0
        else:
            decel_t = (t - 40.0) / 20.0
            forward_speed[i] = max(0.0, 15.0 * (1.0 - decel_t))
            heading[i] = np.pi / 4.0

    pos = np.zeros(3)
    for i in range(1, num_steps):
        dt_step = timestamps[i] - timestamps[i-1]
        v_east = forward_speed[i] * np.sin(heading[i])
        v_north = forward_speed[i] * np.cos(heading[i])
        ground_truth_vel[i] = np.array([v_east, v_north, 0.0])
        pos = pos + ground_truth_vel[i] * dt_step
        ground_truth_enu[i] = pos

    packets = []
    accel_noise_std = 0.05
    gyro_noise_std = 0.003

    for i, t in enumerate(timestamps):
        if i == 0:
            acc_enu = np.zeros(3)
            yaw_rate = 0.0
        else:
            acc_enu = (ground_truth_vel[i] - ground_truth_vel[i-1]) / dt
            yaw_rate = (heading[i] - heading[i-1]) / dt

        acc_enu[2] += 9.80665  # Add gravity

        h = heading[i]
        cos_h, sin_h = np.cos(h), np.sin(h)
        acc_body_x = acc_enu[0] * cos_h - acc_enu[1] * sin_h
        acc_body_y = acc_enu[0] * sin_h + acc_enu[1] * cos_h
        acc_body_z = acc_enu[2]

        acc_body = np.array([acc_body_x, acc_body_y, acc_body_z]) + np.random.normal(0, accel_noise_std, 3)
        gyro_body = np.array([0.0, 0.0, yaw_rate]) + np.random.normal(0, gyro_noise_std, 3)

        is_outage = (15.0 <= t <= 45.0)
        gnss_enu = ground_truth_enu[i] + np.random.normal(0, 1.0, 3) if not is_outage else None

        packet = SensorPacket(
            timestamp=t,
            accelerometer=acc_body,
            gyroscope=gyro_body,
            gnss_speed=forward_speed[i] * 3.6 if not is_outage else None,
            gnss_accuracy=1.5 if not is_outage else None,
            metadata={"gnss_enu": gnss_enu, "is_outage": is_outage, "sampling_rate_hz": 200.0}
        )
        packets.append(packet)

    return timestamps, packets, ground_truth_enu

def run_200hz_benchmark() -> typing.Dict[str, typing.Any]:
    """
    Evaluates 200 Hz Edge Mode propagation performance and throughput metrics.
    """
    np.random.seed(42)
    timestamps, packets, ground_truth_enu = generate_synthetic_200hz_trajectory(duration=60.0, dt=0.005)

    adapter = ExternalHighRateIMUAdapter(target_rate_hz=200.0)
    engine = NavigationEngine(adapter=adapter)

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

    start_time = time.perf_counter()
    estimated_positions = []
    
    for packet in packets:
        state = engine.process_packet(packet)
        estimated_positions.append(state.position.copy())

    elapsed_sec = time.perf_counter() - start_time
    total_packets = len(packets)
    throughput_hz = total_packets / max(1e-6, elapsed_sec)
    latency_us_per_packet = (elapsed_sec / total_packets) * 1e6

    estimated_positions = np.array(estimated_positions)

    # Outage error metrics
    outage_mask = (timestamps >= 15.0) & (timestamps <= 45.0)
    outage_gt = ground_truth_enu[outage_mask]
    outage_est = estimated_positions[outage_mask]
    
    errors = np.linalg.norm(outage_gt - outage_est, axis=1)
    max_drift = float(np.max(errors))
    final_drift = float(errors[-1])
    rmse = float(np.sqrt(np.mean(errors**2)))

    return {
        "status": "SUCCESS",
        "imu_sampling_rate_hz": 200.0,
        "total_packets_processed": total_packets,
        "elapsed_time_sec": elapsed_sec,
        "throughput_packets_per_sec": throughput_hz,
        "latency_us_per_packet": latency_us_per_packet,
        "realtime_speedup_factor": throughput_hz / 200.0,
        "outage_duration_sec": 30.0,
        "outage_rmse_meters": rmse,
        "outage_max_drift_meters": max_drift,
        "outage_final_drift_meters": final_drift
    }

if __name__ == "__main__":
    res = run_200hz_benchmark()
    print("=== SENSOR-AGNOSTIC 200 Hz EDGE MODE BENCHMARK ===")
    for k, v in res.items():
        print(f"  {k}: {v}")

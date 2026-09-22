import numpy as np
from typing import List, Tuple
from src.navigation.state import SensorPacket

def create_windows(packets: List[SensorPacket], window_size: int = 20) -> Tuple[np.ndarray, np.ndarray]:
    """
    Creates temporal sliding windows of IMU data and matches them with GNSS speed.
    
    Inputs:
    - packets: Chronological list of SensorPackets.
    - window_size: Number of consecutive samples in one window.
    
    Outputs:
    - X: (num_windows, window_size, 6) tensor of [accel, gyro]
    - Y: (num_windows, 1) tensor of ground-truth GNSS speed for the window's end.
    """
    X = []
    Y = []
    
    # We require gnss_speed to be present to create a target.
    # We will build windows that end at a valid GNSS measurement.
    
    # We will maintain a running buffer of the last `window_size` samples
    buffer = []
    
    for packet in packets:
        # Extract the 6-DOF IMU features
        # Some packets might have missing data, we pad with 0s if necessary
        accel = packet.accelerometer if packet.accelerometer is not None else np.zeros(3)
        gyro = packet.gyroscope if packet.gyroscope is not None else np.zeros(3)
        
        feature_vector = np.concatenate([accel, gyro])
        buffer.append(feature_vector)
        
        if len(buffer) > window_size:
            buffer.pop(0)
            
        if len(buffer) == window_size:
            # Check if this packet has a valid gnss speed
            if packet.gnss_speed is not None and not np.isnan(packet.gnss_speed):
                X.append(np.array(buffer))
                
                # Convert km/h to m/s
                speed_ms = packet.gnss_speed * (1000.0 / 3600.0)
                Y.append([speed_ms])
                
    return np.array(X, dtype=np.float32), np.array(Y, dtype=np.float32)

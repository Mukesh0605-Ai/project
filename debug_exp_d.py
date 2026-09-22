import numpy as np
import os
import json
from src.data.loader import load_s_dataset
from src.navigation.state import NavigationState
from src.fusion.ekf import ErrorStateEKF
from src.utils.geodesy import ENUConverter
from src.ai.velocity_model import VelocityModel
from src.fusion.map_matching import MapMatcher

data_path = r"IO-VNBD/Unsynchronised V and S Dataset/Uncategorised IOVNB (V and S) Dataset/S-Dataset/S-Vta10.csv"
packets = load_s_dataset(data_path)
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

road_network = MapMatcher.extract_route_from_gnss(gt_enu)
map_matcher = MapMatcher(road_network, max_snap_distance=15.0)
model = VelocityModel(window_size=20)
model.load("src/models/velocity_model.pkl")

outage_start = 30.0
outage_duration = 30.0

ekf = ErrorStateEKF()
state = NavigationState(
    timestamp=packets[0].timestamp,
    position=np.zeros(3),
    velocity=np.zeros(3),
    orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    accel_bias=np.zeros(3),
    gyro_bias=np.zeros(3)
)

t0 = packets[0].timestamp
buffer = []

for i in range(1, len(packets)):
    packet = packets[i]
    dt = packet.timestamp - packets[i-1].timestamp
    if dt <= 0: continue
    t_rel = packet.timestamp - t0
    
    accel = packet.accelerometer if packet.accelerometer is not None else np.zeros(3)
    gyro = packet.gyroscope if packet.gyroscope is not None else np.zeros(3)
    buffer.append(np.concatenate([accel, gyro]))
    if len(buffer) > 20: buffer.pop(0)
    
    state = ekf.predict(state, packet, dt)
    is_outage = outage_start <= t_rel <= (outage_start + outage_duration)
    
    has_gnss = packet.gnss_lat_lon_alt is not None and not np.isnan(packet.gnss_lat_lon_alt[0])
    enu = enu_conv.lla_to_enu(packet.gnss_lat_lon_alt[0], packet.gnss_lat_lon_alt[1], packet.gnss_lat_lon_alt[2]) if has_gnss else None

    if not is_outage and has_gnss:
        state = ekf.update_gnss(state, enu, 5.0)
    elif is_outage:
        if len(buffer) == 20:
            x = np.array(buffer, dtype=np.float32)[np.newaxis, ...]
            pred_speed = model.predict(x)[0]
            state = ekf.update_velocity(state, pred_speed, accuracy=2.0)
        state = ekf.update_nhc(state, lateral_accuracy=0.5, vertical_accuracy=0.5)
        if i % 100 == 0:
            matched = map_matcher.get_matched_position(state.position)
            if np.linalg.norm(matched - state.position) > 0.01:
                state = ekf.update_gnss(state, matched, accuracy=3.0)
                
    if is_outage and i % 100 == 0:
        print(f"t={t_rel:.1f}, pos={state.position}, vel={state.velocity}, q={state.orientation}")

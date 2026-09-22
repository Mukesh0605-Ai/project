import threading
import time
import os
import sys

# Ensure repository root is in python path
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

import numpy as np
from flask import Flask, request, jsonify, render_template, send_from_directory
from src.navigation.state import NavigationState, SensorPacket
from src.fusion.ekf import ErrorStateEKF
from src.utils.geodesy import ENUConverter
from src.ai.velocity_model import VelocityModel
from src.data.loader import load_s_dataset

# Resolve prototype path for static file serving
PROTOTYPE_DIR = os.path.join(BASE_DIR, "prototype")

app = Flask(__name__, static_folder=PROTOTYPE_DIR, template_folder=PROTOTYPE_DIR)

# Global Navigation Engine State
ekf = ErrorStateEKF()
nav_state = None
enu_conv = None
ai_model = VelocityModel(window_size=20)
try:
    ai_model.load("src/models/velocity_model.pkl")
    print("AI Model loaded successfully.")
except Exception as e:
    print("Warning: Could not load AI Model:", e)

imu_buffer = []
is_outage = False
outage_start_time = 0.0
last_imu = {"ax": 0.0, "ay": 0.0, "az": 0.0, "gx": 0.0, "gy": 0.0, "gz": 0.0}
sim_thread = None
sim_running = False

# Enable CORS for all responses
@app.after_request
def add_cors_headers(response):
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type,Authorization'
    response.headers['Access-Control-Allow-Methods'] = 'GET,POST,OPTIONS'
    return response

@app.route('/', methods=['GET'])
def index():
    return send_from_directory(PROTOTYPE_DIR, 'index.html')

@app.route('/<path:filename>', methods=['GET'])
def static_files(filename):
    return send_from_directory(PROTOTYPE_DIR, filename)

@app.route('/favicon.ico', methods=['GET'])
def favicon():
    return '', 204

@app.route('/state', methods=['GET'])
def get_state():
    global nav_state, enu_conv, is_outage, last_imu, outage_start_time, sim_running
    if nav_state is None or enu_conv is None:
        return jsonify({
            "status": "waiting",
            "sim_running": sim_running,
            "is_outage": is_outage
        })
    
    lat, lon, alt = enu_conv.enu_to_lla(nav_state.position[0], nav_state.position[1], nav_state.position[2])
    speed_kmh = float(np.linalg.norm(nav_state.velocity) * 3.6)
    
    # Estimate heading from orientation quaternion (yaw)
    q = nav_state.orientation
    # yaw = arctan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
    cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
    yaw_deg = float(np.degrees(np.arctan2(siny_cosp, cosy_cosp))) % 360
    
    cardinal = ["N", "NE", "E", "SE", "S", "SW", "W", "NW", "N"][int((yaw_deg + 22.5) / 45) % 8]
    heading_str = f"{cardinal} {int(yaw_deg)}°"
    
    # Confidence calculation
    if is_outage:
        blackout_secs = float(time.time() - outage_start_time) if outage_start_time > 0 else 5.0
        confidence = max(65, int(98 - (blackout_secs * 0.8)))
    else:
        blackout_secs = 0.0
        confidence = 98

    return jsonify({
        "status": "active",
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "speed_kmh": round(speed_kmh, 1),
        "heading": heading_str,
        "confidence": confidence,
        "is_outage": is_outage,
        "blackout_duration": round(blackout_secs, 1),
        "imu": last_imu,
        "sim_running": sim_running,
        "pos_enu": [float(nav_state.position[0]), float(nav_state.position[1]), float(nav_state.position[2])],
        "vel_enu": [float(nav_state.velocity[0]), float(nav_state.velocity[1]), float(nav_state.velocity[2])]
    })

@app.route('/reset', methods=['POST'])
def reset():
    global nav_state, enu_conv, imu_buffer, is_outage, outage_start_time
    data = request.json or {}
    lat = data.get('lat')
    lon = data.get('lon')
    alt = data.get('alt', 0.0)
    
    if lat is not None and lon is not None:
        enu_conv = ENUConverter(lat, lon, alt)
        nav_state = NavigationState(
            timestamp=0.0,
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3)
        )
        imu_buffer = []
        is_outage = False
        outage_start_time = 0.0
        return jsonify({"status": "reset_ok"})
    return jsonify({"error": "Missing coordinates"}), 400

@app.route('/outage', methods=['POST', 'OPTIONS'])
def set_outage():
    global is_outage, outage_start_time
    if request.method == 'OPTIONS':
        return jsonify({})
    data = request.json or {}
    is_outage = data.get('outage', not is_outage)
    if is_outage:
        outage_start_time = time.time()
    else:
        outage_start_time = 0.0
    return jsonify({"status": "outage_updated", "is_outage": is_outage})

@app.route('/update', methods=['POST'])
def update():
    global nav_state, imu_buffer, is_outage, last_imu
    
    if nav_state is None or enu_conv is None:
        return jsonify({"error": "Not initialized"}), 400
        
    data = request.json or {}
    batch = data.get('batch', [])
    
    for packet_data in batch:
        t = packet_data['t']
        dt = packet_data['dt']
        
        acc = np.array(packet_data['acc']) if packet_data.get('acc') else np.zeros(3)
        gyr = np.array(packet_data['gyr']) if packet_data.get('gyr') else np.zeros(3)
        gnss = packet_data.get('gnss') # [lat, lon, alt, accuracy]
        
        last_imu = {
            "ax": round(float(acc[0]), 2),
            "ay": round(float(acc[1]), 2),
            "az": round(float(acc[2]), 2),
            "gx": round(float(gyr[0]), 2),
            "gy": round(float(gyr[1]), 2),
            "gz": round(float(gyr[2]), 2)
        }

        packet = SensorPacket(
            timestamp=t,
            accelerometer=acc,
            gyroscope=gyr,
            gnss_lat_lon_alt=np.array(gnss[:3]) if gnss else None,
            gnss_accuracy=gnss[3] if gnss else None
        )
        
        # Predict Nominal State + Error Covariance
        if dt > 0:
            nav_state = ekf.predict(nav_state, packet, dt)
        
        imu_buffer.append(np.concatenate([acc, gyr]))
        if len(imu_buffer) > 20:
            imu_buffer.pop(0)
            
        # Measurement Updates
        if not is_outage and gnss is not None:
            enu = enu_conv.lla_to_enu(gnss[0], gnss[1], gnss[2])
            nav_state = ekf.update_gnss(nav_state, enu, gnss[3])
        elif is_outage and len(imu_buffer) == 20:
            # AI 1D-CNN+GRU Temporal Velocity Update
            x = np.array(imu_buffer, dtype=np.float32)[np.newaxis, ...]
            try:
                pred_speed = float(ai_model.predict(x)[0])
                nav_state = ekf.update_velocity(nav_state, pred_speed, accuracy=1.5)
            except Exception:
                pass
            
            # Non-Holonomic Constraint (NHC) Lateral/Vertical Bounds
            nav_state = ekf.update_nhc(nav_state, lateral_accuracy=0.5, vertical_accuracy=0.5)
            
    lat, lon, alt = enu_conv.enu_to_lla(nav_state.position[0], nav_state.position[1], nav_state.position[2])
    return jsonify({
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "is_outage": is_outage
    })

# Simulation thread runner
def _run_dataset_playback():
    global sim_running, is_outage, outage_start_time
    dataset_path = os.path.join(BASE_DIR, "IO-VNBD", "Unsynchronised V and S Dataset", "Uncategorised IOVNB (V and S) Dataset", "S-Dataset", "S-Vta10.csv")
    if not os.path.exists(dataset_path):
        print("Dataset not found at", dataset_path)
        sim_running = False
        return

    while sim_running:
        print("Starting Dataset Playback Loop...")
        packets = load_s_dataset(dataset_path)
        p0 = next((p for p in packets if p.gnss_lat_lon_alt is not None), None)
        if not p0:
            break
            
        # Reset server
        global enu_conv, nav_state, imu_buffer
        enu_conv = ENUConverter(p0.gnss_lat_lon_alt[0], p0.gnss_lat_lon_alt[1], p0.gnss_lat_lon_alt[2])
        nav_state = NavigationState(
            timestamp=0.0,
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3)
        )
        imu_buffer = []
        is_outage = False
        outage_start_time = 0.0
        
        t0 = packets[0].timestamp
        batch = []
        last_time = time.time()
        
        for i in range(1, len(packets)):
            if not sim_running:
                break
                
            p = packets[i]
            dt = p.timestamp - packets[i-1].timestamp
            if dt <= 0:
                continue
                
            acc = p.accelerometer.tolist() if p.accelerometer is not None else [0, 0, 0]
            gyr = p.gyroscope.tolist() if p.gyroscope is not None else [0, 0, 0]
            gnss = None
            if p.gnss_lat_lon_alt is not None and not np.isnan(p.gnss_lat_lon_alt[0]):
                gnss = p.gnss_lat_lon_alt.tolist() + [p.gnss_accuracy or 5.0]
                
            batch.append({
                "t": p.timestamp,
                "dt": dt,
                "acc": acc,
                "gyr": gyr,
                "gnss": gnss
            })
            
            if len(batch) >= 5:
                # Update EKF directly
                for item in batch:
                    acc_arr = np.array(item['acc'])
                    gyr_arr = np.array(item['gyr'])
                    global last_imu
                    last_imu = {
                        "ax": round(float(acc_arr[0]), 2),
                        "ay": round(float(acc_arr[1]), 2),
                        "az": round(float(acc_arr[2]), 2),
                        "gx": round(float(gyr_arr[0]), 2),
                        "gy": round(float(gyr_arr[1]), 2),
                        "gz": round(float(gyr_arr[2]), 2)
                    }
                    pkt = SensorPacket(
                        timestamp=item['t'],
                        accelerometer=acc_arr,
                        gyroscope=gyr_arr,
                        gnss_lat_lon_alt=np.array(item['gnss'][:3]) if item['gnss'] else None,
                        gnss_accuracy=item['gnss'][3] if item['gnss'] else None
                    )
                    nav_state = ekf.predict(nav_state, pkt, item['dt'])
                    imu_buffer.append(np.concatenate([acc_arr, gyr_arr]))
                    if len(imu_buffer) > 20:
                        imu_buffer.pop(0)
                        
                    if not is_outage and item['gnss']:
                        enu = enu_conv.lla_to_enu(item['gnss'][0], item['gnss'][1], item['gnss'][2])
                        nav_state = ekf.update_gnss(nav_state, enu, item['gnss'][3])
                    elif is_outage and len(imu_buffer) == 20:
                        x = np.array(imu_buffer, dtype=np.float32)[np.newaxis, ...]
                        try:
                            speed = float(ai_model.predict(x)[0])
                            nav_state = ekf.update_velocity(nav_state, speed, accuracy=1.5)
                        except Exception:
                            pass
                        nav_state = ekf.update_nhc(nav_state, lateral_accuracy=0.5, vertical_accuracy=0.5)

                batch = []
                time.sleep(0.04) # Smooth real-time rate

@app.route('/api/simulation/start', methods=['POST', 'OPTIONS'])
def start_simulation():
    global sim_thread, sim_running
    if request.method == 'OPTIONS':
        return jsonify({})
    if not sim_running:
        sim_running = True
        sim_thread = threading.Thread(target=_run_dataset_playback, daemon=True)
        sim_thread.start()
    return jsonify({"status": "simulation_started", "sim_running": True})

@app.route('/api/simulation/stop', methods=['POST', 'OPTIONS'])
def stop_simulation():
    global sim_running
    if request.method == 'OPTIONS':
        return jsonify({})
    sim_running = False
    return jsonify({"status": "simulation_stopped", "sim_running": False})

if __name__ == '__main__':
    # Automatically start simulation thread on startup
    sim_running = True
    sim_thread = threading.Thread(target=_run_dataset_playback, daemon=True)
    sim_thread.start()
    app.run(host='0.0.0.0', port=3000, debug=False)

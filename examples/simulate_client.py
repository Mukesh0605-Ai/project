import time
import urllib.request
import json
import numpy as np
from src.data.loader import load_s_dataset

# Configuration
DATA_PATH = r"IO-VNBD/Unsynchronised V and S Dataset/Uncategorised IOVNB (V and S) Dataset/S-Dataset/S-Vta10.csv"
SERVER_URL = "http://127.0.0.1:5000"
OUTAGE_START = 30.0
OUTAGE_DURATION = 30.0
PLAYBACK_SPEED = 2.0 # Speed multiplier for the demo

def run_simulation():
    print("Loading Dataset...")
    packets = load_s_dataset(DATA_PATH)
    
    p0 = next((p for p in packets if p.gnss_lat_lon_alt is not None), None)
    if not p0:
        print("No GNSS data found.")
        return
        
    print(f"Initializing Edge Server at {SERVER_URL}/reset...")
    req = urllib.request.Request(f"{SERVER_URL}/reset", 
                                 data=json.dumps({
                                     "lat": p0.gnss_lat_lon_alt[0],
                                     "lon": p0.gnss_lat_lon_alt[1],
                                     "alt": p0.gnss_lat_lon_alt[2]
                                 }).encode('utf-8'),
                                 headers={'Content-Type': 'application/json'})
    try:
        resp = urllib.request.urlopen(req)
        status_code = resp.getcode()
    except Exception as e:
        print("Failed to initialize server:", str(e))
        return
        
    if status_code != 200:
        print("Failed to initialize server. Status:", status_code)
        return
        
    print("Server Initialized. Starting Simulation...")
    
    t0 = packets[0].timestamp
    batch = []
    
    current_outage_state = False
    
    # We will simulate pushing data in batches of 10
    last_real_time = time.time()
    
    for i in range(1, len(packets)):
        packet = packets[i]
        dt = packet.timestamp - packets[i-1].timestamp
        if dt <= 0: continue
        
        t_rel = packet.timestamp - t0
        
        # Determine outage state
        is_outage = bool(OUTAGE_START <= t_rel <= (OUTAGE_START + OUTAGE_DURATION))
        
        if is_outage != current_outage_state:
            current_outage_state = is_outage
            req = urllib.request.Request(f"{SERVER_URL}/outage", 
                                         data=json.dumps({"outage": is_outage}).encode('utf-8'),
                                         headers={'Content-Type': 'application/json'})
            urllib.request.urlopen(req)
            print(f"\n---> OUTAGE STATE CHANGED: {is_outage} at t={t_rel:.1f}s\n")
            
        acc = packet.accelerometer.tolist() if packet.accelerometer is not None else [0,0,0]
        gyr = packet.gyroscope.tolist() if packet.gyroscope is not None else [0,0,0]
        
        gnss = None
        if packet.gnss_lat_lon_alt is not None and not np.isnan(packet.gnss_lat_lon_alt[0]):
            gnss = packet.gnss_lat_lon_alt.tolist() + [packet.gnss_accuracy or 5.0]
            
        batch.append({
            "t": packet.timestamp,
            "dt": dt,
            "acc": acc,
            "gyr": gyr,
            "gnss": gnss
        })
        
        if len(batch) >= 10:
            req = urllib.request.Request(f"{SERVER_URL}/update", 
                                         data=json.dumps({"batch": batch}).encode('utf-8'),
                                         headers={'Content-Type': 'application/json'})
            urllib.request.urlopen(req)
            batch = []
            
            # Throttle to simulate real-time playback
            elapsed = time.time() - last_real_time
            sleep_time = (10.0 * 0.01 / PLAYBACK_SPEED) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            last_real_time = time.time()
            
            print(f"\rSimulating t={t_rel:.1f}s / {packets[-1].timestamp - t0:.1f}s", end="")
            
    print("\nSimulation Complete.")

if __name__ == "__main__":
    run_simulation()

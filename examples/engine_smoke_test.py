import numpy as np
from src.navigation.state import SensorPacket, NavigationState
from src.navigation.engine import NavigationEngine
from src.data.replay import DatasetReplay

def main():
    print("SYNTHETIC SOFTWARE TEST — NOT REAL DATA")
    print("Initializing Smoke Test...")

    # 1. Create a dummy initial state
    initial_state = NavigationState(
        timestamp=0.0,
        position=np.zeros(3),
        velocity=np.zeros(3),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        accel_bias=np.zeros(3),
        gyro_bias=np.zeros(3)
    )

    # 2. Create the engine
    engine = NavigationEngine()
    engine.initialize(initial_state)
    print("Engine Initialized.")

    # 3. Create dummy synthetic packets
    packets = [
        SensorPacket(timestamp=0.1, accelerometer=np.array([0, 0, 9.8])),
        SensorPacket(timestamp=0.2, accelerometer=np.array([0, 0, 9.8]))
    ]
    replay = DatasetReplay(packets)

    # 4. Process packets (Should hit NotImplementedError because data is not verified)
    packet = replay.next_packet()
    while packet:
        print(f"Feeding packet at t={packet.timestamp} to engine...")
        try:
            engine.process_imu(packet)
        except NotImplementedError as e:
            print(f"Caught expected algorithmic block: {e}")
        packet = replay.next_packet()

    print("Smoke test structural validation complete.")

if __name__ == "__main__":
    main()

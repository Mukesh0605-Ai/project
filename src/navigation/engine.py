import numpy as np
import typing
from collections import deque
from src.navigation.state import SensorPacket, NavigationState
from src.estimation.ekf import NavigationEKF
from src.ai.model import AIVelocityEstimator
from src.ai.motion_classifier import MotionClass
from src.constraints.nhc import NonHolonomicConstraint
from src.map.map_matcher import ProbabilisticMapMatcher
from src.data.synchronization import SensorSynchronizer
from src.calibration.phone_vehicle_alignment import PhoneVehicleAligner
from src.fusion.confidence import ConfidenceManager
from src.fusion.gnss_quality import GNSSQualityMonitor, GNSSMode

from src.sensors.adapters import BaseSensorAdapter, SmartphoneSensorAdapter

class NavigationEngine:
    """
    Core Intelligent Dead Reckoning Engine.
    Coordinates real-time sensor fusion between Error-State EKF, 
    SIH 5-Class Motion Classifier, AI temporal velocity estimation,
    Non-Holonomic Constraints (NHC), probabilistic map matching,
    and Confidence & Drift Manager.
    Supports sensor-agnostic operation up to 200 Hz high-rate IMUs.
    """
    def __init__(self, 
                 ai_model: typing.Optional[AIVelocityEstimator] = None,
                 map_matcher: typing.Optional[ProbabilisticMapMatcher] = None,
                 synchronizer: typing.Optional[SensorSynchronizer] = None,
                 aligner: typing.Optional[PhoneVehicleAligner] = None,
                 gnss_monitor: typing.Optional[GNSSQualityMonitor] = None,
                 confidence_manager: typing.Optional[ConfidenceManager] = None,
                 adapter: typing.Optional[BaseSensorAdapter] = None,
                 window_size: int = 20):
        self._current_state: typing.Optional[NavigationState] = None
        self.ekf = NavigationEKF()
        self.ai_model = ai_model if ai_model is not None else AIVelocityEstimator(window_size=window_size)
        self.nhc = NonHolonomicConstraint()
        self.map_matcher = map_matcher if map_matcher is not None else ProbabilisticMapMatcher()
        self.synchronizer = synchronizer if synchronizer is not None else SensorSynchronizer()
        self.aligner = aligner if aligner is not None else PhoneVehicleAligner()
        self.gnss_monitor = gnss_monitor if gnss_monitor is not None else GNSSQualityMonitor()
        self.confidence_manager = confidence_manager if confidence_manager is not None else ConfidenceManager()
        self.adapter = adapter if adapter is not None else SmartphoneSensorAdapter(100.0)
        
        self.window_size = window_size
        imu_rate = self.adapter.get_imu_sampling_rate()
        self._ai_stride = max(1, int(round(imu_rate / 100.0)))
        max_buf_len = window_size * self._ai_stride
        self._imu_window = deque(maxlen=max_buf_len)
        self._last_timestamp: typing.Optional[float] = None
        self._last_bias_residual: typing.Optional[np.ndarray] = None
        self._sample_counter = 0

    def initialize(self, initial_state: NavigationState) -> None:
        """Initialize the estimator with a defensible prior state."""
        self._current_state = initial_state
        self._last_timestamp = initial_state.timestamp
        self._last_bias_residual = None
        self._sample_counter = 0
        self._imu_window.clear()

    def set_map_route(self, route_enu: np.ndarray):
        """Sets the reference road graph for map-matching snaps."""
        self.map_matcher.set_reference_route(route_enu)

    def process_packet(self, raw_packet: typing.Any) -> NavigationState:
        """
        Main entry point for processing an incoming SensorPacket or raw payload.
        Supports high-rate EKF propagation (up to 200 Hz) with 10 Hz decimated AI & map matching.
        """
        if self._current_state is None:
            raise RuntimeError("Engine not initialized. Call initialize() first.")

        # Convert raw payload to SensorPacket via adapter if necessary
        if not isinstance(raw_packet, SensorPacket):
            packet = self.adapter.process_raw_sample(raw_packet)
        else:
            packet = raw_packet

        self._sample_counter += 1

        # Phone-to-Vehicle Alignment Transformation
        packet = self.aligner.process_packet(packet)

        # Compute delta time dt
        if self._last_timestamp is None:
            dt = 1.0 / self.adapter.get_imu_sampling_rate()
        else:
            dt = packet.timestamp - self._last_timestamp
            if dt <= 0:
                dt = 0.0001
        self._last_timestamp = packet.timestamp

        # 1. IMU Feature Buffer for AI temporal inference
        accel = packet.accelerometer if packet.accelerometer is not None else np.zeros(3)
        gyro = packet.gyroscope if packet.gyroscope is not None else np.zeros(3)
        imu_feat = np.concatenate([accel, gyro])
        self._imu_window.append(imu_feat)

        # 2. Run 5-Class AI Motion Classifier & Motion-Aware Fusion Integration
        q_scale = 1.0
        ai_speed = 0.0
        ai_unc = 2.0
        dominant_motion = MotionClass.NORMAL_DRIVING

        if len(self._imu_window) >= self.window_size:
            if self._ai_stride > 1:
                imu_win_arr = np.array(self._imu_window)[::self._ai_stride][:self.window_size]
            else:
                imu_win_arr = np.array(self._imu_window)

            if len(imu_win_arr) == self.window_size:
                ai_speed, ai_unc, probs, dominant_motion, conf, bias_res = self.ai_model.forward_full(imu_win_arr)

                self._last_bias_residual = bias_res
                self._current_state.metadata["motion_class"] = dominant_motion.value
                self._current_state.metadata["motion_confidence"] = conf
                self._current_state.metadata["bias_residual"] = bias_res.tolist()

                # Fusion Rule A: PHONE_MOVEMENT -> Invalidate alignment & trigger recalibration
                if dominant_motion == MotionClass.PHONE_MOVEMENT:
                    self.aligner.invalidate_alignment()

                # Fusion Rule B: BRAKING_ACCELERATION -> Adaptive process noise scaling
                elif dominant_motion == MotionClass.BRAKING_ACCELERATION:
                    q_scale = 2.5

                # Fusion Rule C: POTHOLE_BUMP -> Temporarily increase process uncertainty
                elif dominant_motion == MotionClass.POTHOLE_BUMP:
                    q_scale = 4.0

                # Fusion Rule D: IDLING_VIBRATION -> Reduce/clamp velocity confidence at zero speed
                elif dominant_motion == MotionClass.IDLING_VIBRATION:
                    ai_speed = 0.0
                    ai_unc = 0.05

        # 3. High-Rate EKF Time Update (Prediction) — runs on EVERY incoming packet (up to 200 Hz)
        self._current_state = self.ekf.predict(
            self._current_state, packet, dt, q_scale=q_scale, bias_residual=self._last_bias_residual
        )

        # 4. Evaluate Multi-Signal GNSS Quality State
        quality_res = self.gnss_monitor.evaluate_quality(packet, self._current_state)

        if quality_res.mode in [GNSSMode.HEALTHY, GNSSMode.DEGRADED, GNSSMode.RECOVERY] and quality_res.measurement_weight > 0.0 and getattr(quality_res, 'is_measurement_accepted', True):
            # Valid or degraded GNSS available: Update position & velocity with adaptive noise weighting
            gnss_enu = packet.metadata.get("gnss_enu") if packet.metadata else None
            if gnss_enu is None and packet.gnss_lat_lon_alt is not None:
                gnss_enu = packet.gnss_lat_lon_alt
                
            raw_accuracy = packet.gnss_accuracy if packet.gnss_accuracy is not None else 2.0
            # Scale R covariance by down-weighting: accuracy_eff = raw_accuracy / sqrt(weight)
            effective_accuracy = raw_accuracy / float(np.sqrt(max(1e-4, quality_res.measurement_weight)))
            
            self._current_state = self.ekf.update_gnss(self._current_state, gnss_enu, effective_accuracy)
            self._current_state.navigation_mode = quality_res.mode.value
            self._current_state.confidence = quality_res.confidence_score
        else:
            # GNSS Outage / Denied mode: Use AI model + NHC + Map Matching
            self._current_state.navigation_mode = "GNSS_DENIED"

            # 4a. AI Forward Velocity Update
            if len(self._imu_window) >= self.window_size:
                self._current_state = self.ekf.update_velocity(self._current_state, ai_speed, accuracy=ai_unc)

            # 4b & 4c: Run NHC & HMM Map Matching (decimated to 10 Hz for high-rate IMUs)
            map_stride = max(1, int(round(self.adapter.get_imu_sampling_rate() / 10.0)))
            if self._sample_counter % map_stride == 0:
                self._current_state = self.nhc.apply_constraint(self._current_state)
                self._current_state.metadata["map_matching_rate_hz"] = 10.0
                self._current_state = self.map_matcher.match(self._current_state, covariance=self._current_state.covariance)

        # 5. Evaluate System Confidence & Drift Manager
        align_state_str = str(self.aligner.state.value) if hasattr(self.aligner.state, "value") else str(self.aligner.state)
        gnss_mode_str = str(quality_res.mode.value) if hasattr(quality_res.mode, "value") else str(quality_res.mode)
        motion_str = str(dominant_motion.value) if hasattr(dominant_motion, "value") else str(dominant_motion)
        map_conf = self._current_state.metadata.get("map_confidence", None)

        nav_conf = self.confidence_manager.evaluate(
            covariance=self._current_state.covariance,
            ai_velocity_uncertainty=ai_unc,
            gnss_mode=gnss_mode_str,
            gnss_quality_score=quality_res.confidence_score,
            gnss_weight=quality_res.measurement_weight,
            motion_class=motion_str,
            map_confidence=map_conf,
            alignment_status=align_state_str,
            navigation_mode=self._current_state.navigation_mode
        )

        self._current_state.navigation_confidence = nav_conf
        self._current_state.confidence = nav_conf.overall_confidence
        self._current_state.metadata["adapter_type"] = self.adapter.__class__.__name__
        self._current_state.metadata["imu_sampling_rate_hz"] = self.adapter.get_imu_sampling_rate()

        return self._current_state

    def process_imu(self, packet: SensorPacket) -> NavigationState:
        """Legacy helper: Propagate state forward using IMU mechanics."""
        return self.process_packet(packet)

    def process_gnss(self, packet: SensorPacket) -> NavigationState:
        """Legacy helper: Apply GNSS measurement correction if available."""
        return self.process_packet(packet)

    def process_asynchronous_packets(self, raw_packets: typing.List[SensorPacket]) -> typing.List[NavigationState]:
        """
        Passes a stream of un-synchronized raw packets through the SensorSynchronizer,
        and feeds the resulting high-rate synchronized stream into the NavigationEngine loop.
        """
        synced = self.synchronizer.synchronize_stream(raw_packets)
        states = []
        for p in synced.high_rate_imu_stream:
            st = self.process_packet(p)
            states.append(st)
        return states

    def get_state(self) -> NavigationState:
        """Return the current fused navigation state."""
        return self._current_state




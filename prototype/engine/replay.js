// engine/replay.js — Deterministic Simulation Replay
// SIH 2026 NAVISENSE-IDR
// When real GPS is unavailable, replays a pre-computed trajectory
// with realistic speed, heading, and IMU data patterns.
// Label: [SIMULATION — Deterministic replay mode]

import { haversine, computeBearing, interpolate } from './gps.js';

export class ReplayEngine {
    constructor() {
        this.isRunning = false;
        this.rafId = null;
        this.routeCoords = [];
        this.currentIdx = 0;
        this.progressAlongSegment = 0;
        this.simulationSpeed = 1.0; // multiplier
        this.avgRouteSpeedKmh = 50;
        this.lastFrameTime = null;

        // Callbacks
        this.onPositionUpdate = null;
        this.onArrival = null;

        // GPS outage zone: fraction of route [start, end]
        this.outageZone = [0.35, 0.65];
        this.outageTriggered = false;
        this.outageEnded = false;
        this.onOutageStart = null;
        this.onOutageEnd = null;
    }

    setRoute(coords, avgSpeedKmh = 50, outageZone = [0.35, 0.65]) {
        this.routeCoords = coords;
        this.avgRouteSpeedKmh = avgSpeedKmh;
        this.outageZone = outageZone;
        this.currentIdx = 0;
        this.progressAlongSegment = 0;
        this.outageTriggered = false;
        this.outageEnded = false;
    }

    setSpeed(multiplier) {
        this.simulationSpeed = multiplier;
    }

    start() {
        this.isRunning = true;
        this.lastFrameTime = performance.now();
        this._tick();
    }

    stop() {
        this.isRunning = false;
        if (this.rafId) {
            cancelAnimationFrame(this.rafId);
            this.rafId = null;
        }
    }

    _tick(now) {
        if (!this.isRunning) return;

        const currentNow = now || performance.now();
        const dt = (currentNow - this.lastFrameTime) / 1000; // seconds
        this.lastFrameTime = currentNow;

        this._advance(dt);

        this.rafId = requestAnimationFrame((t) => this._tick(t));
    }

    _advance(dt) {
        const coords = this.routeCoords;
        if (!coords || coords.length < 2) return;

        // Speed in meters per second
        const speedMs = (this.avgRouteSpeedKmh / 3.6) * this.simulationSpeed;
        const distToAdvance = speedMs * dt; // meters this frame

        // Determine segment length
        const cur = coords[this.currentIdx];
        const next = coords[Math.min(this.currentIdx + 1, coords.length - 1)];
        const segLen = haversine(cur.lat, cur.lng, next.lat, next.lng);

        // Progress increment
        const progressIncrement = segLen > 0 ? distToAdvance / segLen : 0;
        this.progressAlongSegment += progressIncrement;

        // Advance to next segment(s)
        while (this.progressAlongSegment >= 1.0 && this.currentIdx < coords.length - 2) {
            this.progressAlongSegment -= 1.0;
            this.currentIdx++;
        }

        // Check arrival
        if (this.currentIdx >= coords.length - 2 && this.progressAlongSegment >= 0.99) {
            this.isRunning = false;
            if (this.onArrival) this.onArrival();
            return;
        }

        // Interpolate current position
        const p = Math.min(1, this.progressAlongSegment);
        const pos = interpolate(
            coords[this.currentIdx].lat, coords[this.currentIdx].lng,
            coords[Math.min(this.currentIdx + 1, coords.length - 1)].lat,
            coords[Math.min(this.currentIdx + 1, coords.length - 1)].lng,
            p
        );

        const heading = computeBearing(
            coords[this.currentIdx].lat, coords[this.currentIdx].lng,
            coords[Math.min(this.currentIdx + 1, coords.length - 1)].lat,
            coords[Math.min(this.currentIdx + 1, coords.length - 1)].lng
        );

        // Current fraction along total route
        const fraction = this.currentIdx / (coords.length - 1);

        // Check outage zone
        const inOutage = fraction >= this.outageZone[0] && fraction <= this.outageZone[1];

        if (inOutage && !this.outageTriggered) {
            this.outageTriggered = true;
            if (this.onOutageStart) this.onOutageStart(pos.lat, pos.lng, this.avgRouteSpeedKmh, heading);
        }

        if (!inOutage && this.outageTriggered && !this.outageEnded) {
            this.outageEnded = true;
            if (this.onOutageEnd) this.onOutageEnd(pos.lat, pos.lng, this.avgRouteSpeedKmh, heading);
        }

        // Simulate speed variation (±5 km/h oscillation)
        const speedVariation = Math.sin(this.currentIdx * 0.3) * 5;
        const currentSpeed = this.avgRouteSpeedKmh + speedVariation;

        // Simulate synthetic IMU
        const imu = this._synthesizeIMU(heading, currentSpeed, dt);

        // Emit GPS update (or no GPS if in outage)
        if (this.onPositionUpdate) {
            this.onPositionUpdate({
                lat: pos.lat,
                lng: pos.lng,
                speed: currentSpeed,
                heading,
                accuracy: 5 + Math.random() * 5,
                fraction,
                isGPSAvailable: !inOutage,
                imu,
                dt,
                source: 'SIMULATION'
            });
        }
    }

    _synthesizeIMU(heading, speed, dt) {
        const noise = () => (Math.random() - 0.5) * 0.15;
        const hdgRad = heading * Math.PI / 180;
        return {
            ax: Math.cos(hdgRad) * 0.05 + noise(),
            ay: Math.sin(hdgRad) * 0.03 + noise(),
            az: 9.81 + noise() * 0.02,
            gx: noise() * 0.3,
            gy: noise() * 0.3,
            gz: noise() * 0.5,
            isReal: false
        };
    }

    getFraction() {
        if (!this.routeCoords || this.routeCoords.length < 2) return 0;
        return this.currentIdx / (this.routeCoords.length - 1);
    }
}

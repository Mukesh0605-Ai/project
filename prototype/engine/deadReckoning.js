// engine/deadReckoning.js — EKF + AI Dead Reckoning Engine
// SIH 2026 NAVISENSE-IDR
// Combines AI model output with physics-based EKF for position estimation
// during GPS outages. Also handles map matching and smooth recovery.

import { AIModel } from './aiModel.js';
import { haversine, computeBearing } from './gps.js';

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;

export class DeadReckoningEngine {
    constructor() {
        this.ai = new AIModel();

        // EKF state: [lat, lng, speed (km/h), heading (deg)]
        this.state = null;
        this.isActive = false;

        // Drift accumulation
        this.blackoutSeconds = 0;
        this.blackoutInterval = null;
        this.driftMeters = 0;
        this.totalDR_distance = 0;

        // Map matching
        this.routeCoords = [];
        this.lastMatchedIdx = 0;

        // Recovery blend
        this.isRecovering = false;
        this.recoveryStart = null;
        this.recoveryDuration = 3000; // ms
        this.drPosAtRecovery = null;
        this.gpsPosAtRecovery = null;

        // Listeners
        this.onDRUpdate = null;
        this.onBlackoutTick = null;
    }

    // Initialize DR from last known GPS position
    startOutage(lat, lng, speed, heading) {
        this.state = { lat, lng, speed, heading };
        this.isActive = true;
        this.blackoutSeconds = 0;
        this.driftMeters = 0;
        this.totalDR_distance = 0;
        this.ai.reset();

        // Start blackout timer
        this.blackoutInterval = setInterval(() => {
            this.blackoutSeconds += 0.1;

            // Physical Kalman drift model: σ_pos = 0.5 * a_bias * t²
            const accelBias = 0.08; // m/s²
            this.driftMeters = 0.5 * accelBias * Math.pow(this.blackoutSeconds, 2);

            if (this.onBlackoutTick) {
                this.onBlackoutTick(this.blackoutSeconds, this.driftMeters);
            }
        }, 100);

        console.log('[DR] Outage started at', lat, lng);
    }

    // Stop DR and begin smooth recovery toward GPS fix
    startRecovery(gpsLat, gpsLng, gpsSpeed, gpsHeading) {
        if (!this.isActive) return;

        const finalDrift = haversine(this.state.lat, this.state.lng, gpsLat, gpsLng);

        this.isRecovering = true;
        this.recoveryStart = Date.now();
        this.drPosAtRecovery = { ...this.state };
        this.gpsPosAtRecovery = { lat: gpsLat, lng: gpsLng, speed: gpsSpeed, heading: gpsHeading };

        clearInterval(this.blackoutInterval);
        this.blackoutInterval = null;
        this.isActive = false;

        console.log('[DR] Recovery started, final drift:', finalDrift.toFixed(1), 'm');

        return {
            finalDrift,
            blackoutDuration: this.blackoutSeconds,
            totalDRdistance: this.totalDR_distance
        };
    }

    // Process one DR step using sensor data + AI model
    step(imuData, dt) {
        if (!this.isActive && !this.isRecovering) return null;

        // If in recovery, interpolate between DR and GPS position
        if (this.isRecovering) {
            return this._recoveryStep();
        }

        const { ax, ay, az, gx, gy, gz } = imuData;
        const { speed, heading } = this.state;

        // 1. AI model inference
        const ai = this.ai.infer(ax, ay, az, gx, gy, gz, dt, speed);

        // 2. Physics-based speed update
        //    v_new = v_old + (ax_forward * dt * 3.6) + AI_correction
        const axForward = ax * Math.cos(heading * DEG2RAD) + ay * Math.sin(heading * DEG2RAD);
        const physDeltaSpeed = axForward * dt * 3.6; // km/h
        let newSpeed = speed + physDeltaSpeed * 0.3 + ai.deltaSpeed * 0.7;
        newSpeed = Math.max(0, Math.min(200, newSpeed)); // clamp

        // 3. Heading update using gyroscope
        //    heading_new = heading + gz * dt + AI_correction
        const physDeltaHeading = gz * dt * RAD2DEG * 0.5; // deg
        let newHeading = (heading + physDeltaHeading * 0.4 + ai.deltaHeading * 0.6 + 360) % 360;

        // 4. Position integration (dead reckoning)
        //    Using average speed in km/h and dt in seconds
        const avgSpeed = (speed + newSpeed) / 2;
        const distMeters = (avgSpeed / 3.6) * dt;
        this.totalDR_distance += distMeters;

        const headingRad = newHeading * DEG2RAD;
        const dlat = (distMeters * Math.cos(headingRad)) / 111320;
        const dlng = (distMeters * Math.sin(headingRad)) / (111320 * Math.cos(this.state.lat * DEG2RAD));

        let newLat = this.state.lat + dlat;
        let newLng = this.state.lng + dlng;

        // 5. Map matching — snap to nearest route point
        const matched = this._mapMatch(newLat, newLng, newHeading);
        if (matched) {
            newLat = matched.lat * 0.85 + newLat * 0.15; // Soft snap
            newLng = matched.lng * 0.85 + newLng * 0.15;
            newHeading = matched.heading * 0.5 + newHeading * 0.5;
        }

        this.state = { lat: newLat, lng: newLng, speed: newSpeed, heading: newHeading };

        const result = {
            lat: newLat,
            lng: newLng,
            speed: newSpeed,
            heading: newHeading,
            confidence: ai.confidence * Math.max(0.3, 1 - this.blackoutSeconds / 120),
            drift: this.driftMeters,
            blackoutSeconds: this.blackoutSeconds,
            source: 'DR',
            aiConfidence: ai.confidence
        };

        if (this.onDRUpdate) this.onDRUpdate(result);
        return result;
    }

    _recoveryStep() {
        const elapsed = Date.now() - this.recoveryStart;
        const t = Math.min(1, elapsed / this.recoveryDuration);

        // Ease-in-out interpolation
        const eased = t < 0.5 ? 2*t*t : -1+(4-2*t)*t;

        const lat = this.drPosAtRecovery.lat + (this.gpsPosAtRecovery.lat - this.drPosAtRecovery.lat) * eased;
        const lng = this.drPosAtRecovery.lng + (this.gpsPosAtRecovery.lng - this.drPosAtRecovery.lng) * eased;
        const spd = this.drPosAtRecovery.speed + (this.gpsPosAtRecovery.speed - this.drPosAtRecovery.speed) * eased;

        if (t >= 1) {
            this.isRecovering = false;
            this.state = { ...this.gpsPosAtRecovery };
        }

        return {
            lat, lng,
            speed: spd,
            heading: this.gpsPosAtRecovery.heading,
            confidence: eased * 0.6 + 0.4, // Growing confidence
            source: 'RECOVERY',
            recoveryProgress: eased
        };
    }

    // Map matching: find nearest point on route within 50m
    _mapMatch(lat, lng, heading) {
        if (!this.routeCoords || this.routeCoords.length < 2) return null;

        let minDist = 50; // meters — max snap distance
        let bestIdx = -1;

        // Search from last matched position (optimization)
        const start = Math.max(0, this.lastMatchedIdx - 5);
        const end = Math.min(this.routeCoords.length - 1, this.lastMatchedIdx + 30);

        for (let i = start; i <= end; i++) {
            const d = haversine(lat, lng, this.routeCoords[i].lat, this.routeCoords[i].lng);
            if (d < minDist) {
                minDist = d;
                bestIdx = i;
            }
        }

        if (bestIdx < 0) return null;

        this.lastMatchedIdx = bestIdx;
        const pt = this.routeCoords[bestIdx];
        const nextPt = this.routeCoords[Math.min(bestIdx + 1, this.routeCoords.length - 1)];
        const roadHeading = computeBearing(pt.lat, pt.lng, nextPt.lat, nextPt.lng);

        return { lat: pt.lat, lng: pt.lng, heading: roadHeading };
    }

    setRoute(coords) {
        this.routeCoords = coords;
        this.lastMatchedIdx = 0;
    }

    stop() {
        this.isActive = false;
        this.isRecovering = false;
        clearInterval(this.blackoutInterval);
        this.blackoutInterval = null;
    }

    getMetrics() {
        return {
            blackoutDuration: this.blackoutSeconds,
            drift: this.driftMeters,
            totalDRdistance: this.totalDR_distance,
            aiLabel: this.ai.label
        };
    }
}

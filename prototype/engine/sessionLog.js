// engine/sessionLog.js — Records all telemetry for final results
// SIH 2026 NAVISENSE-IDR

import { haversine } from './gps.js';

export class SessionLog {
    constructor() {
        this.reset();
    }

    reset() {
        this.startTime = null;
        this.endTime = null;
        this.gpsPoints = [];      // {lat, lng, speed, ts}
        this.drPoints = [];       // {lat, lng, speed, confidence, ts}
        this.outageEvents = [];   // {startTs, endTs, driftM}
        this.currentOutage = null;
        this.totalDistanceM = 0;
        this.lastGpsPoint = null;
        this.maxDrift = 0;
        this.speedErrors = [];
        this.headingErrors = [];
        this.recoveryTimes = [];
    }

    startSession() {
        this.reset();
        this.startTime = Date.now();
    }

    logGPS(lat, lng, speed, heading, accuracy) {
        const point = { lat, lng, speed, heading, accuracy, ts: Date.now(), source: 'GPS' };
        this.gpsPoints.push(point);

        if (this.lastGpsPoint) {
            const d = haversine(this.lastGpsPoint.lat, this.lastGpsPoint.lng, lat, lng);
            this.totalDistanceM += d;
        }
        this.lastGpsPoint = point;
    }

    logDR(lat, lng, speed, heading, confidence, drift, blackoutSecs) {
        const point = { lat, lng, speed, heading, confidence, drift, blackoutSecs, ts: Date.now(), source: 'DR' };
        this.drPoints.push(point);
        if (drift > this.maxDrift) this.maxDrift = drift;

        // Compare DR speed to last GPS speed
        if (this.lastGpsPoint) {
            const speedErr = Math.abs(speed - this.lastGpsPoint.speed);
            this.speedErrors.push(speedErr);

            const hdgErr = Math.abs(heading - (this.lastGpsPoint.heading || 0));
            this.headingErrors.push(Math.min(hdgErr, 360 - hdgErr));
        }
    }

    startOutage() {
        this.currentOutage = {
            startTs: Date.now(),
            startPos: this.lastGpsPoint ? { ...this.lastGpsPoint } : null
        };
    }

    endOutage(finalDrift, recoveryTimeMs) {
        if (!this.currentOutage) return;
        this.outageEvents.push({
            startTs: this.currentOutage.startTs,
            endTs: Date.now(),
            duration: (Date.now() - this.currentOutage.startTs) / 1000,
            driftM: finalDrift
        });
        if (recoveryTimeMs) this.recoveryTimes.push(recoveryTimeMs);
        this.currentOutage = null;
    }

    endSession() {
        this.endTime = Date.now();
    }

    // Compute all final metrics
    getResults() {
        const totalDuration = this.endTime
            ? (this.endTime - this.startTime) / 1000
            : (Date.now() - this.startTime) / 1000;

        const totalBlackoutSecs = this.outageEvents.reduce((s, e) => s + e.duration, 0);
        const totalBlackoutDuration = totalBlackoutSecs.toFixed(1);

        const finalPositionError = this.outageEvents.length > 0
            ? this.outageEvents[this.outageEvents.length - 1].driftM
            : 0;

        const avgSpeedError = this.speedErrors.length > 0
            ? (this.speedErrors.reduce((a, b) => a + b, 0) / this.speedErrors.length).toFixed(2)
            : '0.00';

        const avgHeadingError = this.headingErrors.length > 0
            ? (this.headingErrors.reduce((a, b) => a + b, 0) / this.headingErrors.length).toFixed(2)
            : '0.00';

        const avgRecoveryTime = this.recoveryTimes.length > 0
            ? (this.recoveryTimes.reduce((a, b) => a + b, 0) / this.recoveryTimes.length / 1000).toFixed(2)
            : '3.00';

        // Overall confidence score (0-100)
        const drConfidences = this.drPoints.map(p => p.confidence);
        const avgConf = drConfidences.length > 0
            ? drConfidences.reduce((a, b) => a + b, 0) / drConfidences.length
            : 0;

        // Scoring formula
        const driftScore = Math.max(0, 100 - this.maxDrift * 2);
        const speedScore = Math.max(0, 100 - parseFloat(avgSpeedError) * 5);
        const headingScore = Math.max(0, 100 - parseFloat(avgHeadingError) * 2);
        const overallScore = ((driftScore + speedScore + headingScore) / 3).toFixed(1);

        return {
            totalDuration: totalDuration.toFixed(0),
            travelledDistanceM: this.totalDistanceM.toFixed(0),
            travelledDistanceKm: (this.totalDistanceM / 1000).toFixed(2),
            gpsBlackoutDuration: totalBlackoutDuration,
            positionErrorM: finalPositionError.toFixed(2),
            maxDriftM: this.maxDrift.toFixed(2),
            speedErrorKmh: avgSpeedError,
            headingErrorDeg: avgHeadingError,
            recoveryTimeSecs: avgRecoveryTime,
            outageCount: this.outageEvents.length,
            drPointsLogged: this.drPoints.length,
            overallConfidence: (avgConf * 100).toFixed(1),
            overallScore,
            gpsPointsLogged: this.gpsPoints.length,
            drPointsLogged: this.drPoints.length
        };
    }
}

// engine/gps.js — Authoritative Real GPS Engine with Standardized State Machine
// SIH 2026 NAVISENSE-IDR

export class GPSEngine {
    constructor() {
        this.watchId = null;
        this.lastPos = null;
        this.lastHeading = 0;
        this.speedBuffer = [];
        this.isLive = false;
        this.accuracy = null;
        this.lastFixTime = 0;
        this.watchdogTimer = null;
        
        // Standardized state machine:
        // GPS_WAITING | GPS_GOOD | GPS_WEAK | GPS_LOST | GPS_RECOVERING
        this.status = 'GPS_WAITING';
        
        this.onUpdate = null;
        this.onStatusChange = null;
        this.onError = null;
    }

    start(onUpdate, onStatusChange, onError) {
        this.onUpdate = onUpdate;
        this.onStatusChange = onStatusChange;
        this.onError = onError;

        if (!navigator.geolocation) {
            this._handleError(2, 'Geolocation is not supported by this browser.');
            this._setStatus('GPS_LOST');
            return;
        }

        this._setStatus('GPS_WAITING');

        // Start loss watchdog: checks every 2s if signal was lost for > 6s
        this._startWatchdog();

        // 1. Initial fix request with high accuracy
        navigator.geolocation.getCurrentPosition(
            (pos) => {
                this.isLive = true;
                this._processPosition(pos);
                this._startWatch();
            },
            (err) => {
                console.warn('[GPS] Initial getCurrentPosition error:', err.code, err.message);
                this._handleGeolocationError(err);
                // Even if getCurrentPosition times out, watchPosition may still lock when satellite fix arrives
                this._startWatch();
            },
            { enableHighAccuracy: true, timeout: 15000, maximumAge: 2000 }
        );
    }

    _startWatch() {
        if (this.watchId !== null) return;

        this.watchId = navigator.geolocation.watchPosition(
            (pos) => {
                this._processPosition(pos);
            },
            (err) => {
                console.warn('[GPS] watchPosition error:', err.code, err.message);
                this._handleGeolocationError(err);
            },
            { enableHighAccuracy: true, timeout: 15000, maximumAge: 2000 }
        );
    }

    _handleGeolocationError(err) {
        let msg = 'Unknown GPS error occurred.';
        switch (err.code) {
            case 1: // PERMISSION_DENIED
                msg = 'Location permission denied. Enable location permission and try again.';
                this._setStatus('GPS_LOST');
                break;
            case 2: // POSITION_UNAVAILABLE
                msg = 'GPS signal unavailable. Move to an area with better GPS reception.';
                this._setStatus('GPS_LOST');
                break;
            case 3: // TIMEOUT
                msg = 'GPS request timed out. Retrying...';
                if (!this.isLive) {
                    this._setStatus('GPS_WAITING');
                } else {
                    this._setStatus('GPS_LOST');
                }
                break;
        }
        this._handleError(err.code, msg);
    }

    _handleError(code, message) {
        if (this.onError) {
            this.onError(code, message);
        }
    }

    _processPosition(pos) {
        const { latitude: lat, longitude: lng, accuracy, heading, speed } = pos.coords;
        const ts = pos.timestamp || Date.now();
        this.lastFixTime = Date.now();

        // Calculate speed (km/h)
        let rawSpeed = null;
        if (speed !== null && !isNaN(speed) && speed >= 0) {
            rawSpeed = speed * 3.6; // Convert m/s -> km/h
        } else if (this.lastPos && this.lastPos.ts) {
            const dt = (ts - this.lastPos.ts) / 1000;
            if (dt > 0.2 && dt < 15) {
                const dist = haversine(this.lastPos.lat, this.lastPos.lng, lat, lng);
                rawSpeed = (dist / dt) * 3.6;
            }
        }
        if (rawSpeed === null || isNaN(rawSpeed) || rawSpeed < 0) rawSpeed = 0;

        // Stationary threshold: movement < 1.8 km/h is treated as 0
        const isStationary = rawSpeed < 1.8;
        if (isStationary) {
            rawSpeed = 0;
        }

        // Rolling 4-sample average to suppress GPS noise
        this.speedBuffer.push(rawSpeed);
        if (this.speedBuffer.length > 4) this.speedBuffer.shift();
        const smoothSpeed = isStationary
            ? 0
            : (this.speedBuffer.reduce((a, b) => a + b, 0) / this.speedBuffer.length);

        // Heading calculation
        let validHeading = null;
        if (heading !== null && heading !== undefined && !isNaN(heading) && heading >= 0) {
            validHeading = heading;
            this.lastHeading = validHeading;
        } else if (this.lastPos && !isStationary && smoothSpeed > 2.5) {
            const movedDist = haversine(this.lastPos.lat, this.lastPos.lng, lat, lng);
            if (movedDist >= 3.0) {
                validHeading = computeBearing(this.lastPos.lat, this.lastPos.lng, lat, lng);
                this.lastHeading = validHeading;
            }
        }

        if (validHeading === null) {
            validHeading = this.lastHeading || 0;
        }

        // State Machine Transition
        const wasLost = (this.status === 'GPS_LOST');
        let nextStatus;

        if (wasLost) {
            nextStatus = 'GPS_RECOVERING';
            this._setStatus(nextStatus);
            // Quick recovery transition to good/weak
            setTimeout(() => {
                const resolvedStatus = (accuracy <= 25) ? 'GPS_GOOD' : 'GPS_WEAK';
                this._setStatus(resolvedStatus);
            }, 500);
        } else {
            nextStatus = (accuracy <= 25) ? 'GPS_GOOD' : 'GPS_WEAK';
            this._setStatus(nextStatus);
        }

        this.lastPos = { lat, lng, ts };
        this.accuracy = accuracy;
        this.isLive = true;

        const update = {
            lat,
            lng,
            accuracy,
            speed: smoothSpeed,
            rawSpeed,
            bearing: validHeading,
            isStationary,
            ts,
            source: 'GPS',
            status: this.status
        };

        if (this.onUpdate) {
            this.onUpdate(update);
        }
    }

    _startWatchdog() {
        if (this.watchdogTimer) clearInterval(this.watchdogTimer);
        this.watchdogTimer = setInterval(() => {
            if (this.isLive && this.lastFixTime > 0) {
                const silenceDuration = Date.now() - this.lastFixTime;
                if (silenceDuration > 6000 && this.status !== 'GPS_LOST') {
                    console.warn(`[GPS Watchdog] No fix received for ${(silenceDuration / 1000).toFixed(1)}s. Marking GPS_LOST.`);
                    this._setStatus('GPS_LOST');
                }
            }
        }, 2000);
    }

    stop() {
        if (this.watchId !== null) {
            navigator.geolocation.clearWatch(this.watchId);
            this.watchId = null;
        }
        if (this.watchdogTimer) {
            clearInterval(this.watchdogTimer);
            this.watchdogTimer = null;
        }
        this.isLive = false;
        this._setStatus('GPS_WAITING');
    }

    _setStatus(status) {
        if (this.status === status) return;
        this.status = status;
        if (this.onStatusChange) {
            this.onStatusChange(status);
        }
    }
}

// Utility: Haversine distance in meters
export function haversine(lat1, lon1, lat2, lon2) {
    const R = 6371000;
    const p1 = lat1 * Math.PI / 180;
    const p2 = lat2 * Math.PI / 180;
    const dp = (lat2 - lat1) * Math.PI / 180;
    const dl = (lon2 - lon1) * Math.PI / 180;
    const a = Math.sin(dp / 2) ** 2 + Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

// Utility: Compass bearing in degrees (0-360)
export function computeBearing(lat1, lon1, lat2, lon2) {
    const p1 = lat1 * Math.PI / 180;
    const p2 = lat2 * Math.PI / 180;
    const dl = (lon2 - lon1) * Math.PI / 180;
    const y = Math.sin(dl) * Math.cos(p2);
    const x = Math.cos(p1) * Math.sin(p2) - Math.sin(p1) * Math.cos(p2) * Math.cos(dl);
    return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
}

// Utility: Interpolate between two lat/lng points
export function interpolate(lat1, lng1, lat2, lng2, t) {
    return {
        lat: lat1 + (lat2 - lat1) * t,
        lng: lng1 + (lng2 - lng1) * t
    };
}

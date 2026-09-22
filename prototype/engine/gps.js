// engine/gps.js — Real GPS Engine with Live Tracking + Simulation Fallback
// SIH 2026 NAVISENSE-IDR

export class GPSEngine {
    constructor() {
        this.watchId = null;
        this.lastPos = null;
        this.listeners = [];
        this.speedBuffer = [];
        this.headingBuffer = [];
        this.isLive = false;
        this.simulatedPos = null;
        this.accuracy = 5;
        this.status = 'IDLE'; // IDLE | ACQUIRING | LOCKED | LOST | SIMULATED
    }

    start(onUpdate, onStatusChange) {
        this.onUpdate = onUpdate;
        this.onStatusChange = onStatusChange;

        if (!navigator.geolocation) {
            this._setStatus('SIMULATED');
            return;
        }

        this._setStatus('ACQUIRING');

        // Try to get current position first (fast initial fix)
        navigator.geolocation.getCurrentPosition(
            (pos) => {
                this.isLive = true;
                this._processPosition(pos);
                this._setStatus('LOCKED');
                // Then start watching
                this._startWatch();
            },
            (err) => {
                console.warn('[GPS] Initial fix failed, starting watch anyway:', err.message);
                this._startWatch();
            },
            { enableHighAccuracy: true, timeout: 10000, maximumAge: 5000 }
        );
    }

    _startWatch() {
        this.watchId = navigator.geolocation.watchPosition(
            (pos) => {
                this.isLive = true;
                this._setStatus('LOCKED');
                this._processPosition(pos);
            },
            (err) => {
                console.warn('[GPS] Watch error:', err.message);
                if (err.code === 1) {
                    // Permission denied
                    this._setStatus('SIMULATED');
                } else {
                    this._setStatus('LOST');
                }
            },
            { enableHighAccuracy: true, maximumAge: 0, timeout: 15000 }
        );
    }

    _processPosition(pos) {
        const { latitude: lat, longitude: lng, accuracy, heading, speed } = pos.coords;
        const ts = pos.timestamp;

        // Calculate speed manually if device doesn't report
        let rawSpeed = speed !== null ? speed * 3.6 : null; // km/h
        if (rawSpeed === null && this.lastPos) {
            const dist = haversine(this.lastPos.lat, this.lastPos.lng, lat, lng);
            const dt = (ts - this.lastPos.ts) / 1000;
            rawSpeed = dt > 0 ? (dist / dt) * 3.6 : 0;
        }
        if (rawSpeed === null) rawSpeed = 0;

        // Stationary detection: < 2 km/h = stop
        const isStationary = rawSpeed < 2.0;
        if (isStationary) rawSpeed = 0;

        // Rolling speed smoothing (5-sample)
        this.speedBuffer.push(rawSpeed);
        if (this.speedBuffer.length > 5) this.speedBuffer.shift();
        const smoothSpeed = this.speedBuffer.reduce((a, b) => a + b, 0) / this.speedBuffer.length;

        // Bearing from heading or computed
        let bearing = heading;
        if ((bearing === null || bearing === undefined) && this.lastPos) {
            bearing = computeBearing(this.lastPos.lat, this.lastPos.lng, lat, lng);
        }
        if (bearing === null || bearing === undefined) bearing = 0;

        this.lastPos = { lat, lng, ts };
        this.accuracy = accuracy;

        const update = {
            lat, lng,
            accuracy,
            speed: smoothSpeed,
            rawSpeed,
            bearing,
            isStationary,
            ts,
            source: 'GPS'
        };

        if (this.onUpdate) this.onUpdate(update);
    }

    // Simulate position along a route for GPS replay
    setSimulatedPosition(lat, lng, speed, bearing, accuracy = 5) {
        const update = {
            lat, lng,
            accuracy,
            speed,
            rawSpeed: speed,
            bearing,
            isStationary: speed < 2,
            ts: Date.now(),
            source: 'GPS'
        };
        if (this.onUpdate) this.onUpdate(update);
    }

    stop() {
        if (this.watchId !== null) {
            navigator.geolocation.clearWatch(this.watchId);
            this.watchId = null;
        }
        this.isLive = false;
    }

    _setStatus(status) {
        this.status = status;
        if (this.onStatusChange) this.onStatusChange(status);
    }
}

// Utility: Haversine distance in meters
export function haversine(lat1, lon1, lat2, lon2) {
    const R = 6371000;
    const p1 = lat1 * Math.PI / 180;
    const p2 = lat2 * Math.PI / 180;
    const dp = (lat2 - lat1) * Math.PI / 180;
    const dl = (lon2 - lon1) * Math.PI / 180;
    const a = Math.sin(dp/2)**2 + Math.cos(p1)*Math.cos(p2)*Math.sin(dl/2)**2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
}

// Utility: Compass bearing in degrees
export function computeBearing(lat1, lon1, lat2, lon2) {
    const p1 = lat1 * Math.PI / 180;
    const p2 = lat2 * Math.PI / 180;
    const dl = (lon2 - lon1) * Math.PI / 180;
    const y = Math.sin(dl) * Math.cos(p2);
    const x = Math.cos(p1)*Math.sin(p2) - Math.sin(p1)*Math.cos(p2)*Math.cos(dl);
    return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
}

// Utility: Interpolate between two lat/lng points
export function interpolate(lat1, lng1, lat2, lng2, t) {
    return {
        lat: lat1 + (lat2 - lat1) * t,
        lng: lng1 + (lng2 - lng1) * t
    };
}

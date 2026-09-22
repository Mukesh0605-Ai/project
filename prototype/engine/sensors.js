// engine/sensors.js — Device Motion & Orientation Sensors
// SIH 2026 NAVISENSE-IDR
// Uses DeviceMotion API (accelerometer + gyroscope) when available,
// falls back to synthetic IMU simulation based on vehicle motion

export class SensorEngine {
    constructor() {
        this.isReal = false;
        this.listeners = [];
        this.data = {
            ax: 0, ay: 0, az: 9.81,   // Accelerometer (m/s²)
            gx: 0, gy: 0, gz: 0,       // Gyroscope (deg/s)
            heading: 0,                 // Compass heading (deg)
            pitch: 0,
            roll: 0
        };
        this.motionInterval = null;
        this.prevVehicleSpeed = 0;
        this.prevVehicleHeading = 0;
        this.noiseSeeds = { ax: Math.random(), ay: Math.random(), az: Math.random() };
    }

    start(onData) {
        this.onData = onData;

        // Try DeviceMotionEvent (requires HTTPS or localhost)
        if (typeof DeviceMotionEvent !== 'undefined') {
            // iOS 13+ requires permission
            if (typeof DeviceMotionEvent.requestPermission === 'function') {
                DeviceMotionEvent.requestPermission()
                    .then(perm => {
                        if (perm === 'granted') {
                            this._attachMotionListeners();
                        } else {
                            this._startSynthetic();
                        }
                    })
                    .catch(() => this._startSynthetic());
            } else {
                this._attachMotionListeners();
            }
        } else {
            this._startSynthetic();
        }

        // DeviceOrientation for compass heading
        if (typeof DeviceOrientationEvent !== 'undefined') {
            window.addEventListener('deviceorientation', (e) => {
                if (e.alpha !== null) {
                    this.data.heading = e.alpha;
                    this.data.pitch = e.beta || 0;
                    this.data.roll = e.gamma || 0;
                }
            }, { passive: true });
        }
    }

    _attachMotionListeners() {
        window.addEventListener('devicemotion', (e) => {
            const acc = e.accelerationIncludingGravity;
            const rot = e.rotationRate;

            if (acc && acc.x !== null) {
                // Desktop browsers sometimes fire events with exactly 0 for all axes
                if (acc.x === 0 && acc.y === 0 && acc.z === 0) {
                    return; // Ignore fake desktop event
                }
                this.data.ax = acc.x;
                this.data.ay = acc.y;
                this.data.az = acc.z;
                this.isReal = true;
            }
            if (rot) {
                this.data.gx = rot.alpha || 0;
                this.data.gy = rot.beta || 0;
                this.data.gz = rot.gamma || 0;
            }

            if (this.onData) this.onData({ ...this.data, isReal: true });
        }, { passive: true });

        // Check if we got real data after 2 seconds
        setTimeout(() => {
            if (!this.isReal) {
                this._startSynthetic();
            }
        }, 2000);
    }

    _startSynthetic() {
        // Simulate realistic IMU data based on vehicle motion
        this.isReal = false;
        this.motionInterval = setInterval(() => {
            this._updateSynthetic();
            if (this.onData) this.onData({ ...this.data, isReal: false });
        }, 50); // 20 Hz IMU simulation
    }

    // Called from outside with current vehicle state to drive synthetic IMU
    updateVehicleState(speed, heading, isOutage, isAccelerating = false) {
        this._vehicleSpeed = speed;
        this._vehicleHeading = heading;
        this._isOutage = isOutage;
        this._isAccelerating = isAccelerating;
    }

    _updateSynthetic() {
        const spd = this._vehicleSpeed || 0;
        const hdg = this._vehicleHeading || 0;
        const isOutage = this._isOutage || false;

        // Forward acceleration (along vehicle heading)
        const dv = (spd - this.prevVehicleSpeed) / 0.05; // m/s² approximation
        this.prevVehicleSpeed = spd;

        // Heading change rate (for gyro)
        const dHeading = hdg - this.prevVehicleHeading;
        const dHdgNorm = ((dHeading + 540) % 360) - 180; // -180 to 180
        this.prevVehicleHeading = hdg;

        // White noise (realistic IMU noise model)
        const noise = () => (Math.random() - 0.5) * 0.15;

        // Convert vehicle-frame to body-frame (simplified: assume flat road)
        const hdgRad = hdg * Math.PI / 180;
        const axForward = Math.cos(hdgRad) * (dv * 0.1) + noise();
        const ayLateral = Math.sin(hdgRad) * (dv * 0.05) + noise();

        this.data.ax = axForward;
        this.data.ay = ayLateral;
        this.data.az = 9.81 + noise() * 0.05; // Gravity + vibration
        this.data.gx = noise() * 0.5;
        this.data.gy = noise() * 0.5;
        this.data.gz = (dHdgNorm / 0.05) * 0.1 + noise() * 0.2; // Yaw rate

        // Add outage noise (no GPS correction → higher drift)
        if (isOutage) {
            this.data.ax += noise() * 0.2;
            this.data.ay += noise() * 0.2;
            this.data.gz += noise() * 0.5;
        }
    }

    stop() {
        if (this.motionInterval) {
            clearInterval(this.motionInterval);
            this.motionInterval = null;
        }
    }

    get() {
        return { ...this.data, isReal: this.isReal };
    }
}

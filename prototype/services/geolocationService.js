export const GeolocationService = {
    watchId: null,
    lastPosition: null,
    speedBuffer: [],
    
    startLiveTracking: (onPositionUpdate, onError) => {
        if (!navigator.geolocation) {
            onError(new Error('Geolocation is not supported by your browser.'));
            return;
        }
        
        GeolocationService.watchId = navigator.geolocation.watchPosition(
            (pos) => {
                const lat = pos.coords.latitude;
                const lng = pos.coords.longitude;
                const accuracy = pos.coords.accuracy;
                let rawSpeedMps = pos.coords.speed; // meters per second
                const timestamp = pos.timestamp;

                // Calculate speed manually if the device does not report it
                if (rawSpeedMps === null && GeolocationService.lastPosition) {
                    const dist = GeolocationService.calculateHaversine(
                        GeolocationService.lastPosition.lat, GeolocationService.lastPosition.lng,
                        lat, lng
                    );
                    const timeDiffSec = (timestamp - GeolocationService.lastPosition.timestamp) / 1000;
                    if (timeDiffSec > 0) {
                        rawSpeedMps = dist / timeDiffSec;
                    } else {
                        rawSpeedMps = 0;
                    }
                }
                
                if (rawSpeedMps === null || isNaN(rawSpeedMps)) {
                    rawSpeedMps = 0;
                }

                // Stationary Detection (speed < 1.0 m/s ~ 3.6 km/h)
                const isStationary = rawSpeedMps < 1.0;
                if (isStationary) {
                    rawSpeedMps = 0;
                }

                // Smooth speed (rolling average)
                GeolocationService.speedBuffer.push(rawSpeedMps);
                if (GeolocationService.speedBuffer.length > 5) {
                    GeolocationService.speedBuffer.shift();
                }
                
                const avgSpeedMps = GeolocationService.speedBuffer.reduce((a, b) => a + b, 0) / GeolocationService.speedBuffer.length;
                const filteredSpeedKmh = avgSpeedMps * 3.6;
                const rawSpeedKmh = rawSpeedMps * 3.6;

                GeolocationService.lastPosition = { lat, lng, timestamp };

                onPositionUpdate({
                    lat, lng, accuracy, rawSpeedKmh, filteredSpeedKmh, isStationary, timestamp
                });
            },
            (error) => {
                onError(error);
            },
            {
                enableHighAccuracy: true,
                maximumAge: 0,
                timeout: 10000
            }
        );
    },

    stopLiveTracking: () => {
        if (GeolocationService.watchId !== null) {
            navigator.geolocation.clearWatch(GeolocationService.watchId);
            GeolocationService.watchId = null;
        }
    },

    // Helper: Haversine distance in meters
    calculateHaversine: (lat1, lon1, lat2, lon2) => {
        const R = 6371e3;
        const φ1 = lat1 * Math.PI / 180;
        const φ2 = lat2 * Math.PI / 180;
        const Δφ = (lat2 - lat1) * Math.PI / 180;
        const Δλ = (lon2 - lon1) * Math.PI / 180;
        const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ/2) * Math.sin(Δλ/2);
        return R * (2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a)));
    },
    
    // Helper: Bearing
    calculateBearingDeg: (lat1, lon1, lat2, lon2) => {
        const φ1 = lat1 * Math.PI / 180;
        const φ2 = lat2 * Math.PI / 180;
        const Δλ = (lon2 - lon1) * Math.PI / 180;
        const y = Math.sin(Δλ) * Math.cos(φ2);
        const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
        return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
    }
};

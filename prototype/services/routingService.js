export const RoutingService = {
    fetchRouteOSRM: async (origin, dest) => {
        const url = \`https://router.project-osrm.org/route/v1/driving/\${origin.lng},\${origin.lat};\${dest.lng},\${dest.lat}?overview=full&geometries=geojson&steps=true\`;
        try {
            const res = await fetch(url);
            if (res.ok) {
                const data = await res.json();
                if (data.routes && data.routes.length > 0) {
                    const r = data.routes[0];
                    const coords = r.geometry.coordinates.map(c => ({ lat: c[1], lng: c[0] }));
                    const steps = r.legs[0].steps.map(s => ({
                        instruction: s.maneuver.modifier ? \`Turn \${s.maneuver.modifier}\` : 'Continue',
                        distMeters: s.distance,
                        distance: s.distance > 1000 ? (s.distance / 1000).toFixed(1) + ' km' : Math.round(s.distance) + ' m'
                    }));
                    return { coords, steps, distanceMeters: r.distance, durationSeconds: r.duration };
                }
            }
        } catch (e) {
            console.warn("OSRM routing failed, falling back", e);
        }
        return null;
    },

    fetchRouteGoogle: (origin, dest) => {
        return new Promise((resolve) => {
            if (!window.google || !google.maps.DirectionsService) {
                resolve(null);
                return;
            }
            const directionsService = new google.maps.DirectionsService();
            directionsService.route({
                origin: new google.maps.LatLng(origin.lat, origin.lng),
                destination: new google.maps.LatLng(dest.lat, dest.lng),
                travelMode: 'DRIVING'
            }, (response, status) => {
                if (status === 'OK' && response.routes.length > 0) {
                    const r = response.routes[0];
                    const leg = r.legs[0];
                    // Extract detailed path
                    const coords = r.overview_path.map(p => ({ lat: p.lat(), lng: p.lng() }));
                    const steps = leg.steps.map(s => ({
                        instruction: s.instructions.replace(/<[^>]*>?/gm, ''), // Strip HTML
                        distMeters: s.distance.value,
                        distance: s.distance.text
                    }));
                    resolve({ coords, steps, distanceMeters: leg.distance.value, durationSeconds: leg.duration.value });
                } else {
                    resolve(null);
                }
            });
        });
    },

    getRoute: async (origin, dest) => {
        // Prefer Google if available, else fallback to OSRM
        if (window.google) {
            const res = await RoutingService.fetchRouteGoogle(origin, dest);
            if (res) return res;
        }
        return await RoutingService.fetchRouteOSRM(origin, dest);
    }
};

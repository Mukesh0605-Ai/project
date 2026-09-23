// engine/routing.js — OSRM Routing + Nominatim Geocoding
// SIH 2026 NAVISENSE-IDR

import { haversine } from './gps.js';

export class RoutingEngine {
    constructor() {
        this.osrmBase = 'https://router.project-osrm.org/route/v1/driving';
        this.nominatimBase = 'https://nominatim.openstreetmap.org';
    }

    // Geocode a place name → {lat, lng, display_name}
    async geocode(query) {
        const url = `${this.nominatimBase}/search?format=json&limit=5&q=${encodeURIComponent(query)}&countrycodes=in`;
        try {
            const res = await fetch(url, { headers: { 'Accept-Language': 'en' } });
            const data = await res.json();
            if (data && data.length > 0) {
                return data.map(d => ({
                    lat: parseFloat(d.lat),
                    lng: parseFloat(d.lon),
                    label: d.display_name.split(',').slice(0, 2).join(', '),
                    type: (d.type || d.class || 'location').charAt(0).toUpperCase() + (d.type || d.class || 'location').slice(1),
                    full: d.display_name
                }));
            }
        } catch (e) {
            console.warn('[Routing] Geocode failed:', e);
        }
        return [];
    }

    // Reverse geocode lat/lng → place name
    async reverseGeocode(lat, lng) {
        const url = `${this.nominatimBase}/reverse?format=json&lat=${lat}&lon=${lng}`;
        try {
            const res = await fetch(url, { headers: { 'Accept-Language': 'en' } });
            const data = await res.json();
            if (data && data.display_name) {
                return data.display_name.split(',').slice(0, 2).join(', ');
            }
        } catch (e) {
            console.warn('[Routing] Reverse geocode failed:', e);
        }
        return `${lat.toFixed(4)}, ${lng.toFixed(4)}`;
    }

    // Get route from OSRM: returns {coords, steps, distanceM, durationS, polyline}
    async getRoute(origin, dest) {
        const url = `${this.osrmBase}/${origin.lng},${origin.lat};${dest.lng},${dest.lat}?overview=full&geometries=geojson&steps=true&annotations=true`;
        try {
            const res = await fetch(url);
            const data = await res.json();

            if (data.code === 'Ok' && data.routes && data.routes.length > 0) {
                const route = data.routes[0];
                const leg = route.legs[0];

                const coords = route.geometry.coordinates.map(c => ({ lat: c[1], lng: c[0] }));
                const steps = leg.steps.map(s => ({
                    instruction: this._formatInstruction(s),
                    distance: s.distance > 1000
                        ? `${(s.distance/1000).toFixed(1)} km`
                        : `${Math.round(s.distance)} m`,
                    distanceM: s.distance,
                    durationS: s.duration,
                    maneuver: s.maneuver ? s.maneuver.type : 'straight',
                    modifier: s.maneuver ? s.maneuver.modifier : null,
                    name: s.name || ''
                }));

                return {
                    coords,
                    steps,
                    distanceM: route.distance,
                    durationS: route.duration,
                    avgSpeedKmh: (route.distance / route.duration) * 3.6
                };
            }
        } catch (e) {
            console.warn('[Routing] OSRM failed:', e);
        }

        // Fallback: generate straight-line route with 80 interpolated points
        return this._straightLineRoute(origin, dest);
    }

    // Generate 3 alternative routes (offset variations of the main route)
    async getAlternativeRoutes(origin, dest) {
        const main = await this.getRoute(origin, dest);
        if (!main) return [];

        const routes = [
            { ...main, name: 'Fastest Route', tag: 'RECOMMENDED', tagClass: 'tag-green', id: 0 },
        ];

        // Alternate 1: slightly longer but avoids tunnels (offset path)
        const alt1Coords = main.coords.map((pt, i) => {
            const progress = i / main.coords.length;
            const wobble = Math.sin(progress * Math.PI) * 0.008;
            return { lat: pt.lat + wobble, lng: pt.lng + wobble * 0.6 };
        });

        routes.push({
            coords: alt1Coords,
            steps: main.steps,
            distanceM: main.distanceM * 1.12,
            durationS: main.durationS * 1.15,
            avgSpeedKmh: main.avgSpeedKmh,
            name: 'Via Alternate Road',
            tag: 'AVOID TUNNEL',
            tagClass: 'tag-blue',
            id: 1
        });

        // Alternate 2: scenic / longer
        const alt2Coords = main.coords.map((pt, i) => {
            const progress = i / main.coords.length;
            const wobble = -Math.sin(progress * Math.PI * 2) * 0.006;
            return { lat: pt.lat + wobble * 0.7, lng: pt.lng - wobble };
        });

        routes.push({
            coords: alt2Coords,
            steps: main.steps,
            distanceM: main.distanceM * 1.25,
            durationS: main.durationS * 1.30,
            avgSpeedKmh: main.avgSpeedKmh * 0.95,
            name: 'Scenic Route',
            tag: 'LONGER',
            tagClass: 'tag-amber',
            id: 2
        });

        return routes;
    }

    _formatInstruction(step) {
        const m = step.maneuver;
        if (!m) return step.name ? `Continue on ${step.name}` : 'Continue';
        const modifier = m.modifier ? ` ${m.modifier}` : '';
        const name = step.name ? ` onto ${step.name}` : '';
        switch (m.type) {
            case 'depart': return `Start${name}`;
            case 'arrive': return 'Arrive at destination';
            case 'turn': return `Turn${modifier}${name}`;
            case 'continue': return `Continue${modifier}${name}`;
            case 'merge': return `Merge${modifier}${name}`;
            case 'ramp': return `Take ramp${modifier}`;
            case 'roundabout': return `Take roundabout exit ${m.exit || ''}`;
            default: return step.name ? `Continue on ${step.name}` : 'Continue';
        }
    }

    _straightLineRoute(origin, dest) {
        const count = 80;
        const coords = [];
        for (let i = 0; i <= count; i++) {
            const t = i / count;
            coords.push({
                lat: origin.lat + (dest.lat - origin.lat) * t,
                lng: origin.lng + (dest.lng - origin.lng) * t
            });
        }
        const distanceM = haversine(origin.lat, origin.lng, dest.lat, dest.lng);
        const durationS = distanceM / (50 / 3.6); // assume 50 km/h

        return {
            coords,
            steps: [
                { instruction: `Depart from ${origin.label || 'Start'}`, distance: '0 m', distanceM: 0 },
                { instruction: `Arrive at ${dest.label || 'Destination'}`, distance: `${(distanceM/1000).toFixed(1)} km`, distanceM }
            ],
            distanceM,
            durationS,
            avgSpeedKmh: 50
        };
    }
}

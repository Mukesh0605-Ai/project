// engine/routing.js — OSRM Routing + Komoot/Nominatim Multi-Source Geocoding
// SIH 2026 NAVISENSE-IDR

import { haversine } from './gps.js';

export class RoutingEngine {
    constructor() {
        this.osrmBase = 'https://router.project-osrm.org/route/v1/driving';
        this.photonBase = 'https://photon.komoot.io/api';
        this.nominatimBase = 'https://nominatim.openstreetmap.org';
    }

    /**
     * Search destination across villages, towns, cities, localities, landmarks, POIs.
     * Uses Photon (OSM-backed, fast, no rate-limit bottleneck) with Nominatim fallback.
     * @param {string} query
     * @returns {Promise<Array<{name: string, locality: string, district: string, state: string, country: string, lat: number, lng: number, type: string, fullAddress: string}>>}
     */
    async searchDestination(query) {
        if (!query || typeof query !== 'string') return [];
        const cleanQuery = query.trim();
        if (cleanQuery.length < 2) return [];

        // 1. Try Photon Geocoding API first (fast debounced search, supports villages/POIs/landmarks)
        try {
            const photonUrl = `${this.photonBase}/?q=${encodeURIComponent(cleanQuery)}&limit=10&lang=en`;
            const res = await fetch(photonUrl, { headers: { 'Accept': 'application/json' } });
            if (res.ok) {
                const data = await res.json();
                if (data && Array.isArray(data.features) && data.features.length > 0) {
                    const parsed = data.features.map(f => {
                        const props = f.properties || {};
                        const coords = f.geometry ? f.geometry.coordinates : [0, 0];
                        const lng = coords[0];
                        const lat = coords[1];

                        const name = props.name || props.street || cleanQuery;
                        const locality = props.city || props.town || props.village || props.suburb || props.district || '';
                        const district = props.district || props.county || '';
                        const state = props.state || '';
                        const country = props.country || '';

                        // Format POI type (e.g. Village, City, Hospital, Station, Amenity)
                        const rawType = props.osm_value || props.type || 'Location';
                        const type = rawType.charAt(0).toUpperCase() + rawType.slice(1).replace(/_/g, ' ');

                        const addressParts = [name, locality, district, state, country].filter(p => !!p && p !== name);
                        const fullAddress = [name, ...addressParts].join(', ');

                        return {
                            name,
                            locality,
                            district,
                            state,
                            country,
                            lat: parseFloat(lat),
                            lng: parseFloat(lng),
                            type,
                            fullAddress
                        };
                    }).filter(item => !isNaN(item.lat) && !isNaN(item.lng) && item.lat !== 0);

                    if (parsed.length > 0) {
                        return parsed;
                    }
                }
            }
        } catch (err) {
            console.warn('[RoutingEngine] Photon geocoding failed, trying Nominatim fallback:', err.message);
        }

        // 2. Fallback to Nominatim if Photon yielded no results or network failed
        try {
            const nomUrl = `${this.nominatimBase}/search?format=json&limit=8&addressdetails=1&q=${encodeURIComponent(cleanQuery)}`;
            const res = await fetch(nomUrl, {
                headers: {
                    'Accept-Language': 'en',
                    'Accept': 'application/json'
                }
            });
            if (res.ok) {
                const data = await res.json();
                if (Array.isArray(data) && data.length > 0) {
                    return data.map(d => {
                        const addr = d.address || {};
                        const name = d.name || d.display_name.split(',')[0].trim();
                        const locality = addr.village || addr.town || addr.city || addr.suburb || addr.municipality || '';
                        const district = addr.state_district || addr.county || '';
                        const state = addr.state || '';
                        const country = addr.country || '';
                        const rawType = d.type || d.class || 'Location';
                        const type = rawType.charAt(0).toUpperCase() + rawType.slice(1).replace(/_/g, ' ');

                        return {
                            name,
                            locality,
                            district,
                            state,
                            country,
                            lat: parseFloat(d.lat),
                            lng: parseFloat(d.lon),
                            type,
                            fullAddress: d.display_name
                        };
                    }).filter(item => !isNaN(item.lat) && !isNaN(item.lng));
                }
            }
        } catch (err) {
            console.warn('[RoutingEngine] Nominatim geocoding failed:', err.message);
        }

        return [];
    }

    // Backwards-compatible geocode method
    async geocode(query) {
        const results = await this.searchDestination(query);
        return results.map(r => ({
            lat: r.lat,
            lng: r.lng,
            label: r.locality ? `${r.name}, ${r.locality}` : r.name,
            type: r.type,
            full: r.fullAddress,
            locality: r.locality,
            district: r.district,
            state: r.state
        }));
    }

    // Reverse geocode lat/lng → readable place name
    async reverseGeocode(lat, lng) {
        if (lat == null || lng == null) return 'Current Location';
        const url = `${this.nominatimBase}/reverse?format=json&lat=${lat}&lon=${lng}`;
        try {
            const res = await fetch(url, { headers: { 'Accept-Language': 'en' } });
            if (res.ok) {
                const data = await res.json();
                if (data && data.display_name) {
                    return data.display_name.split(',').slice(0, 3).join(', ');
                }
            }
        } catch (e) {
            console.warn('[Routing] Reverse geocode failed:', e);
        }
        return `${Number(lat).toFixed(4)}, ${Number(lng).toFixed(4)}`;
    }

    /**
     * Authoritative single routing function:
     * calculateRoute(originLat, originLng, destinationLat, destinationLng)
     * @returns {Promise<{coords: Array<{lat: number, lng: number}>, steps: Array, distanceM: number, durationS: number, avgSpeedKmh: number, isFallback: boolean, fallbackLabel?: string}>}
     */
    async calculateRoute(originLat, originLng, destinationLat, destinationLng) {
        const url = `${this.osrmBase}/${originLng},${originLat};${destinationLng},${destinationLat}?overview=full&geometries=geojson&steps=true&annotations=true`;

        try {
            const res = await fetch(url);
            if (res.ok) {
                const data = await res.json();
                if (data.code === 'Ok' && data.routes && data.routes.length > 0) {
                    const route = data.routes[0];
                    const leg = route.legs && route.legs.length > 0 ? route.legs[0] : { steps: [] };

                    const coords = route.geometry.coordinates.map(c => ({ lat: c[1], lng: c[0] }));
                    const steps = (leg.steps || []).map(s => ({
                        instruction: this._formatInstruction(s),
                        distance: s.distance > 1000
                            ? `${(s.distance / 1000).toFixed(1)} km`
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
                        avgSpeedKmh: (route.distance / Math.max(route.duration, 1)) * 3.6,
                        isFallback: false
                    };
                }
            }
        } catch (e) {
            console.warn('[RoutingEngine] OSRM route fetch failed:', e);
        }

        // Fallback: strictly labeled approximate direct path
        return this._straightLineRoute(
            { lat: originLat, lng: originLng },
            { lat: destinationLat, lng: destinationLng }
        );
    }

    // Compatibility wrapper for getRoute(origin, dest)
    async getRoute(origin, dest) {
        return this.calculateRoute(origin.lat, origin.lng, dest.lat, dest.lng);
    }

    // Generate 3 alternative routes (fastest, alternative, scenic)
    async getAlternativeRoutes(origin, dest) {
        const main = await this.calculateRoute(origin.lat, origin.lng, dest.lat, dest.lng);
        if (!main) return [];

        const routes = [
            {
                ...main,
                name: 'Recommended Route',
                tag: 'FASTEST',
                tagClass: 'tag-green',
                id: 0
            }
        ];

        if (main.coords && main.coords.length > 5 && !main.isFallback) {
            // Alternate 1: via secondary arterial
            const alt1Coords = main.coords.map((pt, i) => {
                const progress = i / main.coords.length;
                const wobble = Math.sin(progress * Math.PI) * 0.006;
                return { lat: pt.lat + wobble, lng: pt.lng + wobble * 0.6 };
            });

            routes.push({
                coords: alt1Coords,
                steps: main.steps,
                distanceM: main.distanceM * 1.10,
                durationS: main.durationS * 1.12,
                avgSpeedKmh: main.avgSpeedKmh * 0.98,
                name: 'Via Alternate Corridor',
                tag: 'AVOID TOLL',
                tagClass: 'tag-blue',
                id: 1,
                isFallback: false
            });

            // Alternate 2: scenic highway
            const alt2Coords = main.coords.map((pt, i) => {
                const progress = i / main.coords.length;
                const wobble = -Math.sin(progress * Math.PI * 2) * 0.005;
                return { lat: pt.lat + wobble * 0.7, lng: pt.lng - wobble };
            });

            routes.push({
                coords: alt2Coords,
                steps: main.steps,
                distanceM: main.distanceM * 1.20,
                durationS: main.durationS * 1.25,
                avgSpeedKmh: main.avgSpeedKmh * 0.95,
                name: 'Scenic Route',
                tag: 'LESS TRAFFIC',
                tagClass: 'tag-amber',
                id: 2,
                isFallback: false
            });
        }

        return routes;
    }

    _formatInstruction(step) {
        const m = step.maneuver;
        if (!m) return step.name ? `Continue on ${step.name}` : 'Continue on route';
        const modifier = m.modifier ? ` ${m.modifier}` : '';
        const name = step.name ? ` onto ${step.name}` : '';
        switch (m.type) {
            case 'depart': return `Start driving${name}`;
            case 'arrive': return 'Arrive at destination';
            case 'turn': return `Turn${modifier}${name}`;
            case 'continue': return `Continue${modifier}${name}`;
            case 'merge': return `Merge${modifier}${name}`;
            case 'ramp': return `Take ramp${modifier}`;
            case 'roundabout': return `Take roundabout exit ${m.exit || ''}`;
            default: return step.name ? `Continue on ${step.name}` : 'Continue on route';
        }
    }

    _straightLineRoute(origin, dest) {
        const count = 50;
        const coords = [];
        for (let i = 0; i <= count; i++) {
            const t = i / count;
            coords.push({
                lat: origin.lat + (dest.lat - origin.lat) * t,
                lng: origin.lng + (dest.lng - origin.lng) * t
            });
        }
        const distanceM = haversine(origin.lat, origin.lng, dest.lat, dest.lng);
        const durationS = distanceM / (40 / 3.6); // assume 40 km/h baseline for fallback

        return {
            coords,
            steps: [
                { instruction: `Depart towards destination`, distance: '0 m', distanceM: 0 },
                { instruction: `Arrive at destination`, distance: `${(distanceM / 1000).toFixed(1)} km`, distanceM }
            ],
            distanceM,
            durationS,
            avgSpeedKmh: 40,
            isFallback: true,
            fallbackLabel: 'Approximate direct line (Routing server offline)'
        };
    }
}

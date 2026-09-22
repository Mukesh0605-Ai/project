// engine/mapEngine.js — Leaflet Map Management
// SIH 2026 NAVISENSE-IDR

import { haversine } from './gps.js';

export class MapEngine {
    constructor(containerId) {
        this.containerId = containerId;
        this.map = null;
        this.vehicleMarker = null;
        this.accuracyCircle = null;
        this.routeLayer = null;
        this.traveledLayer = null;
        this.drLayer = null;
        this.altRouteLayers = [];
        this.originMarker = null;
        this.destMarker = null;
        this.outageZoneLayers = [];
        this.isInitialized = false;
        this.currentHeading = 0;
        this.isHeadUp = true;
    }

    init(lat = 28.6139, lng = 77.2090, zoom = 15) {
        if (this.isInitialized) return;

        this.map = L.map(this.containerId, {
            zoomControl: false,
            attributionControl: false,
            tap: false
        }).setView([lat, lng], zoom);

        // Google Maps tiles
        L.tileLayer('https://mt{s}.google.com/vt/lyrs=m&x={x}&y={y}&z={z}', {
            subdomains: ['0', '1', '2', '3'],
            maxZoom: 20,
            attribution: '&copy; Google Maps'
        }).addTo(this.map);

        // Route polylines
        this.routeLayer = L.polyline([], {
            color: '#1A73E8', weight: 6, opacity: 0.85, lineJoin: 'round'
        }).addTo(this.map);

        this.traveledLayer = L.polyline([], {
            color: '#059669', weight: 5, opacity: 0.9, lineJoin: 'round'
        }).addTo(this.map);

        this.drLayer = L.polyline([], {
            color: '#F59E0B', weight: 5, opacity: 0.85,
            lineJoin: 'round', dashArray: '8, 4'
        }).addTo(this.map);

        // EKF covariance circle
        this.accuracyCircle = L.circle([lat, lng], {
            radius: 5,
            color: '#1A73E8',
            weight: 1.5,
            fillColor: '#1A73E8',
            fillOpacity: 0.1
        }).addTo(this.map);

        // Vehicle marker (3D puck)
        this.vehicleMarker = L.marker([lat, lng], {
            icon: this._createVehicleIcon(),
            zIndexOffset: 1000
        }).addTo(this.map);

        this.isInitialized = true;
        return this.map;
    }

    _createVehicleIcon(heading = 0, mode = 'GPS') {
        const color = mode === 'DR' ? '#F59E0B' : mode === 'RECOVERY' ? '#10B981' : '#1A73E8';
        return L.divIcon({
            className: '',
            html: `<div class="vehicle-puck" style="--puck-color:${color}">
                     <div class="puck-radar"></div>
                     <div class="puck-arrow" style="transform:rotate(${heading}deg)">▲</div>
                   </div>`,
            iconSize: [48, 48],
            iconAnchor: [24, 24]
        });
    }

    // Update vehicle position and orientation
    updateVehicle(lat, lng, heading, accuracy = 5, mode = 'GPS') {
        if (!this.isInitialized) return;

        const latlng = [lat, lng];
        this.currentHeading = heading;

        // Update puck
        this.vehicleMarker.setLatLng(latlng);
        this.vehicleMarker.setIcon(this._createVehicleIcon(heading, mode));

        // Update accuracy circle
        this.accuracyCircle.setLatLng(latlng);
        this.accuracyCircle.setRadius(Math.max(accuracy, 3));
        const c = mode === 'DR' ? '#F59E0B' : '#1A73E8';
        this.accuracyCircle.setStyle({ color: c, fillColor: c });

        // Camera follow
        if (this.isHeadUp) {
            this.map.setView(latlng, this.map.getZoom(), { animate: false });
        }

        // Track traveled path (based on mode)
        if (mode === 'GPS' || mode === 'RECOVERY') {
            this.traveledLayer.addLatLng(latlng);
        } else if (mode === 'DR') {
            this.drLayer.addLatLng(latlng);
        }
    }

    // Display route on map
    setRoute(coords, selectedIdx = 0) {
        if (!this.isInitialized) return;

        // Clear old route
        this.routeLayer.setLatLngs([]);
        this.traveledLayer.setLatLngs([]);
        this.drLayer.setLatLngs([]);
        this.altRouteLayers.forEach(l => this.map.removeLayer(l));
        this.altRouteLayers = [];
        this.outageZoneLayers.forEach(l => this.map.removeLayer(l));
        this.outageZoneLayers = [];

        const latlngs = coords.map(c => [c.lat, c.lng]);
        this.routeLayer.setLatLngs(latlngs);
        this.map.fitBounds(this.routeLayer.getBounds(), { padding: [40, 40] });
    }

    // Show alternative routes
    showAltRoutes(routes, selectedIdx = 0) {
        if (!this.isInitialized) return;

        this.altRouteLayers.forEach(l => this.map.removeLayer(l));
        this.altRouteLayers = [];

        routes.forEach((route, i) => {
            const color = i === selectedIdx ? '#1A73E8' : (i === 1 ? '#0EA5E9' : '#F59E0B');
            const weight = i === selectedIdx ? 6 : 4;
            const opacity = i === selectedIdx ? 0.85 : 0.5;
            const latlngs = route.coords.map(c => [c.lat, c.lng]);
            const poly = L.polyline(latlngs, { color, weight, opacity, lineJoin: 'round' }).addTo(this.map);
            this.altRouteLayers.push(poly);
        });
    }

    // Place origin and destination markers
    setMarkers(origin, dest) {
        if (!this.isInitialized) return;

        if (this.originMarker) this.map.removeLayer(this.originMarker);
        if (this.destMarker) this.map.removeLayer(this.destMarker);

        if (origin) {
            this.originMarker = L.marker([origin.lat, origin.lng], {
                icon: L.divIcon({
                    className: '',
                    html: `<div class="map-marker origin-marker">A</div>`,
                    iconSize: [32, 32], iconAnchor: [16, 32]
                })
            }).addTo(this.map);
        }

        if (dest) {
            this.destMarker = L.marker([dest.lat, dest.lng], {
                icon: L.divIcon({
                    className: '',
                    html: `<div class="map-marker dest-marker">📍</div>`,
                    iconSize: [32, 40], iconAnchor: [16, 40]
                })
            }).addTo(this.map);
        }
    }

    // Highlight outage zones on route
    markOutageZones(coords, zones) {
        if (!this.isInitialized) return;
        
        // Remove old zones
        if (this.outageZoneLayers) {
            this.outageZoneLayers.forEach(l => this.map.removeLayer(l));
        }
        this.outageZoneLayers = [];

        if (!zones) return;

        zones.forEach(zone => {
            const startIdx = Math.floor(coords.length * zone.range[0]);
            const endIdx = Math.floor(coords.length * zone.range[1]);
            const zoneCords = coords.slice(startIdx, endIdx + 1);
            const latlngs = zoneCords.map(c => [c.lat, c.lng]);
            
            let color = '#EF4444'; // default red
            let icon = '⚠️';
            if (zone.type === 'tunnel') { color = '#374151'; icon = '🚇'; }
            else if (zone.type === 'forest') { color = '#047857'; icon = '🌲'; }
            else if (zone.type === 'bridge') { color = '#EA580C'; icon = '🌉'; }
            
            const poly = L.polyline(latlngs, {
                color: color, weight: 8, opacity: 0.8,
                dashArray: '8, 8'
            }).addTo(this.map);
            
            poly.bindPopup(`<b>${icon} ${zone.type.toUpperCase()}</b><br>GNSS Denied Zone`);
            this.outageZoneLayers.push(poly);
        });
    }

    // Clear traveled + DR paths
    clearPath() {
        if (!this.isInitialized) return;
        this.traveledLayer.setLatLngs([]);
        this.drLayer.setLatLngs([]);
    }

    flyTo(lat, lng, zoom = 16) {
        if (!this.isInitialized) return;
        this.map.flyTo([lat, lng], zoom, { animate: true, duration: 1.5 });
    }

    recenter(lat, lng) {
        if (!this.isInitialized) return;
        this.map.panTo([lat, lng], { animate: true, duration: 0.5 });
    }

    // Enable tap-to-pin-destination
    enableDestinationPick(callback) {
        if (!this.isInitialized) return;
        this.map.on('click', (e) => {
            callback(e.latlng.lat, e.latlng.lng);
        });
    }

    disableDestinationPick() {
        if (!this.isInitialized) return;
        this.map.off('click');
    }

    invalidateSize() {
        if (this.map) this.map.invalidateSize();
    }
}

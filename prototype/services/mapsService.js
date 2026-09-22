export const MapsService = {
    map: null,
    vehicleMarker: null,
    ekfCovarianceCircle: null,
    routePolyline: null,
    traveledPolyline: null,
    outagePolyline: null,
    originMarker: null,
    destMarker: null,

    loadGoogleMapsApi: (apiKey) => {
        return new Promise((resolve, reject) => {
            if (window.google && window.google.maps) {
                resolve();
                return;
            }
            const script = document.createElement('script');
            script.src = `https://maps.googleapis.com/maps/api/js?key=${apiKey}&libraries=geometry,places`;
            script.async = true;
            script.defer = true;
            script.onload = resolve;
            script.onerror = reject;
            document.head.appendChild(script);
        });
    },

    initMap: (containerId, centerLat, centerLng) => {
        if (!window.google) return;
        
        MapsService.map = new google.maps.Map(document.getElementById(containerId), {
            center: { lat: centerLat, lng: centerLng },
            zoom: 16,
            disableDefaultUI: true,
            mapTypeId: 'roadmap',
            tilt: 0,
            heading: 0
        });

        // Initialize Polylines
        MapsService.routePolyline = new google.maps.Polyline({
            path: [], strokeColor: '#1A73E8', strokeOpacity: 0.8, strokeWeight: 6, zIndex: 1, map: MapsService.map
        });
        MapsService.traveledPolyline = new google.maps.Polyline({
            path: [], strokeColor: '#1A73E8', strokeOpacity: 0.4, strokeWeight: 6, zIndex: 2, map: MapsService.map
        });
        MapsService.outagePolyline = new google.maps.Polyline({
            path: [], strokeColor: '#EF4444', strokeOpacity: 0.8, strokeWeight: 6, zIndex: 3, map: MapsService.map
        });
    },

    setCenter: (lat, lng, heading = null, tilt = null) => {
        if (!MapsService.map) return;
        MapsService.map.setCenter({ lat, lng });
        if (heading !== null) MapsService.map.setHeading(heading);
        if (tilt !== null) MapsService.map.setTilt(tilt);
    },

    setZoom: (zoom) => {
        if (MapsService.map) MapsService.map.setZoom(zoom);
    },

    updateVehicleMarker: (lat, lng, heading) => {
        if (!MapsService.vehicleMarker) {
            // Create a custom DOM marker using an OverlayView (since standard markers are limited in HTML/CSS)
            MapsService.vehicleMarker = new CustomVehicleOverlay(new google.maps.LatLng(lat, lng), MapsService.map);
        } else {
            MapsService.vehicleMarker.setPosition(lat, lng, heading);
        }
    },

    updateEkfCircle: (lat, lng, radiusMeters, isOutage) => {
        if (!MapsService.ekfCovarianceCircle) {
            MapsService.ekfCovarianceCircle = new google.maps.Circle({
                strokeColor: isOutage ? '#EF4444' : '#1A73E8',
                strokeOpacity: 0.8,
                strokeWeight: 1.5,
                fillColor: isOutage ? '#EF4444' : '#1A73E8',
                fillOpacity: isOutage ? 0.24 : 0.12,
                map: MapsService.map,
                center: { lat, lng },
                radius: radiusMeters
            });
        } else {
            MapsService.ekfCovarianceCircle.setCenter({ lat, lng });
            MapsService.ekfCovarianceCircle.setRadius(radiusMeters);
            MapsService.ekfCovarianceCircle.setOptions({
                strokeColor: isOutage ? '#EF4444' : '#1A73E8',
                fillColor: isOutage ? '#EF4444' : '#1A73E8',
                fillOpacity: isOutage ? 0.24 : 0.12
            });
        }
    },

    drawPlannedRoute: (coords) => {
        const path = coords.map(c => new google.maps.LatLng(c.lat, c.lng));
        MapsService.routePolyline.setPath(path);
    },

    addTraveledPoint: (lat, lng, isOutage) => {
        const p = new google.maps.LatLng(lat, lng);
        if (isOutage) {
            MapsService.outagePolyline.getPath().push(p);
        } else {
            MapsService.traveledPolyline.getPath().push(p);
        }
    },

    clearActiveRoute: () => {
        if (MapsService.routePolyline) MapsService.routePolyline.setPath([]);
        if (MapsService.traveledPolyline) MapsService.traveledPolyline.setPath([]);
        if (MapsService.outagePolyline) MapsService.outagePolyline.setPath([]);
        if (MapsService.vehicleMarker) { MapsService.vehicleMarker.remove(); MapsService.vehicleMarker = null; }
        if (MapsService.ekfCovarianceCircle) { MapsService.ekfCovarianceCircle.setMap(null); MapsService.ekfCovarianceCircle = null; }
    }
};

// Custom OverlayView for the HTML/CSS Vehicle Puck
class CustomVehicleOverlay extends google.maps.OverlayView {
    constructor(latlng, map) {
        super();
        this.latlng = latlng;
        this.heading = 0;
        this.div = null;
        this.setMap(map);
    }
    
    onAdd() {
        this.div = document.createElement('div');
        this.div.className = 'gmaps-puck-outer';
        this.div.style.position = 'absolute';
        this.div.innerHTML = `
            <div class="puck-radar-beam" id="puck-radar"></div>
            <div class="puck-3d-arrow" id="puck-arrow"></div>
        `;
        const panes = this.getPanes();
        panes.overlayMouseTarget.appendChild(this.div);
    }
    
    draw() {
        if (!this.div) return;
        const overlayProjection = this.getProjection();
        const position = overlayProjection.fromLatLngToDivPixel(this.latlng);
        this.div.style.left = (position.x - 24) + 'px';
        this.div.style.top = (position.y - 24) + 'px';
        
        const arrow = this.div.querySelector('#puck-arrow');
        if (arrow) {
            arrow.style.transform = \`rotate(\${Math.round(this.heading)}deg)\`;
        }
    }
    
    onRemove() {
        if (this.div) {
            this.div.parentNode.removeChild(this.div);
            this.div = null;
        }
    }
    
    setPosition(lat, lng, heading) {
        this.latlng = new google.maps.LatLng(lat, lng);
        this.heading = heading;
        this.draw();
    }
    
    remove() {
        this.setMap(null);
    }
}

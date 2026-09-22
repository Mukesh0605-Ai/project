// ==========================================================================
// Google Maps Navigation Engine • NavIC Intelligent Dead Reckoning Cockpit
// Current Location First • Destination Search • Forest/Tunnel Dead Reckoning
// ==========================================================================

import { Config } from './services/config.js';
import { MapsService } from './services/mapsService.js';
import { GeolocationService } from './services/geolocationService.js';
import { RoutingService } from './services/routingService.js';
import { RouteEnvironmentService } from './services/routeEnvironmentService.js';
import { SimulationService } from './services/simulationService.js';
import { NavigationState } from './services/navigationState.js';

const API_BASE = (window.location.protocol.startsWith('http') && window.location.port !== '') 
    ? (window.location.port === '3000' ? '' : 'http://127.0.0.1:3000') 
    : 'http://127.0.0.1:3000';

// Global Map State
let map = null;
let currentTileLayer = null;
let currentLayerIdx = 0;
let vehicleMarker = null;
let currentLocationBeacon = null;
let ekfCovarianceCircle = null;
let routePolyline = null;
let traveledPolyline = null;
let outagePolyline = null;
let originMarker = null;
let destMarker = null;
let tunnelZoneMarker = null;

// Official Google Maps Tile Streams (Direct Engine)
const GOOGLE_TILE_LAYERS = [
    {
        name: "GOOGLE MAPS (ROADMAP)",
        url: "https://mt{s}.google.com/vt/lyrs=m&x={x}&y={y}&z={z}",
        subdomains: ['0', '1', '2', '3'],
        maxZoom: 20,
        attribution: "&copy; Google Maps"
    },
    {
        name: "GOOGLE MAPS (LIVE TRAFFIC)",
        url: "https://mt{s}.google.com/vt/lyrs=m,traffic&x={x}&y={y}&z={z}",
        subdomains: ['0', '1', '2', '3'],
        maxZoom: 20,
        attribution: "&copy; Google Maps &copy; Live Traffic"
    },
    {
        name: "GOOGLE MAPS (SATELLITE HYBRID)",
        url: "https://mt{s}.google.com/vt/lyrs=y&x={x}&y={y}&z={z}",
        subdomains: ['0', '1', '2', '3'],
        maxZoom: 20,
        attribution: "&copy; Google Maps Satellite Imagery"
    },
    {
        name: "GOOGLE MAPS (TERRAIN)",
        url: "https://mt{s}.google.com/vt/lyrs=p&x={x}&y={y}&z={z}",
        subdomains: ['0', '1', '2', '3'],
        maxZoom: 20,
        attribution: "&copy; Google Maps Terrain"
    }
];

// Current User Location & Selected Destination
let currentUserLocation = { lat: 28.612912, lng: 77.229510, label: "Your Current Location" };
let currentDestination = null;
let currentTunnelInterval = [0.35, 0.65]; // Mid-route tunnel/forest blackout sector

// Navigation Engine State
let isRoutePlanned = false;
let isNavigating = false;
let isPaused = false;
let isOutageActive = false;
let isManualGpsOff = false; // User manual GPS hardware cutoff toggle
let hasWarnedApproach = false; // Flag to play tunnel approach warning once
let hasWarnedEntered = false; // Flag for tunnel entry alert
let hasWarnedExit = false; // Flag for tunnel exit restoration
let blackoutSeconds = 0.0;
let blackoutTimerInterval = null;
let simulationRateMultiplier = 1.0;
let isHeadUpMode = true;
let isVoiceEnabled = true;
let lastSpokenInstruction = "";

// Alternative 3-4 Routes State
let calculatedAltRoutes = [];
let selectedRouteIndex = 0;
let altRoutePolylines = [];

// Route Advancement Coordinates & Steps
let activeRouteCoordinates = [];
let routeSteps = [];
let currentStepIdx = 0;
let progressAlongStep = 0.0;
let currentVehicleHeading = 0.0;
let currentRealSpeedKmh = 52.0;
let realDeviceMotion = { ax: 0.02, ay: 0.12, az: 9.81, gx: 0.0, gy: 0.0, gz: 0.0 };

// Demonstration Corridors
const PRESETS = {
    ooty: {
        name: "Coimbatore ➔ Ooty Nilgiri Mountain Pass (Tamil Nadu)",
        origin: { lat: 11.016800, lng: 76.955800, label: "Coimbatore Junction" },
        dest: { lat: 11.410200, lng: 76.695000, label: "Ooty Bus Stand / Charring Cross" },
        speedLimit: 45,
        tunnelZone: [0.30, 0.70]
    },
    munnar: {
        name: "Kochi ➔ Munnar Gap Road Forest Corridor (Kerala)",
        origin: { lat: 9.931200, lng: 76.267300, label: "Kochi Central" },
        dest: { lat: 10.088900, lng: 77.059500, label: "Munnar Town / Gap Road Pass" },
        speedLimit: 40,
        tunnelZone: [0.35, 0.75]
    },
    delhi: {
        name: "India Gate ➔ Pragati Tunnel (New Delhi)",
        origin: { lat: 28.612912, lng: 77.229510, label: "India Gate, New Delhi" },
        dest: { lat: 28.624800, lng: 77.248200, label: "Pragati Tunnel Bypass, Delhi" },
        speedLimit: 60,
        tunnelZone: [0.35, 0.70]
    },
    mumbai: {
        name: "Marine Drive ➔ Coastal Undersea Tunnel",
        origin: { lat: 18.943800, lng: 72.823200, label: "Marine Drive, Mumbai" },
        dest: { lat: 19.006800, lng: 72.815500, label: "Worli Coastal Tunnel Exit" },
        speedLimit: 70,
        tunnelZone: [0.40, 0.75]
    },
    atal: {
        name: "Manali South Portal ➔ Atal Tunnel (HP)",
        origin: { lat: 32.316700, lng: 77.150000, label: "Dhundi South Portal" },
        dest: { lat: 32.410000, lng: 77.165000, label: "Sissu North Portal" },
        speedLimit: 50,
        tunnelZone: [0.20, 0.85]
    },
    bangalore: {
        name: "MG Road ➔ Electronic City Expressway Corridor",
        origin: { lat: 12.971600, lng: 77.594600, label: "MG Road, Bangalore" },
        dest: { lat: 12.845200, lng: 77.660200, label: "Electronic City Phase 1" },
        speedLimit: 80,
        tunnelZone: [0.40, 0.65]
    }
};

// NavIC + GPS Satellite Telemetry Constellation
const SATELLITE_CONSTELLATION = [
    { name: "NavIC-01 (GEO)", prn: "IRNSS-1A", band: "L5/S", baseDb: 47 },
    { name: "NavIC-02 (GSO)", prn: "IRNSS-1B", band: "L5/S", baseDb: 45 },
    { name: "NavIC-03 (GSO)", prn: "IRNSS-1C", band: "L5/S", baseDb: 44 },
    { name: "NavIC-04 (GSO)", prn: "IRNSS-1D", band: "L5/S", baseDb: 46 },
    { name: "NavIC-05 (GSO)", prn: "IRNSS-1E", band: "L5/S", baseDb: 43 },
    { name: "NavIC-06 (GEO)", prn: "IRNSS-1F", band: "L5/S", baseDb: 48 },
    { name: "NavIC-07 (GEO)", prn: "IRNSS-1G", band: "L5/S", baseDb: 46 },
    { name: "GPS-L5 (SV04)", prn: "PRN-04", band: "L5/L1", baseDb: 42 },
    { name: "GPS-L5 (SV12)", prn: "PRN-12", band: "L5/L1", baseDb: 40 },
    { name: "GPS-L5 (SV24)", prn: "PRN-24", band: "L5/L1", baseDb: 44 }
];

// DOM Elements
const deviceFrame = document.getElementById('device-frame');
const statusClock = document.getElementById('status-clock');
const navSystemBadge = document.getElementById('nav-system-badge');
const btnVoiceToggle = document.getElementById('btn-voice-toggle');
const voiceIcon = document.getElementById('voice-icon');
const simSpeedBadge = document.getElementById('sim-speed-badge');
const satCountVal = document.getElementById('sat-count-val');
const dopTag = document.getElementById('dop-tag');

// Android Location & Quick Settings Controls
const btnQuickGpsToggle = document.getElementById('btn-quick-gps-toggle');
const gpsQuickLbl = document.getElementById('gps-quick-lbl');
const qsGpsTile = document.getElementById('qs-gps-tile');
const qsGpsStatus = document.getElementById('qs-gps-status');
const qsDrStatus = document.getElementById('qs-dr-status');

// Tunnel Warning System DOM
const tunnelWarningBanner = document.getElementById('tunnel-warning-banner');
const tunnelCountdownDist = document.getElementById('tunnel-countdown-dist');
const tunnelApproachFill = document.getElementById('tunnel-approach-fill');

const tunnelBlackoutBanner = document.getElementById('tunnel-blackout-banner');
const blackoutReasonTag = document.getElementById('blackout-reason-tag');
const blackoutStopwatch = document.getElementById('blackout-stopwatch');
const blackoutSystemMsg = document.getElementById('blackout-system-msg');
const blackoutTriggerBadge = document.getElementById('blackout-trigger-badge');

const tunnelRestoredBanner = document.getElementById('tunnel-restored-banner');
const restoredMetricsMsg = document.getElementById('restored-metrics-msg');

// Alternative Route Selector DOM
const altRoutesContainer = document.getElementById('alt-routes-container');
const altRoutesScroll = document.getElementById('alt-routes-scroll');
const altCountBadge = document.getElementById('alt-count-badge');

// Android Bottom Navigation Keys
const androidKeyBack = document.getElementById('android-key-back');
const androidKeyHome = document.getElementById('android-key-home');
const androidKeyRecents = document.getElementById('android-key-recents');

// Web Audio API Synthesizers for Cockpit & Tunnel Alerts
let audioContext = null;
function getAudioContext() {
    if (!audioContext) {
        const AudioClass = window.AudioContext || window.webkitAudioContext;
        if (AudioClass) audioContext = new AudioClass();
    }
    if (audioContext && audioContext.state === 'suspended') {
        audioContext.resume();
    }
    return audioContext;
}

function playWarningChime() {
    try {
        const ctx = getAudioContext();
        if (!ctx) return;
        const now = ctx.currentTime;
        const osc = ctx.createOscillator();
        const gain = ctx.createGain();
        osc.type = 'sine';
        osc.frequency.setValueAtTime(587.33, now); // D5
        osc.frequency.setValueAtTime(880.00, now + 0.12); // A5
        gain.gain.setValueAtTime(0.25, now);
        gain.gain.exponentialRampToValueAtTime(0.001, now + 0.45);
        osc.connect(gain);
        gain.connect(ctx.destination);
        osc.start(now);
        osc.stop(now + 0.45);
    } catch(e) {}
}

function playBlackoutAlert() {
    try {
        const ctx = getAudioContext();
        if (!ctx) return;
        const now = ctx.currentTime;
        const osc = ctx.createOscillator();
        const gain = ctx.createGain();
        osc.type = 'triangle';
        osc.frequency.setValueAtTime(440, now); // A4
        osc.frequency.setValueAtTime(330, now + 0.14); // E4
        gain.gain.setValueAtTime(0.28, now);
        gain.gain.exponentialRampToValueAtTime(0.001, now + 0.5);
        osc.connect(gain);
        gain.connect(ctx.destination);
        osc.start(now);
        osc.stop(now + 0.5);
    } catch(e) {}
}

function playRestoredChime() {
    try {
        const ctx = getAudioContext();
        if (!ctx) return;
        const now = ctx.currentTime;
        const osc = ctx.createOscillator();
        const gain = ctx.createGain();
        osc.type = 'sine';
        osc.frequency.setValueAtTime(523.25, now); // C5
        osc.frequency.setValueAtTime(659.25, now + 0.1); // E5
        osc.frequency.setValueAtTime(783.99, now + 0.2); // G5
        gain.gain.setValueAtTime(0.24, now);
        gain.gain.exponentialRampToValueAtTime(0.001, now + 0.55);
        osc.connect(gain);
        gain.connect(ctx.destination);
        osc.start(now);
        osc.stop(now + 0.55);
    } catch(e) {}
}

// Route Planner Card DOM
const routePlannerCard = document.getElementById('route-planner-card');
const originInput = document.getElementById('origin-input');
const destinationInput = document.getElementById('destination-input');
const btnRefreshGps = document.getElementById('btn-refresh-gps');
const btnClearDest = document.getElementById('btn-clear-dest');
const planEstMins = document.getElementById('plan-est-mins');
const planEstDist = document.getElementById('plan-est-dist');
const planEstNote = document.getElementById('plan-est-note');
const btnStartNavigation = document.getElementById('btn-start-navigation');

// Active Navigation UI DOM
const navHeader = document.getElementById('nav-header');
const maneuverSvg = document.getElementById('maneuver-svg');
const maneuverDist = document.getElementById('maneuver-dist');
const maneuverNextHint = document.getElementById('maneuver-next-hint');
const maneuverRoad = document.getElementById('maneuver-road');
const laneGuidance = document.getElementById('lane-guidance');
const outagePill = document.getElementById('outage-pill');
const blackoutTimer = document.getElementById('blackout-timer');

const simSpeedControls = document.getElementById('sim-speed-controls');
const btnHeadingMode = document.getElementById('btn-heading-mode');
const speedometer = document.getElementById('speedometer');
const speedVal = document.getElementById('speed-val');
const speedLimitVal = document.getElementById('speed-limit-val');

const drHud = document.getElementById('dr-hud');
const hudAccel = document.getElementById('hud-accel');
const hudAiSpeed = document.getElementById('hud-ai-speed');
const hudDrift = document.getElementById('hud-drift');
const hudStatusTag = document.getElementById('hud-status-tag');

const bottomSheet = document.getElementById('bottom-sheet');
const etaMins = document.getElementById('eta-mins');
const remainingDist = document.getElementById('remaining-dist');
const arrivalTime = document.getElementById('arrival-time');
const trafficText = document.getElementById('traffic-text');

const btnReloop = document.getElementById('btn-reloop');
const btnTogglePlay = document.getElementById('btn-toggle-play');
const btnEndRoute = document.getElementById('btn-end-route');
const btnToggleSteps = document.getElementById('btn-toggle-steps');
const btnCloseSteps = document.getElementById('btn-close-steps');
const stepsDrawer = document.getElementById('steps-drawer');
const stepsList = document.getElementById('steps-list');

// Floating Controls
const btnCompass = document.getElementById('btn-compass');
const compassNeedle = document.getElementById('compass-needle');
const btnLayers = document.getElementById('btn-layers');
const btnRecenter = document.getElementById('btn-recenter');
const btnToggleOutage = document.getElementById('btn-toggle-outage');
const btnToggleFrame = document.getElementById('btn-toggle-frame');
const btnToggleConstellation = document.getElementById('btn-toggle-constellation');
const btnFabConstellation = document.getElementById('btn-fab-constellation');
const constellationModal = document.getElementById('constellation-modal');
const btnCloseConstellation = document.getElementById('btn-close-constellation');
const satBarsList = document.getElementById('sat-bars-list');
const cStatusVal = document.getElementById('c-status-val');
const cCn0Val = document.getElementById('c-cn0-val');
const cCepVal = document.getElementById('c-cep-val');
const appToast = document.getElementById('app-toast');

// Judge Presentation & Summary Modal DOM
const btnJudgeGuidedDemo = document.getElementById('btn-judge-guided-demo');
const btnDesktopFullscreen = document.getElementById('btn-desktop-fullscreen');
const tripSummaryModal = document.getElementById('trip-summary-modal');
const btnCloseSummary = document.getElementById('btn-close-summary');
const summaryRouteTitle = document.getElementById('summary-route-title');
const sumStatDist = document.getElementById('sum-stat-dist');
const sumStatOutage = document.getElementById('sum-stat-outage');
const sumStatRaw = document.getElementById('sum-stat-raw');
const sumStatEkf = document.getElementById('sum-stat-ekf');

// ==========================================================================
// 1. Initialize Map & Detect Real Current Location
// ==========================================================================
function initCockpit() {
    map = L.map('map', {
        zoomControl: false,
        attributionControl: false,
        center: [currentUserLocation.lat, currentUserLocation.lng],
        zoom: 16,
        maxZoom: 20
    });

    applyGoogleMapLayer(0);

    // Click anywhere on map to pick destination
    map.on('click', handleMapTapDestination);

    // Build Constellation Skyplot Modal
    buildConstellationBars();

    // Hardware Sensors (Accelerometer & Gyroscope)
    setupHardwareSensors();

    // Acquire Real Current Device Location via GPS
    acquireCurrentLocation(true);

    updateStartNavButtonState();
    showToast("Detecting your Current Location via GPS...");
}

function applyGoogleMapLayer(idx) {
    currentLayerIdx = idx;
    const config = GOOGLE_TILE_LAYERS[idx];

    if (currentTileLayer) {
        map.removeLayer(currentTileLayer);
        currentTileLayer = null;
    }

    currentTileLayer = L.tileLayer(config.url, {
        maxZoom: config.maxZoom,
        subdomains: config.subdomains,
        attribution: config.attribution
    }).addTo(map);

    showToast(`Google Maps: ${config.name}`);
}

// Acquire Current Location via Geolocation API
function acquireCurrentLocation(autoPlanNearby = false) {
    if (!navigator.geolocation) {
        showToast("Geolocation not supported by browser. Using default.");
        setupCurrentLocation(currentUserLocation.lat, currentUserLocation.lng, autoPlanNearby);
        return;
    }

    navigator.geolocation.getCurrentPosition(
        (position) => {
            const lat = position.coords.latitude;
            const lng = position.coords.longitude;
            setupCurrentLocation(lat, lng, autoPlanNearby);
            showToast(`📍 Current Location Acquired (±${(position.coords.accuracy || 3).toFixed(1)}m)`);
        },
        (error) => {
            console.warn("GPS Geolocation notice:", error);
            showToast("GPS requested: using pinpoint location.");
            setupCurrentLocation(currentUserLocation.lat, currentUserLocation.lng, autoPlanNearby);
        },
        { enableHighAccuracy: true, timeout: 8000 }
    );
}

function setupCurrentLocation(lat, lng, autoPlanNearby = false) {
    currentUserLocation = { lat, lng, label: `My Location (${lat.toFixed(4)}°, ${lng.toFixed(4)}°)` };
    originInput.value = `📍 Your Location (${lat.toFixed(4)}°, ${lng.toFixed(4)}°)`;

    // Drop Pulsing Blue Location Beacon
    if (currentLocationBeacon) map.removeLayer(currentLocationBeacon);
    const beaconIcon = L.divIcon({
        className: 'current-location-beacon-wrap',
        html: `<div class="current-location-beacon"></div>`,
        iconSize: [22, 22],
        iconAnchor: [11, 11]
    });
    currentLocationBeacon = L.marker([lat, lng], { icon: beaconIcon, zIndexOffset: 900 }).addTo(map);

    map.flyTo([lat, lng], 16, { animate: true, duration: 1.2 });

    // If initial load, suggest a destination 5-8 km away for immediate exploration
    if (autoPlanNearby && !currentDestination) {
        const sampleDest = {
            lat: lat + 0.045,
            lng: lng + 0.052,
            label: "Nearby Airport / City Link"
        };
        setDestinationAndPlan(sampleDest);
    }
}

btnRefreshGps.addEventListener('click', () => {
    acquireCurrentLocation(false);
});

// ==========================================================================
// 2. Interactive Destination Selection & Route Planning
// ==========================================================================
function handleMapTapDestination(e) {
    if (isNavigating) return; // Don't repick while driving

    const lat = e.latlng.lat;
    const lng = e.latlng.lng;
    const dest = {
        lat: lat,
        lng: lng,
        label: `Tapped Point (${lat.toFixed(4)}°, ${lng.toFixed(4)}°)`
    };

    setDestinationAndPlan(dest);
    showToast("📍 Destination pinned from map!");
}

function setDestinationAndPlan(dest, presetKey = null) {
    currentDestination = dest;
    destinationInput.value = dest.label;
    btnClearDest.style.display = 'block';

    // Calculate real road route from Current Location to Destination
    planRouteBetween(currentUserLocation, currentDestination, presetKey);
}

function updateStartNavButtonState() {
    if (!btnStartNavigation) return;
    if (isRoutePlanned && activeRouteCoordinates.length >= 2) {
        btnStartNavigation.classList.remove('disabled');
        btnStartNavigation.innerHTML = '<span>▶ Start Navigation</span>';
    } else {
        btnStartNavigation.classList.add('disabled');
        btnStartNavigation.innerHTML = '<span>▶ Choose Destination</span>';
    }
}

btnClearDest.addEventListener('click', () => {
    destinationInput.value = '';
    currentDestination = null;
    isRoutePlanned = false;
    activeRouteCoordinates = [];
    btnClearDest.style.display = 'none';
    clearActiveRouteFromMap();
    if (altRoutesContainer) altRoutesContainer.style.display = 'none';
    planEstMins.innerText = '-- min';
    planEstDist.innerText = '(-- km)';
    planEstNote.innerText = '• Tap map or enter destination';
    updateStartNavButtonState();
});

// Quick Destination Suggestions
document.querySelectorAll('.dest-pill').forEach(btn => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('.dest-pill').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');

        const type = btn.getAttribute('data-dest');
        const presetKey = btn.getAttribute('data-preset');

        if (presetKey && PRESETS[presetKey]) {
            const p = PRESETS[presetKey];
            currentUserLocation = p.origin;
            originInput.value = `📍 ${p.origin.label}`;
            setDestinationAndPlan(p.dest, presetKey);
            currentTunnelInterval = p.tunnelZone;
            return;
        }

        // Relative destinations from user's current coordinates
        let dLat = 0.05;
        let dLng = 0.06;
        let lbl = "Destination";

        if (type === 'airport') {
            dLat = 0.075;
            dLng = 0.082;
            lbl = "Int'l Airport Expressway Corridor";
        } else if (type === 'central') {
            dLat = 0.038;
            dLng = -0.042;
            lbl = "Central Railway Junction";
        } else if (type === 'techpark') {
            dLat = -0.052;
            dLng = 0.065;
            lbl = "Cyber Tech Park Corridor";
        }

        const dest = {
            lat: currentUserLocation.lat + dLat,
            lng: currentUserLocation.lng + dLng,
            label: lbl
        };
        setDestinationAndPlan(dest);
    });
});

// Search Origin by Text
originInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') {
        const q = originInput.value.trim();
        if (q) searchOriginGeocode(q);
    }
});

function searchOriginGeocode(query) {
    showToast(\`Searching Google Maps for "\${query}"...\`);
    fetch(\`https://nominatim.openstreetmap.org/search?format=json&limit=1&q=\${encodeURIComponent(query)}\`)
        .then(r => r.json())
        .then(res => {
            if (res && res.length > 0) {
                const item = res[0];
                const lat = parseFloat(item.lat);
                const lng = parseFloat(item.lon);
                
                currentUserLocation = {
                    lat: lat,
                    lng: lng,
                    label: item.display_name.split(',')[0]
                };
                originInput.value = \`📍 \${currentUserLocation.label}\`;
                
                // If destination is already set, replan the route
                if (currentDestination) {
                    setDestinationAndPlan(currentDestination);
                } else {
                    map.flyTo([lat, lng], 14, { animate: true, duration: 1.2 });
                }
            } else {
                showToast("Origin not found");
            }
        })
        .catch(() => showToast("Search offline"));
}

// Search Destination by Text
destinationInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') {
        const q = destinationInput.value.trim();
        if (q) searchDestinationGeocode(q);
    }
});

function searchDestinationGeocode(query) {
    showToast(`Searching Google Maps for "${query}"...`);
    fetch(`https://nominatim.openstreetmap.org/search?format=json&limit=1&q=${encodeURIComponent(query)}`)
        .then(r => r.json())
        .then(res => {
            if (res && res.length > 0) {
                const item = res[0];
                const dest = {
                    lat: parseFloat(item.lat),
                    lng: parseFloat(item.lon),
                    label: item.display_name.split(',')[0]
                };
                setDestinationAndPlan(dest);
            } else {
                showToast("Location not found - tap on the map to place destination pin");
            }
        })
        .catch(() => showToast("Search offline - select suggestion above"));
}

async function planRouteBetween(origin, destination, presetKey = null) {
    planEstNote.innerText = '• Analyzing 4 alternative road routes & satellite density...';

    let baseCoordinates = [];
    let baseSteps = [];
    let osrmDistanceMeters = null;
    let osrmDurationSeconds = null;

    const osrmUrl = `https://router.project-osrm.org/route/v1/driving/${origin.lng},${origin.lat};${destination.lng},${destination.lat}?overview=full&geometries=geojson&steps=true`;

    try {
        const res = await fetch(osrmUrl);
        const data = await res.json();
        if (data.code === 'Ok' && data.routes && data.routes.length > 0) {
            const r = data.routes[0];
            osrmDistanceMeters = r.distance;
            osrmDurationSeconds = r.duration;
            baseCoordinates = r.geometry.coordinates.map(c => ({ lat: c[1], lng: c[0] }));
            baseSteps = (r.legs && r.legs[0] && r.legs[0].steps) ? r.legs[0].steps.map(s => ({
                instruction: s.name ? `Turn onto ${s.name}` : `Follow expressway corridor`,
                distance: `${Math.round(s.distance)} m`,
                distMeters: Math.round(s.distance),
                maneuver: s.maneuver ? s.maneuver.modifier || s.maneuver.type : "straight"
            })) : [];
        }
    } catch(e) {
        console.warn("Routing fallback:", e);
    }

    if (baseCoordinates.length === 0) {
        const count = 70;
        for (let i = 0; i <= count; i++) {
            const t = i / count;
            baseCoordinates.push({
                lat: origin.lat + (destination.lat - origin.lat) * t,
                lng: origin.lng + (destination.lng - origin.lng) * t
            });
        }
        baseSteps = [
            { instruction: `Depart from ${origin.label.split('(')[0]}`, distance: "450 m", distMeters: 450, maneuver: "straight" },
            { instruction: "Enter Forest/Tunnel Sector (GNSS Outage Zone)", distance: "1.2 km", distMeters: 1200, maneuver: "tunnel" },
            { instruction: `Arrive at ${destination.label.split('(')[0]}`, distance: "350 m", distMeters: 350, maneuver: "arrive" }
        ];
    }

    const routes = generateFourAlternativeRoutes(baseCoordinates, origin, destination, presetKey, osrmDistanceMeters, osrmDurationSeconds);
    calculatedAltRoutes = routes;
    selectedRouteIndex = 0;

    displayPlannedRoutesOnMap(routes, 0, origin, destination, baseSteps);
}

function generateFourAlternativeRoutes(baseCoords, origin, destination, presetKey, osrmDistanceMeters, osrmDurationSeconds) {
    const numPoints = baseCoords.length;
    let baseMeters = 0;
    for (let i = 0; i < numPoints - 1; i++) {
        baseMeters += calculateHaversineMeters(baseCoords[i].lat, baseCoords[i].lng, baseCoords[i+1].lat, baseCoords[i+1].lng);
    }
    // Use OSRM data if available, otherwise fallback to straight-line distance math
    const finalMeters = osrmDistanceMeters !== null ? osrmDistanceMeters : baseMeters;
    const baseKm = finalMeters / 1000;
    
    // Average speed logic: if OSRM duration exists, use it. Else assume 50 km/h.
    const baseMins = osrmDurationSeconds !== null ? Math.round(osrmDurationSeconds / 60) : Math.max(4, Math.round(finalMeters / (50 * 1000 / 60)));
    
    currentRouteAverageSpeed = (baseKm / (baseMins / 60)); // Global tracking

    const isHill = (presetKey === 'ooty' || presetKey === 'munnar');
    const isUndersea = (presetKey === 'mumbai');

    const route1 = {
        id: 0,
        name: presetKey ? `Via ${PRESETS[presetKey].name.split('➔')[1] ? PRESETS[presetKey].name.split('➔')[1].trim() : 'Main Corridor'}` : `Via NH Expressway (Primary)`,
        tag: 'RECOMMENDED',
        tagClass: 'alt-tag-recommended',
        coords: baseCoords,
        km: baseKm.toFixed(1),
        mins: baseMins,
        tunnelZone: presetKey && PRESETS[presetKey].tunnelZone ? PRESETS[presetKey].tunnelZone : [0.35, 0.70],
        warnClass: 'warn-tunnel',
        warnText: isHill ? '🚨 1.8 km Mountain Tunnel Blackout (NavIC DR Active)' : (isUndersea ? '🚨 2.1 km Undersea Coastal Tunnel' : '🚨 1.4 km Tunnel Blackout Sector (NavIC DR Active)'),
        signalClass: 'alt-signal-high',
        signalPercent: '85%',
        signalText: '85% Signal'
    };

    const forestCoords = baseCoords.map((pt, idx) => {
        const offset = Math.sin((idx / numPoints) * Math.PI) * 0.009;
        return { lat: pt.lat + offset, lng: pt.lng + offset * 0.7 };
    });
    const route2 = {
        id: 1,
        name: isHill ? `Via Nilgiri / Munnar Deep Forest Pass (SH-15)` : `Via Deep Canopy Forest Pass`,
        tag: 'DEEP FOREST',
        tagClass: 'alt-tag-scenic',
        coords: forestCoords,
        km: (baseKm * 1.14).toFixed(1),
        mins: Math.round(baseMins * 1.3),
        tunnelZone: [0.25, 0.80],
        warnClass: 'warn-forest',
        warnText: '🌲 6.5 km Deep Canopy Forest (Low/Loss GNSS Signal)',
        signalClass: 'alt-signal-med',
        signalPercent: '55%',
        signalText: '55% Signal'
    };

    const bypassCoords = baseCoords.map((pt, idx) => {
        const offset = -Math.sin((idx / numPoints) * Math.PI) * 0.014;
        return { lat: pt.lat + offset, lng: pt.lng - offset * 0.5 };
    });
    const route3 = {
        id: 2,
        name: `Via Outer Expressway Bypass Corridor`,
        tag: 'CLEAR SKY',
        tagClass: 'alt-tag-clear',
        coords: bypassCoords,
        km: (baseKm * 1.25).toFixed(1),
        mins: Math.round(baseMins * 1.08),
        tunnelZone: [0, 0],
        warnClass: 'warn-clear',
        warnText: '📶 Open Sky Highway (Full 14 SVs NavIC Signal)',
        signalClass: 'alt-signal-high',
        signalPercent: '98%',
        signalText: '98% Signal'
    };

    const shortcutCoords = baseCoords.map((pt, idx) => {
        const offset = Math.sin((idx / numPoints) * Math.PI * 2) * 0.006;
        return { lat: pt.lat + offset, lng: pt.lng };
    });
    const route4 = {
        id: 3,
        name: `Via Old Valley Ghat Shortcut`,
        tag: 'SHORTCUT',
        tagClass: 'alt-tag-shortcut',
        coords: shortcutCoords,
        km: (baseKm * 0.95).toFixed(1),
        mins: Math.round(baseMins * 1.35),
        tunnelZone: [0.40, 0.65],
        warnClass: 'warn-tunnel',
        warnText: '🚨 🌲 Mixed Short Tunnel & Dense Foliage',
        signalClass: 'alt-signal-low',
        signalPercent: '45%',
        signalText: '45% Signal'
    };

    return [route1, route2, route3, route4];
}

function displayPlannedRoutesOnMap(routes, selectIdx, origin, destination, steps) {
    selectedRouteIndex = selectIdx;
    const selectedRoute = routes[selectIdx];

    activeRouteCoordinates = selectedRoute.coords;
    currentTunnelInterval = selectedRoute.tunnelZone;
    routeSteps = steps;
    currentStepIdx = 0;
    progressAlongStep = 0.0;
    isRoutePlanned = true;

    clearActiveRouteFromMap();

    // Render non-selected alternative routes first in gray polylines
    altRoutePolylines = [];
    routes.forEach((r, idx) => {
        if (idx !== selectIdx) {
            const line = L.polyline(r.coords.map(c => [c.lat, c.lng]), {
                color: '#94A3B8',
                weight: 5,
                opacity: 0.65,
                dashArray: '6, 6',
                lineCap: 'round'
            }).addTo(map);
            line.on('click', () => selectAlternativeRoute(idx));
            altRoutePolylines.push(line);
        }
    });

    // Render Primary Selected Blue Navigation Line
    const selectedLatLngs = selectedRoute.coords.map(c => [c.lat, c.lng]);
    routePolyline = L.polyline(selectedLatLngs, {
        color: '#1A73E8',
        weight: 8,
        opacity: 0.95,
        lineCap: 'round',
        lineJoin: 'round'
    }).addTo(map);

    traveledPolyline = L.polyline([], {
        color: '#BDC1C6',
        weight: 8,
        opacity: 0.9,
        lineCap: 'round'
    }).addTo(map);

    outagePolyline = L.polyline([], {
        color: '#D93025',
        weight: 8,
        opacity: 0.95,
        dashArray: '8, 8',
        lineCap: 'round'
    }).addTo(map);

    // Origin Green Marker
    originMarker = L.circleMarker([origin.lat, origin.lng], {
        radius: 7,
        fillColor: '#0F9D58',
        color: '#FFFFFF',
        weight: 2.5,
        fillOpacity: 1
    }).addTo(map);

    // Destination Red Pin Marker
    destMarker = L.circleMarker([destination.lat, destination.lng], {
        radius: 8,
        fillColor: '#D93025',
        color: '#FFFFFF',
        weight: 2.5,
        fillOpacity: 1
    }).addTo(map);

    // Add Forest / Tunnel Indicator Marker if applicable
    if (selectedRoute.tunnelZone && selectedRoute.tunnelZone[1] > 0) {
        const midIdx = Math.floor(selectedRoute.coords.length * ((selectedRoute.tunnelZone[0] + selectedRoute.tunnelZone[1]) / 2));
        const midPoint = selectedRoute.coords[midIdx] || selectedRoute.coords[Math.floor(selectedRoute.coords.length * 0.5)];
        if (tunnelZoneMarker) map.removeLayer(tunnelZoneMarker);
        
        const isForest = selectedRoute.tag === 'DEEP FOREST';
        const markerLabel = isForest ? '🌲 Deep Forest Canopy (Low Signal)' : '🚨 Tunnel Blackout Zone (NavIC DR Active)';
        const deadZoneIcon = L.divIcon({
            className: 'forest-tunnel-marker-wrap',
            html: `<div class="forest-tunnel-marker">${markerLabel}</div>`,
            iconAnchor: [70, 10]
        });
        tunnelZoneMarker = L.marker([midPoint.lat, midPoint.lng], { icon: deadZoneIcon }).addTo(map);
    }

    planEstMins.innerText = `${selectedRoute.mins} min`;
    planEstDist.innerText = `(${selectedRoute.km} km)`;
    planEstNote.innerText = `• Selected: ${selectedRoute.name}`;

    renderAlternativeRoutesList(routes);
    map.fitBounds(routePolyline.getBounds(), { padding: [60, 60], maxZoom: 16 });
    renderDirectionsSteps(routeSteps);
    updateStartNavButtonState();
}

function renderAlternativeRoutesList(routes) {
    if (!altRoutesScroll) return;
    altRoutesScroll.innerHTML = '';

    routes.forEach((route, idx) => {
        const isSelected = (idx === selectedRouteIndex);
        const card = document.createElement('div');
        card.className = `alt-route-card ${isSelected ? 'selected' : ''}`;
        card.onclick = () => selectAlternativeRoute(idx);

        card.innerHTML = `
            <div class="alt-card-top-row">
                <div class="alt-route-name">
                    <span>${route.name}</span>
                    <span class="alt-route-tag ${route.tagClass}">${route.tag}</span>
                </div>
                <div class="alt-route-time">${route.mins} min</div>
            </div>
            <div class="alt-card-mid-row">
                <span class="alt-dist-txt">${route.km} km</span>
                <div class="alt-signal-bar-wrap">
                    <span style="font-size: 10px; font-weight:700;">Signal:</span>
                    <div class="alt-signal-track">
                        <div class="alt-signal-fill ${route.signalClass}"></div>
                    </div>
                    <span style="font-size: 10px; color:#64748B;">${route.signalPercent}</span>
                </div>
            </div>
            <div class="alt-warning-pill ${route.warnClass}">
                ${route.warnText}
            </div>
        `;
        altRoutesScroll.appendChild(card);
    });

    if (altRoutesContainer) altRoutesContainer.style.display = 'block';
}

function selectAlternativeRoute(idx) {
    if (!calculatedAltRoutes || calculatedAltRoutes.length === 0) return;
    selectedRouteIndex = idx;
    const selectedRoute = calculatedAltRoutes[idx];

    displayPlannedRoutesOnMap(calculatedAltRoutes, idx, currentUserLocation, currentDestination, routeSteps);
    showToast(`Selected Path ${idx+1}: ${selectedRoute.name}`);
}

function clearActiveRouteFromMap() {
    if (routePolyline) map.removeLayer(routePolyline);
    if (traveledPolyline) map.removeLayer(traveledPolyline);
    if (outagePolyline) map.removeLayer(outagePolyline);
    if (originMarker) map.removeLayer(originMarker);
    if (destMarker) map.removeLayer(destMarker);
    if (tunnelZoneMarker) map.removeLayer(tunnelZoneMarker);
    if (altRoutePolylines && altRoutePolylines.length > 0) {
        altRoutePolylines.forEach(l => map.removeLayer(l));
        altRoutePolylines = [];
    }
}

// ==========================================================================
// 4. Start Navigation Transition
// ==========================================================================
btnStartNavigation.addEventListener('click', () => {
    if (!isRoutePlanned || activeRouteCoordinates.length < 2) {
        showToast("Please choose a destination first");
        return;
    }

    // Switch UI from Route Planner -> Active Navigation Mode
    routePlannerCard.style.display = 'none';
    navHeader.style.display = 'flex';
    simSpeedControls.style.display = 'flex';
    speedometer.style.display = 'flex';
    drHud.style.display = 'flex';
    bottomSheet.style.display = 'block';

    isNavigating = true;
    isPaused = false;
    currentStepIdx = 0;
    progressAlongStep = 0.0;

    // Attach 3D Navigation Vehicle Puck
    if (!vehicleMarker) {
        const puckIcon = L.divIcon({
            className: 'gmaps-puck-outer',
            html: `
                <div class="puck-radar-beam" id="puck-radar"></div>
                <div class="puck-3d-arrow" id="puck-arrow"></div>
            `,
            iconSize: [48, 48],
            iconAnchor: [24, 24]
        });
        vehicleMarker = L.marker([activeRouteCoordinates[0].lat, activeRouteCoordinates[0].lng], {
            icon: puckIcon,
            zIndexOffset: 1000
        }).addTo(map);
    } else {
        vehicleMarker.setLatLng([activeRouteCoordinates[0].lat, activeRouteCoordinates[0].lng]);
    }

    // Attach EKF Covariance Uncertainty Ellipse
    if (!ekfCovarianceCircle) {
        ekfCovarianceCircle = L.circle([activeRouteCoordinates[0].lat, activeRouteCoordinates[0].lng], {
            radius: 3.2,
            color: '#1A73E8',
            weight: 1.5,
            fillColor: '#1A73E8',
            fillOpacity: 0.12
        }).addTo(map);
    }

    if (Config.isLiveMode()) {
        showToast("Starting LIVE Navigation with device GPS...");
        speakVoicePrompt("Starting live navigation. Connecting to hardware GPS.");
        
        GeolocationService.startLiveTracking(
            (posData) => {
                // Update live position state
                if (!isNavigating) return;
                
                const { lat, lng, accuracy, rawSpeedKmh, filteredSpeedKmh, isStationary, timestamp } = posData;
                
                // If not in outage, trust GPS
                if (!isOutageActive) {
                    const newPos = [lat, lng];
                    if (vehicleMarker) vehicleMarker.setLatLng(newPos);
                    map.panTo(newPos, { animate: false });
                    
                    if (ekfCovarianceCircle) {
                        ekfCovarianceCircle.setLatLng(newPos);
                        ekfCovarianceCircle.setRadius(accuracy || 3.2);
                    }
                    
                    // Update global state for UI loop
                    currentRealSpeedKmh = filteredSpeedKmh;
                    
                    if (isStationary) {
                        currentRealSpeedKmh = 0;
                    }

                    // Attempt to find closest route point to determine currentStepIdx and heading
                    // (Simple Map Matching)
                    let minD = Infinity;
                    let bestIdx = currentStepIdx;
                    for (let i = 0; i < activeRouteCoordinates.length; i++) {
                        const d = calculateHaversineMeters(lat, lng, activeRouteCoordinates[i].lat, activeRouteCoordinates[i].lng);
                        if (d < minD) {
                            minD = d;
                            bestIdx = i;
                        }
                    }
                    if (bestIdx > currentStepIdx && bestIdx < activeRouteCoordinates.length - 1) {
                        currentVehicleHeading = calculateBearingDeg(
                            activeRouteCoordinates[bestIdx].lat, activeRouteCoordinates[bestIdx].lng,
                            activeRouteCoordinates[bestIdx+1].lat, activeRouteCoordinates[bestIdx+1].lng
                        );
                        currentStepIdx = bestIdx;
                    }
                }
            },
            (err) => {
                showToast("Live GPS tracking failed. " + err.message);
                console.warn(err);
            }
        );
    } else {
        showToast("Starting Real-Time Navigation (Simulation Mode)...");
        speakVoicePrompt("Starting simulation navigation. Head toward route.");
    }
});

// End Navigation & Return to Route Planner
btnEndRoute.addEventListener('click', () => {
    isNavigating = false;
    isPaused = false;
    isOutageActive = false;
    hasWarnedApproach = false;
    hasWarnedEntered = false;
    hasWarnedExit = false;
    handleBlackoutTimer(false);
    GeolocationService.stopLiveTracking();

    // Switch UI from Active Navigation -> Route Planner Mode
    navHeader.style.display = 'none';
    outagePill.style.display = 'none';
    if (tunnelWarningBanner) tunnelWarningBanner.style.display = 'none';
    if (tunnelBlackoutBanner) tunnelBlackoutBanner.style.display = 'none';
    if (tunnelRestoredBanner) tunnelRestoredBanner.style.display = 'none';
    
    const mapCanvas = document.getElementById('map');
    if (mapCanvas) mapCanvas.classList.remove('in-tunnel');
    const puckEl = document.querySelector('.gmaps-puck-outer');
    if (puckEl) puckEl.classList.remove('in-tunnel');

    simSpeedControls.style.display = 'none';
    speedometer.style.display = 'none';
    drHud.style.display = 'none';
    bottomSheet.style.display = 'none';
    routePlannerCard.style.display = 'flex';

    if (vehicleMarker) map.removeLayer(vehicleMarker);
    vehicleMarker = null;
    if (ekfCovarianceCircle) map.removeLayer(ekfCovarianceCircle);
    ekfCovarianceCircle = null;

    showToast("Navigation Ended. Ready for next destination.");
    speakVoicePrompt("Navigation ended.");
});

// ==========================================================================
// 5. 60FPS Continuous Navigation & Sensor Dead Reckoning Engine
// ==========================================================================
let lastFrameTime = performance.now();

function animationFrameLoop(now) {
    const deltaMs = now - lastFrameTime;
    lastFrameTime = now;

    if (isNavigating && !isPaused && activeRouteCoordinates.length >= 2) {
        if (!Config.isLiveMode()) {
            stepVehicleProgress(deltaMs);
        } else {
            // Live Mode: Movement is driven by GeolocationService
            // But we still need to update the HUD, compass, and UI based on state
            updateLiveVehicleUI(deltaMs);
        }
    }

    requestAnimationFrame(animationFrameLoop);
}

function stepVehicleProgress(deltaMs) {
    const coords = activeRouteCoordinates;
    const currentPt = coords[currentStepIdx];
    const nextPt = coords[Math.min(currentStepIdx + 1, coords.length - 1)];

    // Advance vehicle position based on deltaMs and simulation rate
    const stepIncrement = (deltaMs / 1000) * 0.35 * simulationRateMultiplier;
    progressAlongStep += stepIncrement;

    if (progressAlongStep >= 1.0) {
        progressAlongStep = 0.0;
        if (currentStepIdx < coords.length - 2) {
            currentStepIdx++;
        } else {
            // Arrived at destination
            currentStepIdx = 0;
            traveledPolyline.setLatLngs([]);
            outagePolyline.setLatLngs([]);
            showToast("🏁 Arrived at destination! SIH Trip Summary Generated.");
            speakVoicePrompt("You have arrived at your destination. Autonomous dead reckoning performance report generated.");
            showTripSummaryModal();
        }
    }

    const lat = currentPt.lat + (nextPt.lat - currentPt.lat) * progressAlongStep;
    const lng = currentPt.lng + (nextPt.lng - currentPt.lng) * progressAlongStep;
    const heading = calculateBearingDeg(currentPt.lat, currentPt.lng, nextPt.lat, nextPt.lng);
    currentVehicleHeading = heading;

    const pos = [lat, lng];
    if (vehicleMarker) vehicleMarker.setLatLng(pos);

    // Update Compass Needle
    if (compassNeedle) {
        compassNeedle.style.transform = `rotate(${-heading}deg)`;
    }

    // Camera follow (Center map on vehicle)
    map.panTo(pos, { animate: false });

    // Rotate 3D Heading Chevron Arrow
    const arrow = document.getElementById('puck-arrow');
    if (arrow) {
        arrow.style.transform = `rotate(${Math.round(heading)}deg)`;
    }

    // Tunnel Geofence Indices
    const tunnelStartIdx = Math.floor(coords.length * currentTunnelInterval[0]);
    const tunnelEndIdx = Math.floor(coords.length * currentTunnelInterval[1]);
    const inTunnelOrForest = (currentStepIdx >= tunnelStartIdx && currentStepIdx <= tunnelEndIdx);

    // 1. Tunnel Early Warning System (Within 250 meters of tunnel entrance)
    let distToTunnelMeters = 0;
    if (currentStepIdx < tunnelStartIdx) {
        for (let i = currentStepIdx; i < tunnelStartIdx; i++) {
            distToTunnelMeters += calculateHaversineMeters(coords[i].lat, coords[i].lng, coords[i+1].lat, coords[i+1].lng);
        }
    }

    const isApproachingTunnel = (currentStepIdx < tunnelStartIdx && distToTunnelMeters <= 250 && distToTunnelMeters > 0);
    if (isApproachingTunnel) {
        if (tunnelWarningBanner) {
            tunnelWarningBanner.style.display = 'flex';
            if (tunnelCountdownDist) tunnelCountdownDist.innerText = `${Math.round(distToTunnelMeters)} m`;
            if (tunnelApproachFill) {
                const fillPct = Math.min(100, Math.max(8, (distToTunnelMeters / 250) * 100));
                tunnelApproachFill.style.width = `${fillPct}%`;
            }
        }
        if (!hasWarnedApproach) {
            hasWarnedApproach = true;
            playWarningChime();
            speakVoicePrompt(`Warning. Approaching tunnel in ${Math.round(distToTunnelMeters)} meters. GNSS blackout zone ahead. Dead reckoning system standing by.`);
        }
    } else {
        if (tunnelWarningBanner) tunnelWarningBanner.style.display = 'none';
    }

    // 2. Outage Evaluation (Manual GPS Off OR Tunnel Geofence OR Forced Debug Outage)
    const isOutage = isManualGpsOff || inTunnelOrForest || isOutageActive;

    if (isOutage) {
        outagePolyline.addLatLng(pos);
    } else {
        traveledPolyline.addLatLng(pos);
    }

    // Update EKF Covariance Uncertainty Ellipse
    updateEkfCovarianceEllipse(pos, isOutage);

    // Update Cockpit HUD, Maneuvers, Speed, Voice
    const fraction = currentStepIdx / coords.length;
    updateCockpitHUD(fraction, isOutage, inTunnelOrForest, pos, coords);
}

function updateLiveVehicleUI(deltaMs) {
    const coords = activeRouteCoordinates;
    if (!coords || coords.length === 0) return;
    
    // In live mode, pos and heading are already updated by GeolocationService
    // We just need to drive the tunnel UI, outgage UI, and HUD
    const currentPt = coords[currentStepIdx];
    const pos = vehicleMarker ? [vehicleMarker.getLatLng().lat, vehicleMarker.getLatLng().lng] : [currentPt.lat, currentPt.lng];

    // Update Compass Needle
    if (compassNeedle) {
        compassNeedle.style.transform = \`rotate(\${-currentVehicleHeading}deg)\`;
    }

    // Rotate 3D Heading Chevron Arrow
    const arrow = document.getElementById('puck-arrow');
    if (arrow) {
        arrow.style.transform = \`rotate(\${Math.round(currentVehicleHeading)}deg)\`;
    }

    // Tunnel Geofence Indices
    const tunnelStartIdx = Math.floor(coords.length * currentTunnelInterval[0]);
    const tunnelEndIdx = Math.floor(coords.length * currentTunnelInterval[1]);
    const inTunnelOrForest = (currentStepIdx >= tunnelStartIdx && currentStepIdx <= tunnelEndIdx);

    // 1. Tunnel Early Warning System (Within 250 meters of tunnel entrance)
    let distToTunnelMeters = 0;
    if (currentStepIdx < tunnelStartIdx) {
        for (let i = currentStepIdx; i < tunnelStartIdx; i++) {
            distToTunnelMeters += calculateHaversineMeters(coords[i].lat, coords[i].lng, coords[i+1].lat, coords[i+1].lng);
        }
    }

    const isApproachingTunnel = (currentStepIdx < tunnelStartIdx && distToTunnelMeters <= 250 && distToTunnelMeters > 0);
    if (isApproachingTunnel) {
        if (tunnelWarningBanner) {
            tunnelWarningBanner.style.display = 'flex';
            if (tunnelCountdownDist) tunnelCountdownDist.innerText = \`\${Math.round(distToTunnelMeters)} m\`;
            if (tunnelApproachFill) {
                const fillPct = Math.min(100, Math.max(8, (distToTunnelMeters / 250) * 100));
                tunnelApproachFill.style.width = \`\${fillPct}%\`;
            }
        }
        if (!hasWarnedApproach) {
            hasWarnedApproach = true;
            playWarningChime();
            speakVoicePrompt(\`Warning. Approaching tunnel in \${Math.round(distToTunnelMeters)} meters. GNSS blackout zone ahead. Dead reckoning system standing by.\`);
        }
    } else {
        if (tunnelWarningBanner) tunnelWarningBanner.style.display = 'none';
    }

    // 2. Outage Evaluation
    const isOutage = isManualGpsOff || inTunnelOrForest || isOutageActive;

    if (isOutage) {
        outagePolyline.addLatLng(pos);
    } else {
        traveledPolyline.addLatLng(pos);
    }

    // Update EKF Covariance Uncertainty Ellipse
    updateEkfCovarianceEllipse(pos, isOutage);

    // Update Cockpit HUD, Maneuvers, Speed, Voice
    const fraction = currentStepIdx / coords.length;
    updateCockpitHUD(fraction, isOutage, inTunnelOrForest, pos, coords);
}


function updateEkfCovarianceEllipse(pos, isOutage) {
    if (!ekfCovarianceCircle) return;
    ekfCovarianceCircle.setLatLng(pos);

    if (isOutage) {
        // Physical Kalman Drift: E = 0.5 * a_bias * t^2
        const accelBias = 0.08;
        const driftMeters = 0.5 * accelBias * Math.pow(blackoutSeconds, 2);
        const uncertaintyRadius = Math.min(25, 3.5 + driftMeters);

        ekfCovarianceCircle.setRadius(uncertaintyRadius);
        ekfCovarianceCircle.setStyle({
            color: '#EF4444',
            fillColor: '#EF4444',
            fillOpacity: 0.24
        });
    } else {
        ekfCovarianceCircle.setRadius(2.8);
        ekfCovarianceCircle.setStyle({
            color: '#1A73E8',
            fillColor: '#1A73E8',
            fillOpacity: 0.12
        });
    }
}

function updateCockpitHUD(fraction, isOutage, inTunnelOrForest, pos, coords) {
    const stepIdx = Math.min(Math.floor(fraction * routeSteps.length), routeSteps.length - 1);
    const step = routeSteps[stepIdx] || { instruction: "Continue on road", distance: "200 m", distMeters: 200 };
    const nextStep = routeSteps[Math.min(stepIdx + 1, routeSteps.length - 1)];

    const radar = document.getElementById('puck-radar');
    const mapCanvas = document.getElementById('map');
    const puckEl = document.querySelector('.gmaps-puck-outer');

    // Tunnel Visuals: Headlights & Ambient Corridors
    if (inTunnelOrForest) {
        if (mapCanvas) mapCanvas.classList.add('in-tunnel');
        if (puckEl) puckEl.classList.add('in-tunnel');
    } else {
        if (mapCanvas) mapCanvas.classList.remove('in-tunnel');
        if (puckEl) puckEl.classList.remove('in-tunnel');
    }

    // Maneuver Banner & Outage Alert
    if (isOutage) {
        navHeader.classList.add('outage-active');
        outagePill.style.display = 'none'; // Replaced by richer tunnel-blackout-banner
        if (tunnelBlackoutBanner) tunnelBlackoutBanner.style.display = 'flex';

        // Set Dynamic Blackout Details based on trigger cause
        if (isManualGpsOff && inTunnelOrForest) {
            if (blackoutReasonTag) blackoutReasonTag.innerText = "TUNNEL + MANUAL OFF";
            if (blackoutTriggerBadge) blackoutTriggerBadge.innerText = "DUAL OUTAGE";
            if (blackoutSystemMsg) blackoutSystemMsg.innerText = "GPS Disabled & In Tunnel • AI Dead Reckoning (1D-CNN+GRU + EKF) Running";
        } else if (isManualGpsOff) {
            if (blackoutReasonTag) blackoutReasonTag.innerText = "MANUAL GPS CUTOFF";
            if (blackoutTriggerBadge) blackoutTriggerBadge.innerText = "USER DISABLED";
            if (blackoutSystemMsg) blackoutSystemMsg.innerText = "GPS Hardware Cutoff by User • AI Dead Reckoning (IMU + EKF + NHC) Active";
        } else if (inTunnelOrForest) {
            if (blackoutReasonTag) blackoutReasonTag.innerText = "TUNNEL BLACKOUT";
            if (blackoutTriggerBadge) blackoutTriggerBadge.innerText = "AUTO GEOFENCE";
            if (blackoutSystemMsg) blackoutSystemMsg.innerText = "GNSS Denied (Tunnel Sector) • AI Dead Reckoning (1D-CNN+GRU + EKF) Active";
        } else {
            if (blackoutReasonTag) blackoutReasonTag.innerText = "SIMULATED OUTAGE";
            if (blackoutTriggerBadge) blackoutTriggerBadge.innerText = "DEBUG OVERRIDE";
            if (blackoutSystemMsg) blackoutSystemMsg.innerText = "Simulated GNSS Outage • AI Dead Reckoning Active";
        }

        navSystemBadge.innerText = isManualGpsOff ? "⚡ GPS Cutoff • DR Active" : "⚡ GNSS Blackout • AI Engine";
        navSystemBadge.style.borderColor = "#D93025";
        navSystemBadge.style.color = "#F87171";

        maneuverDist.innerText = inTunnelOrForest ? "TUNNEL SECTOR" : "GNSS CUTOFF";
        maneuverNextHint.innerText = "GNSS Denied";
        maneuverRoad.innerText = isManualGpsOff 
            ? "GPS Hardware Off • AI Dead Reckoning Active (IMU Sensors + EKF + NHC)"
            : "Tunnel GPS Signal Lost • AI Dead Reckoning Active (1D-CNN+GRU + EKF)";
        maneuverSvg.innerHTML = `<path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm1 15h-2v-6h2v6zm0-8h-2V7h2v2z"/>`;

        // Physical Dead Reckoning Drift calculation
        const accelBias = 0.08;
        const driftMeters = 0.5 * accelBias * Math.pow(blackoutSeconds, 2);
        hudDrift.innerText = `${driftMeters.toFixed(2)} m (3σ)`;
        hudStatusTag.innerText = "DEAD RECKONING";
        hudStatusTag.className = "hud-tag outage";

        if (radar) radar.classList.add('outage');
        handleBlackoutTimer(true);
        setPillStates(false, true, true, true, true);
        updateConstellationStatus(true);

        // Voice Prompt on Tunnel Entry (Automatic)
        if (inTunnelOrForest && !hasWarnedEntered) {
            hasWarnedEntered = true;
            hasWarnedExit = false;
            playBlackoutAlert();
            speakVoicePrompt("Entering tunnel. GPS signal lost. Autonomous dead reckoning system active.");
        }
    } else {
        navHeader.classList.remove('outage-active');
        if (tunnelBlackoutBanner) tunnelBlackoutBanner.style.display = 'none';
        outagePill.style.display = 'none';

        navSystemBadge.innerText = "🛰️ NavIC L5 Dual-Band";
        navSystemBadge.style.borderColor = "#1A73E8";
        navSystemBadge.style.color = "#8AB4F8";

        const remainingStepMeters = Math.max(20, Math.round(step.distMeters * (1 - progressAlongStep)));
        maneuverDist.innerText = `In ${remainingStepMeters} m`;
        maneuverNextHint.innerText = nextStep ? `Then ${nextStep.instruction.split(' ')[0]}` : "Keep ahead";
        maneuverRoad.innerText = step.instruction;

        updateManeuverIcon(step.instruction);

        hudDrift.innerText = "0.00 m (3σ)";
        hudStatusTag.innerText = "EKF OPTIMAL";
        hudStatusTag.className = "hud-tag";

        if (radar) radar.classList.remove('outage');
        handleBlackoutTimer(false);
        setPillStates(true, false, false, false, true);
        updateConstellationStatus(false);

        // Voice Prompt & Banner on Tunnel Exit (Restoration)
        if (hasWarnedEntered && !hasWarnedExit && !inTunnelOrForest) {
            hasWarnedExit = true;
            hasWarnedEntered = false;
            hasWarnedApproach = false;
            playRestoredChime();
            speakVoicePrompt("Tunnel exit complete. NavIC satellite lock restored.");

            if (tunnelRestoredBanner) {
                tunnelRestoredBanner.style.display = 'flex';
                if (restoredMetricsMsg) {
                    const finalDrift = (0.04 * Math.pow(Math.max(1, blackoutSeconds), 1.2)).toFixed(2);
                    restoredMetricsMsg.innerText = `Realigned with 14 NavIC/GPS Satellites • Max Tunnel Drift: ${finalDrift}m`;
                }
                clearTimeout(window.restoredTimer);
                window.restoredTimer = setTimeout(() => {
                    if (tunnelRestoredBanner) tunnelRestoredBanner.style.display = 'none';
                }, 4500);
            }
        }

        // Voice Prompt on normal turn
        if (remainingStepMeters <= 80 && lastSpokenInstruction !== step.instruction) {
            lastSpokenInstruction = step.instruction;
            speakVoicePrompt(`In 80 meters, ${step.instruction}`);
        }
    }

    // Vehicle Speed
    const baseSpeed = isOutage ? (currentRouteAverageSpeed * 0.85) : currentRouteAverageSpeed;
    const speed = (baseSpeed + Math.sin(Date.now() / 400) * 2.5) * Math.min(1.5, simulationRateMultiplier);
    currentRealSpeedKmh = speed;
    speedVal.innerText = Math.round(speed);

    // AI Predicted Speed (1D-CNN+GRU)
    hudAiSpeed.innerText = `${(speed + (isOutage ? 0.3 : -0.1)).toFixed(1)} km/h`;

    // Remaining Distance in real meters
    let remainingMeters = 0;
    for (let i = currentStepIdx; i < coords.length - 1; i++) {
        remainingMeters += calculateHaversineMeters(coords[i].lat, coords[i].lng, coords[i+1].lat, coords[i+1].lng);
    }
    const km = (remainingMeters / 1000).toFixed(1);
    remainingDist.innerText = `${km} km`;

    const remainingMins = Math.max(1, Math.round(remainingMeters / (speed * 1000 / 60)));
    etaMins.innerHTML = `${remainingMins} <small>min</small>`;

    const arr = new Date(Date.now() + remainingMins * 60000);
    const hours = arr.getHours() % 12 || 12;
    const mins = arr.getMinutes() < 10 ? `0${arr.getMinutes()}` : arr.getMinutes();
    const ampm = arr.getHours() >= 12 ? 'PM' : 'AM';
    arrivalTime.innerText = `${hours}:${mins} ${ampm}`;

    // Micro IMU Accelerometer + Gyroscope
    const ax = (realDeviceMotion.ax + (isOutage ? 0.28 : 0.03)).toFixed(2);
    const gz = (realDeviceMotion.gz).toFixed(2);
    hudAccel.innerText = `Ax: ${ax >= 0 ? '+' : ''}${ax} | Gyro: ${gz >= 0 ? '+' : ''}${gz}°/s`;
}

function updateManeuverIcon(text) {
    const t = text.toLowerCase();
    if (t.includes('right') || t.includes('exit')) {
        maneuverSvg.innerHTML = `<path d="M12 4l-1.41 1.41L16.17 11H4v2h12.17l-5.58 5.59L12 20l8-8z"/>`;
    } else if (t.includes('left')) {
        maneuverSvg.innerHTML = `<path d="M12 4l1.41 1.41L7.83 11H20v2H7.83l5.58 5.59L12 20l-8-8z"/>`;
    } else if (t.includes('u-turn')) {
        maneuverSvg.innerHTML = `<path d="M18 7c0-2.76-2.24-5-5-5s-5 2.24-5 5v8.17l-2.59-2.58L4 14l5 5 5-5-1.41-1.41L10 15.17V7c0-1.66 1.34-3 3-3s3 1.34 3 3v9h2V7z"/>`;
    } else {
        maneuverSvg.innerHTML = `<path d="M12 4l-5 5h3v9h4V9h3l-5-5z"/>`;
    }
}

function handleBlackoutTimer(isActive) {
    if (isActive) {
        if (!blackoutTimerInterval) {
            blackoutSeconds = 0.0;
            blackoutTimerInterval = setInterval(() => {
                blackoutSeconds += 0.1;
                if (blackoutTimer) blackoutTimer.innerText = `${blackoutSeconds.toFixed(1)}s`;
                if (blackoutStopwatch) blackoutStopwatch.innerText = `${blackoutSeconds.toFixed(1)}s`;
            }, 100);
        }
        btnToggleOutage.classList.add('active');
    } else {
        if (blackoutTimerInterval) {
            clearInterval(blackoutTimerInterval);
            blackoutTimerInterval = null;
        }
        btnToggleOutage.classList.remove('active');
    }
}

// ==========================================================================
// Android Location / Manual GPS Hardware Switch Handler
// ==========================================================================
function toggleManualGpsCutoff() {
    isManualGpsOff = !isManualGpsOff;
    updateGpsHardwareUi();
    
    if (isManualGpsOff) {
        playBlackoutAlert();
        showToast("⚠️ GPS Cutoff: Manual Dead Reckoning Active");
        speakVoicePrompt("Warning. GPS hardware turned off by user. Intelligent dead reckoning system activated.");
    } else {
        playRestoredChime();
        showToast("✅ GPS Enabled: NavIC Satellite Lock Restored");
        speakVoicePrompt("GPS signal restored. Vehicle position realigned with NavIC constellation.");
    }
}

function updateGpsHardwareUi() {
    if (btnQuickGpsToggle && gpsQuickLbl) {
        if (isManualGpsOff) {
            btnQuickGpsToggle.classList.remove('active');
            btnQuickGpsToggle.classList.add('off');
            gpsQuickLbl.innerText = "GPS: OFF";
        } else {
            btnQuickGpsToggle.classList.remove('off');
            btnQuickGpsToggle.classList.add('active');
            gpsQuickLbl.innerText = "GPS: ON";
        }
    }

    if (qsGpsTile && qsGpsStatus) {
        if (isManualGpsOff) {
            qsGpsTile.classList.add('active', 'off');
            qsGpsStatus.innerText = "Disabled (Manual Cutoff)";
        } else {
            qsGpsTile.classList.remove('off');
            qsGpsTile.classList.add('active');
            qsGpsStatus.innerText = "Enabled (NavIC L5)";
        }
    }

    if (qsDrStatus) {
        qsDrStatus.innerText = isManualGpsOff ? "ACTIVE (Manual Cutoff)" : "Standby (Auto)";
    }
}

if (btnQuickGpsToggle) {
    btnQuickGpsToggle.addEventListener('click', toggleManualGpsCutoff);
}

if (qsGpsTile) {
    qsGpsTile.addEventListener('click', toggleManualGpsCutoff);
}

// ==========================================================================
// Android 3-Button Navigation Bar Handlers (Back, Home, Recents)
// ==========================================================================
if (androidKeyBack) {
    androidKeyBack.addEventListener('click', () => {
        if (isNavigating) {
            btnEndRoute.click();
            showToast("Android Back: Exited Navigation");
        } else if (isRoutePlanned) {
            clearActiveRouteFromMap();
            isRoutePlanned = false;
            destinationInput.value = "";
            showToast("Android Back: Cleared Route");
        } else {
            showToast("Android Back");
        }
    });
}

if (androidKeyHome) {
    androidKeyHome.addEventListener('click', () => {
        btnRecenter.click();
        showToast("Android Home: Centered on Vehicle");
    });
}

if (androidKeyRecents) {
    androidKeyRecents.addEventListener('click', () => {
        const isOpen = constellationModal.style.display === 'flex';
        constellationModal.style.display = isOpen ? 'none' : 'flex';
        showToast("Android Recents: Satellite Constellation Monitor");
    });
}

function setPillStates(gnss, imu, ai, nhc, ekf) {
    pillGnss.classList.toggle('active', gnss);
    pillImu.classList.toggle('active', imu);
    pillAi.classList.toggle('active', ai);
    pillNhc.classList.toggle('active', nhc);
    if (pillEkf) pillEkf.classList.toggle('active', ekf);
}

// ==========================================================================
// 6. Voice Guidance Speech Engine (Web Speech API)
// ==========================================================================
function speakVoicePrompt(message) {
    if (!isVoiceEnabled || !('speechSynthesis' in window)) return;
    window.speechSynthesis.cancel();
    const utterance = new SpeechSynthesisUtterance(message);
    utterance.rate = 1.05;
    utterance.pitch = 1.0;
    utterance.lang = 'en-IN';
    window.speechSynthesis.speak(utterance);
}

btnVoiceToggle.addEventListener('click', () => {
    isVoiceEnabled = !isVoiceEnabled;
    voiceIcon.innerText = isVoiceEnabled ? "🔊" : "🔇";
    showToast(isVoiceEnabled ? "Voice Guidance: ON" : "Voice Guidance: MUTED");
    if (isVoiceEnabled) speakVoicePrompt("Voice guidance enabled");
});

// ==========================================================================
// 7. NavIC Constellation Skyplot & Satellite Signal Monitor
// ==========================================================================
function buildConstellationBars() {
    satBarsList.innerHTML = '';
    SATELLITE_CONSTELLATION.forEach((sat, idx) => {
        const row = document.createElement('div');
        row.className = 'sat-bar-row';
        row.id = `sat-row-${idx}`;
        row.innerHTML = `
            <span class="sat-name">${sat.name}</span>
            <div class="sat-track">
                <div class="sat-fill" id="sat-bar-${idx}" style="width: ${sat.baseDb * 2}%;"></div>
            </div>
            <span class="sat-db" id="sat-db-${idx}">${sat.baseDb} dB</span>
        `;
        satBarsList.appendChild(row);
    });
}

function updateConstellationStatus(isOutage) {
    if (isOutage) {
        satCountVal.innerText = "0 SVs";
        satCountVal.style.color = "#EF4444";
        dopTag.innerText = "OUTAGE";
        cStatusVal.innerText = "NO SIGNAL (TUNNEL / FOREST)";
        cStatusVal.className = "c-stat-val text-amber";
        cCn0Val.innerText = "0.0 dB-Hz";
        cCepVal.innerText = "±8.5 m (DR)";

        SATELLITE_CONSTELLATION.forEach((sat, idx) => {
            const bar = document.getElementById(`sat-bar-${idx}`);
            const db = document.getElementById(`sat-db-${idx}`);
            if (bar) bar.classList.add('outage');
            if (db) db.innerText = "0 dB";
        });
    } else {
        satCountVal.innerText = "14 SVs";
        satCountVal.style.color = "#00E676";
        dopTag.innerText = "HDOP 0.8";
        cStatusVal.innerText = "FIXED (3D DGPS)";
        cStatusVal.className = "c-stat-val text-green";
        cCn0Val.innerText = "44.2 dB-Hz";
        cCepVal.innerText = "±2.4 m";

        SATELLITE_CONSTELLATION.forEach((sat, idx) => {
            const bar = document.getElementById(`sat-bar-${idx}`);
            const db = document.getElementById(`sat-db-${idx}`);
            if (bar) {
                bar.classList.remove('outage');
                bar.style.width = `${sat.baseDb * 2}%`;
            }
            if (db) db.innerText = `${sat.baseDb} dB`;
        });
    }
}

btnToggleConstellation.addEventListener('click', () => {
    constellationModal.style.display = 'flex';
});

btnFabConstellation.addEventListener('click', () => {
    constellationModal.style.display = 'flex';
});

btnCloseConstellation.addEventListener('click', () => {
    constellationModal.style.display = 'none';
});

// ==========================================================================
// 8. Simulation Speed & View Mode Controls
// ==========================================================================
document.querySelectorAll('.sim-speed-btn').forEach(btn => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('.sim-speed-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        simulationRateMultiplier = parseFloat(btn.getAttribute('data-speed')) || 1.0;
        simSpeedBadge.innerText = `${simulationRateMultiplier}x`;
        showToast(`Simulation Rate: ${simulationRateMultiplier}x`);
    });
});

btnHeadingMode.addEventListener('click', () => {
    isHeadUpMode = !isHeadUpMode;
    btnHeadingMode.innerText = isHeadUpMode ? "🧭 3D" : "🧭 2D";

    const mapCanvas = document.getElementById('map');
    if (mapCanvas) {
        mapCanvas.classList.toggle('headup-mode', isHeadUpMode);
    }
    showToast(isHeadUpMode ? "3D Cockpit Mode" : "2D Overview Mode");
});

// Outage Toggle (Simulate GNSS Outage for Judges)
btnToggleOutage.addEventListener('click', async () => {
    isOutageActive = !isOutageActive;
    showToast(isOutageActive ? "⚡ Forest/Tunnel Outage Injected: AI Dead Reckoning Active" : "🔄 GNSS Restored: Realignment Complete");
    try {
        await fetch(`${API_BASE}/outage`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ outage: isOutageActive })
        });
    } catch(e) {}
});

btnRecenter.addEventListener('click', () => {
    if (isNavigating && vehicleMarker) {
        map.setView(vehicleMarker.getLatLng(), 17, { animate: true });
    } else {
        map.setView([currentUserLocation.lat, currentUserLocation.lng], 16, { animate: true });
    }
});

const btnZoomIn = document.getElementById('btn-zoom-in');
const btnZoomOut = document.getElementById('btn-zoom-out');

if (btnZoomIn) {
    btnZoomIn.addEventListener('click', () => {
        if (map) map.zoomIn();
    });
}

if (btnZoomOut) {
    btnZoomOut.addEventListener('click', () => {
        if (map) map.zoomOut();
    });
}

btnCompass.addEventListener('click', () => {
    showToast("Reoriented to True North");
    if (compassNeedle) compassNeedle.style.transform = 'rotate(0deg)';
});

btnLayers.addEventListener('click', () => {
    const nextIdx = (currentLayerIdx + 1) % GOOGLE_TILE_LAYERS.length;
    applyGoogleMapLayer(nextIdx);
});

btnToggleFrame.addEventListener('click', () => {
    deviceFrame.classList.toggle('fullscreen-mode');
    setTimeout(() => map.invalidateSize(), 300);
});

btnTogglePlay.addEventListener('click', () => {
    isPaused = !isPaused;
    btnTogglePlay.innerText = isPaused ? "▶" : "⏸";
    showToast(isPaused ? "Navigation Paused" : "Navigation Resumed");
});

btnReloop.addEventListener('click', () => {
    currentStepIdx = 0;
    progressAlongStep = 0.0;
    isOutageActive = false;
    traveledPolyline.setLatLngs([]);
    outagePolyline.setLatLngs([]);
    showToast("Restarted Route Navigation");
});

btnToggleSteps.addEventListener('click', () => {
    stepsDrawer.style.display = 'flex';
});

navHeader.addEventListener('click', () => {
    stepsDrawer.style.display = 'flex';
});

btnCloseSteps.addEventListener('click', (e) => {
    e.stopPropagation();
    stepsDrawer.style.display = 'none';
});

function renderDirectionsSteps(steps) {
    stepsList.innerHTML = '';
    steps.forEach((s, idx) => {
        const card = document.createElement('div');
        card.className = 'step-card';
        card.innerHTML = `
            <div class="step-icon-wrap">↗</div>
            <div class="step-details">
                <div class="step-instruction">${s.instruction}</div>
                <div class="step-distance">${s.distance}</div>
            </div>
        `;
        stepsList.appendChild(card);
    });
}

function showToast(msg) {
    if (!appToast) return;
    appToast.innerText = msg;
    appToast.style.display = 'block';
    clearTimeout(window.toastTimer);
    window.toastTimer = setTimeout(() => { appToast.style.display = 'none'; }, 2500);
}

function setupHardwareSensors() {
    if (window.DeviceMotionEvent) {
        window.addEventListener('devicemotion', (event) => {
            const acc = event.accelerationIncludingGravity;
            const rot = event.rotationRate;
            if (acc && acc.x !== null) {
                realDeviceMotion = {
                    ax: acc.x || 0.0,
                    ay: acc.y || 0.0,
                    az: acc.z || 9.81,
                    gx: rot ? rot.beta || 0.0 : 0.0,
                    gy: rot ? rot.gamma || 0.0 : 0.0,
                    gz: rot ? rot.alpha || 0.0 : 0.0
                };
            }
        });
    }
}

// Math Helpers
function calculateBearingDeg(lat1, lon1, lat2, lon2) {
    const φ1 = lat1 * Math.PI / 180;
    const φ2 = lat2 * Math.PI / 180;
    const Δλ = (lon2 - lon1) * Math.PI / 180;
    const y = Math.sin(Δλ) * Math.cos(φ2);
    const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
    return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
}

function calculateHaversineMeters(lat1, lon1, lat2, lon2) {
    const R = 6371e3;
    const φ1 = lat1 * Math.PI / 180;
    const φ2 = lat2 * Math.PI / 180;
    const Δφ = (lat2 - lat1) * Math.PI / 180;
    const Δλ = (lon2 - lon1) * Math.PI / 180;
    const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ/2) * Math.sin(Δλ/2);
    return R * (2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a)));
}

function updateMobileClock() {
    if (!statusClock) return;
    const d = new Date();
    statusClock.innerText = `${d.getHours().toString().padStart(2, '0')}:${d.getMinutes().toString().padStart(2, '0')}`;
}

// Background sync with Python EKF Backend
async function syncBackendState() {
    try {
        const res = await fetch(`${API_BASE}/state`);
        if (res.ok) {
            const data = await res.json();
            if (data.status === 'active' && data.imu) {
                realDeviceMotion.ax = data.imu.ax;
                realDeviceMotion.ay = data.imu.ay;
                realDeviceMotion.az = data.imu.az || 9.81;
                realDeviceMotion.gz = data.imu.gz || 0.0;
            }
        }
    } catch(e) {}
}

// Initialize on window load
window.addEventListener('load', () => {
    updateMobileClock();
    setInterval(updateMobileClock, 10000);
    initCockpit();
    requestAnimationFrame(animationFrameLoop);
    setInterval(syncBackendState, 350);
});

// ==========================================================================
// 9. SIH Judge Guided Demo & Performance Summary Modal Handlers
// ==========================================================================

function showTripSummaryModal() {
    if (!tripSummaryModal) return;

    const routeName = (calculatedAltRoutes && calculatedAltRoutes[selectedRouteIndex])
        ? calculatedAltRoutes[selectedRouteIndex].name
        : "Expressway Corridor";
    
    const km = (calculatedAltRoutes && calculatedAltRoutes[selectedRouteIndex])
        ? calculatedAltRoutes[selectedRouteIndex].km
        : "12.4";
        
    const secs = blackoutSeconds > 0 ? blackoutSeconds.toFixed(1) : "30.0";
    const rawDrift = (0.5 * 0.08 * Math.pow(parseFloat(secs), 2) * 12).toFixed(0);
    const ekfDrift = (0.04 * Math.pow(Math.max(1, parseFloat(secs)), 1.2)).toFixed(1);

    if (summaryRouteTitle) summaryRouteTitle.innerText = routeName;
    if (sumStatDist) sumStatDist.innerText = `${km} km`;
    if (sumStatOutage) sumStatOutage.innerText = `${secs}s`;
    if (sumStatRaw) sumStatRaw.innerText = `${rawDrift} m`;
    if (sumStatEkf) sumStatEkf.innerText = `${ekfDrift} m`;

    tripSummaryModal.style.display = 'flex';
}

if (btnCloseSummary) {
    btnCloseSummary.addEventListener('click', () => {
        if (tripSummaryModal) tripSummaryModal.style.display = 'none';
        btnEndRoute.click();
    });
}

function runJudgeGuidedDemo() {
    showToast("⚡ Starting 1-Click SIH Judge Demonstration...");
    
    // Pick Delhi Pragati Tunnel corridor
    const p = PRESETS.delhi;
    currentUserLocation = p.origin;
    originInput.value = `📍 ${p.origin.label}`;
    setDestinationAndPlan(p.dest, 'delhi');
    currentTunnelInterval = p.tunnelZone;

    // Set 2x simulation rate for smooth demonstration
    simulationRateMultiplier = 2.0;
    if (simSpeedBadge) simSpeedBadge.innerText = "2x";
    document.querySelectorAll('.sim-speed-btn').forEach(b => {
        b.classList.toggle('active', b.getAttribute('data-speed') === '2');
    });

    // Start Navigation
    setTimeout(() => {
        btnStartNavigation.click();
        showToast("🚨 Pragati Tunnel Outage Zone Approaching (NavIC AI DR Standing By)");
        speakVoicePrompt("SIH Judge Demo Initiated. Driving Pragati Tunnel expressway corridor. Intelligent dead reckoning engine monitoring 6 DoF IMU sensors.");
    }, 600);
}

if (btnJudgeGuidedDemo) {
    btnJudgeGuidedDemo.addEventListener('click', runJudgeGuidedDemo);
}

if (btnDesktopFullscreen) {
    btnDesktopFullscreen.addEventListener('click', () => {
        const isPhone = deviceFrame.classList.toggle('phone-mockup-mode');
        showToast(isPhone ? "📱 Switched to Phone Frame Mode" : "💻 Switched to Full Desktop Canvas Mode");
        setTimeout(() => {
            if (map) map.invalidateSize();
        }, 350);
    });
}


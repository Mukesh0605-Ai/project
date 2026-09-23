// app.js — NAVISENSE-IDR Master Application Controller
// SIH 2026 — Problem Statement 26168
// End-to-end Real GPS-based Destination-Independent Navigation System

import { GPSEngine, haversine, computeBearing, interpolate } from './engine/gps.js';
import { SensorEngine } from './engine/sensors.js';
import { AIModel } from './engine/aiModel.js';
import { MapEngine } from './engine/mapEngine.js';
import { RoutingEngine } from './engine/routing.js';
import { SessionLog } from './engine/sessionLog.js';
import { IOVNBD_DATA } from './engine/iovnbd_data.js';

// ============================================================
// CONSTANTS & CONFIG
// ============================================================
const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;

// IO-VNBD dataset cycling index (loops samples during DR)
let datasetIdx = 0;

// ============================================================
// APPLICATION STATE
// ============================================================
const S = {
    screen: 'home',

    // Navigation Flags
    isNavigating: false,
    isGPSLost: false,
    isRecovering: false,
    isArrived: false,
    isSimulation: false,

    // Real Authoritative GPS Telemetry (null until real GPS fix arrives)
    currentLat: null,
    currentLng: null,
    currentSpeed: 0,          // Authoritative speed (km/h)
    currentRawSpeed: 0,       // Raw GPS speed
    currentHeading: 0,        // Authoritative heading (degrees)
    currentAccuracy: null,    // ± meters
    lastGPSTimestamp: 0,

    // Origin & Destination Coordinates
    origin: null,             // { lat, lng, label }
    destination: null,        // { lat, lng, label, address, type }
    route: null,              // Selected active route object
    altRoutes: [],
    selectedRouteIdx: 0,

    // Route traversal (simulation only or route index reference)
    routeIdx: 0,              // Current segment index in route coords
    routeProgress: 0.0,       // Progress within current segment [0..1]
    routeFraction: 0.0,       // Overall route fraction [0..1]

    // Dead Reckoning State (IMU + AI + EKF)
    drLat: 0, drLng: 0,
    drSpeed: 0, drHeading: 0,
    drConfidence: 0.95,
    drDrift: 0,
    blackoutSeconds: 0,
    blackoutTimerId: null,

    // Recovery State
    recoveryStartTime: 0,
    recoveryDuration: 2500,
    drLatAtRecovery: 0, drLngAtRecovery: 0,
    gpsLatAtRecovery: 0, gpsLngAtRecovery: 0,

    // Session Metrics
    totalDistanceM: 0,
    lastPosForDist: null,
    maxDrift: 0,
    speedErrors: [],
    outageStartTime: 0,
    sessionStartTime: 0,

    // IMU Sensor Data (real phone sensors or synthetic fallback)
    imu: { ax: 0, ay: 0, az: 9.81, gx: 0, gy: 0, gz: 0, isReal: false },

    // Simulation multiplier (only applies when isSimulation is true)
    simSpeed: 1.0,

    // GPS Status (Standardized State Machine)
    // GPS_WAITING | GPS_GOOD | GPS_WEAK | GPS_LOST | GPS_RECOVERING
    gpsStatus: 'GPS_WAITING',
    gpsIsLive: false,
    _hasFlownToGPS: false,

    // Manual test loss toggle
    manualGPSLoss: false,

    // Deviation & Network
    deviationTime: 0,
    deviationDistance: 0,
    lastRerouteTime: 0,
    isRerouting: false,
    isOffline: !navigator.onLine
};

// ============================================================
// ENGINE INSTANCES
// ============================================================
const gpsEngine  = new GPSEngine();
const sensors    = new SensorEngine();
const aiModel    = new AIModel();
const mapEngine  = new MapEngine('map');
const routing    = new RoutingEngine();
const session    = new SessionLog();

// ============================================================
// PRESET DEMO CORRIDORS (Simulation Only)
// ============================================================
const PRESETS = [
    {
        id: 'pragati',
        name: 'Pragati Tunnel',
        origin: { lat: 28.6129, lng: 77.2295, label: 'India Gate, New Delhi' },
        dest:   { lat: 28.6248, lng: 77.2482, label: 'Pragati Tunnel, Delhi' },
        outageZones: [{ range: [0.30, 0.75], type: 'tunnel' }]
    },
    {
        id: 'atal',
        name: 'Atal Tunnel',
        origin: { lat: 32.3167, lng: 77.1500, label: 'Dhundi South Portal' },
        dest:   { lat: 32.4100, lng: 77.1650, label: 'Sissu North Portal' },
        outageZones: [{ range: [0.15, 0.90], type: 'tunnel' }]
    },
    {
        id: 'ooty',
        name: 'Ooty Pass',
        origin: { lat: 11.0168, lng: 76.9558, label: 'Coimbatore Junction' },
        dest:   { lat: 11.4102, lng: 76.6950, label: 'Ooty Bus Stand' },
        outageZones: [
            { range: [0.20, 0.45], type: 'forest' },
            { range: [0.65, 0.85], type: 'forest' }
        ]
    },
    {
        id: 'blr',
        name: 'Electronic City',
        origin: { lat: 12.9716, lng: 77.5946, label: 'MG Road, Bangalore' },
        dest:   { lat: 12.8452, lng: 77.6602, label: 'Electronic City Phase 1' },
        outageZones: [
            { range: [0.30, 0.40], type: 'urban' },
            { range: [0.55, 0.65], type: 'bridge' }
        ]
    }
];

// ============================================================
// UI HELPERS
// ============================================================
const MAP_SCREENS = new Set(['home', 'route', 'navigation', 'dr', 'recovery']);

function showScreen(name) {
    document.querySelectorAll('.screen').forEach(s => {
        // Keep navigation UI visible when in DR mode
        if (name === 'dr' && s.id === 'screen-navigation') return;
        s.classList.remove('active');
    });
    const el = document.getElementById(`screen-${name}`);
    if (el) el.classList.add('active');
    S.screen = name;

    const pm = document.getElementById('map');
    if (pm) pm.style.display = MAP_SCREENS.has(name) ? 'block' : 'none';

    // Show/hide floating "My Location" button on map screens
    const fabMyLoc = $('btn-my-location');
    if (fabMyLoc) {
        fabMyLoc.style.display = (name === 'home' || name === 'route' || name === 'navigation') ? 'flex' : 'none';
    }

    if (MAP_SCREENS.has(name)) setTimeout(() => mapEngine.invalidateSize(), 150);
}

const $ = id => document.getElementById(id);
const T = (id, v) => { const el = $(id); if (el) el.textContent = v; };
const H = (id, v) => { const el = $(id); if (el) el.innerHTML = v; };

function toast(msg, type = 'info', ms = 3000) {
    const el = $('toast');
    if (!el) return;
    el.textContent = msg;
    el.className = `toast toast-${type} show`;
    clearTimeout(el._t);
    el._t = setTimeout(() => el.classList.remove('show'), ms);
}

function fmtDist(m) {
    if (m == null || isNaN(m)) return '-- km';
    return m >= 1000 ? `${(m / 1000).toFixed(1)} km` : `${Math.round(m)} m`;
}
function fmtTime(s) {
    if (s == null || isNaN(s)) return '-- min';
    if (s < 60) return `${Math.round(s)}s`;
    if (s < 3600) return `${Math.round(s / 60)} min`;
    return `${Math.floor(s / 3600)}h ${Math.round((s % 3600) / 60)}m`;
}
function speak(t) {
    if (!window.speechSynthesis) return;
    try {
        const u = new SpeechSynthesisUtterance(t);
        u.lang = 'en-IN';
        u.rate = 0.95;
        window.speechSynthesis.cancel();
        window.speechSynthesis.speak(u);
    } catch (e) {}
}
function beep(type) {
    try {
        const ctx = new (window.AudioContext || window.webkitAudioContext)();
        const now = ctx.currentTime, osc = ctx.createOscillator(), g = ctx.createGain();
        osc.type = type === 'loss' ? 'triangle' : 'sine';
        if (type === 'loss') {
            osc.frequency.setValueAtTime(440, now);
            osc.frequency.linearRampToValueAtTime(220, now + 0.4);
        } else {
            osc.frequency.setValueAtTime(523, now);
            osc.frequency.setValueAtTime(784, now + 0.2);
        }
        g.gain.setValueAtTime(0.3, now);
        g.gain.exponentialRampToValueAtTime(0.001, now + 0.5);
        osc.connect(g);
        g.connect(ctx.destination);
        osc.start(now);
        osc.stop(now + 0.5);
    } catch (e) {}
}

// ============================================================
// MAIN CONTINUOUS LOOP
// ============================================================
let lastFrameTime = null;
let rafId = null;

function startMainLoop() {
    lastFrameTime = performance.now();
    function loop(now) {
        if (!S.isNavigating) return;
        const dt = Math.min((now - lastFrameTime) / 1000, 0.1);
        lastFrameTime = now;
        tick(dt);
        rafId = requestAnimationFrame(loop);
    }
    rafId = requestAnimationFrame(loop);
}

function stopMainLoop() {
    S.isNavigating = false;
    if (rafId) cancelAnimationFrame(rafId);
    rafId = null;
    mapEngine.setFollowCamera(false);
}

function tick(dt) {
    if (!S.route || !S.route.coords) return;
    const coords = S.route.coords;

    // 1. ROUTE ADVANCEMENT — STRICTLY FOR DEMO SIMULATION MODE ONLY
    // When live GPS is active (S.gpsIsLive === true and !S.isSimulation), advanceRoute() MUST NOT execute!
    if (S.isSimulation && !S.gpsIsLive) {
        const speedMs = (S.currentSpeed / 3.6) * S.simSpeed;
        const distThisFrame = speedMs * dt;
        advanceRoute(coords, distThisFrame);
    }

    // 2. Feed vehicle state to sensor engine for synthetic IMU fallback
    sensors.updateVehicleState(S.currentSpeed, S.currentHeading, S.isGPSLost);

    let displayLat, displayLng, displayMode;

    if (S.isRecovering) {
        // Recovery interpolation: DR position blending to real GPS
        const elapsed = Date.now() - S.recoveryStartTime;
        const t = Math.min(1, elapsed / S.recoveryDuration);
        const eased = t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t;

        displayLat = S.drLatAtRecovery + (S.gpsLatAtRecovery - S.drLatAtRecovery) * eased;
        displayLng = S.drLngAtRecovery + (S.gpsLngAtRecovery - S.drLngAtRecovery) * eased;
        displayMode = 'RECOVERY';

        S.drConfidence = 0.4 + eased * 0.55;

        if (t >= 1) {
            S.isRecovering = false;
            S.currentLat = S.gpsLatAtRecovery;
            S.currentLng = S.gpsLngAtRecovery;
            session.endOutage(haversine(S.drLatAtRecovery, S.drLngAtRecovery, S.gpsLatAtRecovery, S.gpsLngAtRecovery), S.recoveryDuration);
            showScreen('navigation');
            toast('✅ GPS Restored — Authoritative real GPS active', 'success');
        }

    } else if (S.isGPSLost) {
        // DEAD RECKONING USING IO-VNBD + AI + EKF
        const sample = IOVNBD_DATA[datasetIdx % IOVNBD_DATA.length];
        datasetIdx++;

        const imu = S.imu.isReal ? S.imu : sample;
        const aiOut = aiModel.infer(imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, dt, S.drSpeed);

        const axForward = imu.ax * Math.cos(S.drHeading * DEG2RAD) + imu.ay * Math.sin(S.drHeading * DEG2RAD);
        const physDv = axForward * dt * 3.6 * 0.3;
        const aiDv   = aiOut.deltaSpeed * dt * 10;

        let newSpeed = S.drSpeed + physDv + aiDv;
        newSpeed = Math.max(0, Math.min(120, newSpeed));

        const physDh = imu.gz * dt * RAD2DEG * 0.4;
        const aiDh   = aiOut.deltaHeading * dt * 5;
        let newHeading = (S.drHeading + physDh + aiDh + 360) % 360;

        const avgSpeedMs = (S.drSpeed + newSpeed) / 2 / 3.6;
        const distM = avgSpeedMs * dt * (S.isSimulation ? S.simSpeed : 1.0);

        const hRad = newHeading * DEG2RAD;
        const dlat = (distM * Math.cos(hRad)) / 111320;
        const dlng = (distM * Math.sin(hRad)) / (111320 * Math.cos(S.drLat * DEG2RAD));

        let newLat = S.drLat + dlat;
        let newLng = S.drLng + dlng;

        // Map matching constraint
        const matched = mapMatch(newLat, newLng, coords);
        if (matched) {
            newLat = newLat * 0.25 + matched.lat * 0.75;
            newLng = newLng * 0.25 + matched.lng * 0.75;
            newHeading = newHeading * 0.3 + matched.heading * 0.7;
        }

        S.drLat = newLat;
        S.drLng = newLng;
        S.drSpeed = newSpeed;
        S.drHeading = newHeading;
        S.drDrift = 0.5 * 0.08 * S.blackoutSeconds * S.blackoutSeconds;
        S.drConfidence = Math.max(0.20, aiOut.confidence * Math.max(0.25, 1 - S.blackoutSeconds / 120));
        if (S.drDrift > S.maxDrift) S.maxDrift = S.drDrift;

        session.logDR(newLat, newLng, newSpeed, newHeading, S.drConfidence, S.drDrift, S.blackoutSeconds);

        S.currentSpeed = newSpeed;
        S.currentHeading = newHeading;
        displayLat = newLat;
        displayLng = newLng;
        displayMode = 'DR';

        updateDRHud(imu);

    } else {
        // GPS Active Mode
        if (S.gpsIsLive && S.currentLat !== null) {
            displayLat = S.currentLat;
            displayLng = S.currentLng;
        } else {
            const pt = getRoutePosition(coords);
            displayLat = pt.lat;
            displayLng = pt.lng;
            S.currentLat = pt.lat;
            S.currentLng = pt.lng;
        }
        displayMode = 'GPS';

        // In simulation mode ONLY, heading follows route orientation
        if (S.isSimulation && !S.gpsIsLive) {
            const nextIdx = Math.min(S.routeIdx + 1, coords.length - 1);
            S.currentHeading = computeBearing(
                coords[S.routeIdx].lat, coords[S.routeIdx].lng,
                coords[nextIdx].lat, coords[nextIdx].lng
            );
        }
    }

    // Update map vehicle position
    mapEngine.updateVehicle(displayLat, displayLng, S.currentHeading, S.currentAccuracy || 5, displayMode);

    // Update HUD metrics
    updateNavHUD(displayLat, displayLng, coords);

    // Simulation auto-outage zone trigger
    if (S.isSimulation && !S.isGPSLost && !S.isRecovering && S.route.outageZones) {
        let inside = false;
        for (const z of S.route.outageZones) {
            if (S.routeFraction >= z.range[0] && S.routeFraction < z.range[1]) {
                inside = true;
                break;
            }
        }
        if (inside && !S.manualGPSLoss) {
            triggerGPSLoss();
        }
    } else if (S.isSimulation && S.isGPSLost && !S.manualGPSLoss && S.route.outageZones) {
        let inside = false;
        for (const z of S.route.outageZones) {
            if (S.routeFraction >= z.range[0] && S.routeFraction < z.range[1]) {
                inside = true;
                break;
            }
        }
        if (!inside) {
            triggerGPSRecovery();
        }
    }

    // Arrival Check (threshold: 28 meters to destination)
    if (S.destination && S.destination.lat != null) {
        const distToDest = haversine(displayLat, displayLng, S.destination.lat, S.destination.lng);
        if (distToDest < 28 && !S.isArrived) {
            onArrived();
        }
    }
}

// ============================================================
// SIMULATION-ONLY ROUTE ADVANCEMENT
// ============================================================
function advanceRoute(coords, distMeters) {
    if (!coords || coords.length < 2) return;
    if (S.routeIdx >= coords.length - 1) return;

    let remaining = distMeters;
    while (remaining > 0 && S.routeIdx < coords.length - 1) {
        const cur  = coords[S.routeIdx];
        const next = coords[S.routeIdx + 1];
        const segLen = haversine(cur.lat, cur.lng, next.lat, next.lng);
        const segRemaining = segLen * (1 - S.routeProgress);

        if (remaining < segRemaining) {
            S.routeProgress += remaining / segLen;
            remaining = 0;
        } else {
            remaining -= segRemaining;
            S.routeIdx++;
            S.routeProgress = 0;
        }
    }

    S.routeFraction = S.routeIdx / (coords.length - 1);
}

function getRoutePosition(coords) {
    if (!coords || coords.length < 2) return { lat: S.currentLat || 20.5937, lng: S.currentLng || 78.9629 };
    const i = Math.min(S.routeIdx, coords.length - 2);
    const p = Math.min(1, S.routeProgress);
    return interpolate(coords[i].lat, coords[i].lng, coords[i + 1].lat, coords[i + 1].lng, p);
}

function remainingDistance(lat, lng, coords) {
    if (!coords || coords.length === 0) return 0;
    let d = 0;
    const startIdx = Math.min(S.routeIdx, coords.length - 1);
    // Distance from current position to next route segment point
    if (coords[startIdx]) {
        d += haversine(lat, lng, coords[startIdx].lat, coords[startIdx].lng);
    }
    for (let i = startIdx; i < coords.length - 1; i++) {
        d += haversine(coords[i].lat, coords[i].lng, coords[i + 1].lat, coords[i + 1].lng);
    }
    return d;
}

// Map Matching
let lastMatchedRouteIdx = 0;
function mapMatch(lat, lng, coords) {
    if (!coords || coords.length < 2) return null;
    let minDist = 70;
    let bestIdx = -1;
    const start = Math.max(0, lastMatchedRouteIdx - 2);
    const end   = Math.min(coords.length - 1, lastMatchedRouteIdx + 30);

    for (let i = start; i <= end; i++) {
        const d = haversine(lat, lng, coords[i].lat, coords[i].lng);
        if (d < minDist) { minDist = d; bestIdx = i; }
    }
    if (bestIdx < 0) return null;

    lastMatchedRouteIdx = bestIdx;
    const nxt = coords[Math.min(bestIdx + 1, coords.length - 1)];
    return {
        lat: coords[bestIdx].lat,
        lng: coords[bestIdx].lng,
        heading: computeBearing(coords[bestIdx].lat, coords[bestIdx].lng, nxt.lat, nxt.lng)
    };
}

// ============================================================
// GPS LOSS & RECOVERY (Max 5 mins Dead Reckoning)
// ============================================================
function triggerGPSLoss() {
    if (S.isGPSLost) return;
    S.isGPSLost = true;
    S.outageStartTime = Date.now();
    S.blackoutSeconds = 0;
    datasetIdx = 0;

    // Preserve authoritative last confirmed GPS position, speed, and heading
    S.drLat = S.currentLat;
    S.drLng = S.currentLng;
    S.drSpeed = S.currentSpeed;
    S.drHeading = S.currentHeading;
    S.drConfidence = 0.95;
    S.drDrift = 0;
    lastMatchedRouteIdx = S.routeIdx;

    aiModel.reset();
    session.startOutage();

    // Start blackout timer
    clearInterval(S.blackoutTimerId);
    S.blackoutTimerId = setInterval(() => {
        S.blackoutSeconds += 0.1;

        // Requirement M: Stop navigation updates after max 5 minutes (300s)
        if (S.blackoutSeconds >= 300) {
            clearInterval(S.blackoutTimerId);
            T('dr-banner-title', 'GPS Lost — Navigation Paused');
            T('dr-banner-sub', 'No GPS signal for > 5 minutes. Navigation paused.');
            toast('GPS Lost — Navigation Paused (>5 mins without signal)', 'error', 6000);
            stopMainLoop();
        }
    }, 100);

    showScreen('dr');
    toast('🔴 GPS SIGNAL LOST — Intelligent Dead Reckoning Active', 'error', 4000);
    beep('loss');
    speak('GPS signal lost. Switching to intelligent dead reckoning.');
}

function triggerGPSRecovery() {
    if (!S.isGPSLost) return;
    S.isGPSLost = false;

    clearInterval(S.blackoutTimerId);
    S.blackoutTimerId = null;

    const finalDrift = haversine(S.drLat, S.drLng, S.currentLat || S.drLat, S.currentLng || S.drLng);
    const dur = (Date.now() - S.outageStartTime) / 1000;

    // Real GPS becomes authoritative immediately
    S.isRecovering = true;
    S.recoveryStartTime = Date.now();
    S.drLatAtRecovery = S.drLat;
    S.drLngAtRecovery = S.drLng;

    S.gpsLatAtRecovery = S.currentLat;
    S.gpsLngAtRecovery = S.currentLng;

    showScreen('recovery');
    T('recovery-duration', `${dur.toFixed(1)} s`);
    T('recovery-drift', `${finalDrift.toFixed(1)} m`);
    T('recovery-dist', `${(S.totalDistanceM / 1000).toFixed(2)} km`);

    toast('✅ GPS Signal Restored — Real GPS authoritative', 'success', 3500);
    beep('restore');
    speak('GPS signal restored. Position locked to satellites.');
}

// ============================================================
// HUD UPDATES
// ============================================================
function updateNavHUD(lat, lng, coords) {
    const rem  = remainingDistance(lat, lng, coords);
    const spd  = S.isGPSLost ? S.drSpeed : S.currentSpeed;
    const etaS = spd > 1 ? (rem / (spd / 3.6)) : (rem / (40 / 3.6));

    T('nav-speed', Math.round(spd));

    if (!S.isGPSLost && !S.isRecovering) {
        T('bm-1-lbl', 'Speed');
        T('bm-1-val', `${Math.round(spd)} km/h`);
        T('bm-2-lbl', 'Distance');
        T('bm-2-val', fmtDist(rem));
        T('bm-3-lbl', 'ETA');
        T('bm-3-val', fmtTime(etaS));

        let gpsLabel = 'GPS GOOD';
        let pillClass = 'gps good';
        if (S.gpsStatus === 'GPS_WEAK') {
            gpsLabel = 'GPS WEAK';
            pillClass = 'gps weak';
        } else if (S.gpsStatus === 'GPS_WAITING') {
            gpsLabel = 'GPS WAITING';
            pillClass = 'gps waiting';
        }
        T('current-mode-pill', gpsLabel);
        const pill = $('current-mode-pill');
        if (pill) pill.className = 'mode-pill ' + pillClass;

        const confContainer = $('conf-info-container');
        if (confContainer) confContainer.style.display = 'none';
    }

    // Step instructions
    if (S.route && S.route.steps && S.route.steps.length > 0) {
        const stepIdx = Math.min(Math.floor(S.routeFraction * S.route.steps.length), S.route.steps.length - 1);
        const step = S.route.steps[stepIdx];
        if (step) {
            let instr = step.instruction;
            if (step.distance && step.distance !== '0 m') instr += ` in ${step.distance}`;
            T('maneuver-icon', maneuverIcon(step.instruction));
            T('maneuver-instruction', instr);
        }
    }

    // IMU sensor HUD
    const imu = S.imu;
    T('hud-ax', imu.ax.toFixed(2));
    T('hud-ay', imu.ay.toFixed(2));
    T('hud-gz', imu.gz.toFixed(2));
    T('hud-sensor-type', imu.isReal ? '📱 Device IMU' : '📊 IO-VNBD');
}

function updateDRHud(imu) {
    const dur = (Date.now() - S.outageStartTime) / 1000;
    const conf = Math.round(S.drConfidence * 100);
    const rem = remainingDistance(S.drLat, S.drLng, S.route ? S.route.coords : []);

    T('bm-1-lbl', 'Speed');
    T('bm-1-val', `${Math.round(S.drSpeed)} km/h`);
    T('bm-2-lbl', 'Distance');
    T('bm-2-val', fmtDist(rem));
    T('bm-3-lbl', 'Blackout');
    T('bm-3-val', `${dur.toFixed(1)}s`);

    T('current-mode-pill', 'ESTIMATED (DR)');
    const pill = $('current-mode-pill');
    if (pill) pill.className = 'mode-pill dr estimated';

    const confContainer = $('conf-info-container');
    if (confContainer) confContainer.style.display = 'flex';
    T('bm-conf-val', `${conf}%`);

    updateNavHUD(S.drLat, S.drLng, S.route ? S.route.coords : []);
}

function maneuverIcon(instr) {
    const t = (instr || '').toLowerCase();
    if (t.includes('right')) return '↱';
    if (t.includes('left'))  return '↰';
    if (t.includes('u-turn')) return '↩';
    if (t.includes('arriv')) return '📍';
    return '↑';
}

// ============================================================
// ARRIVAL
// ============================================================
function onArrived() {
    if (S.isArrived) return;
    S.isArrived = true;
    stopMainLoop();
    clearInterval(S.blackoutTimerId);
    session.endSession();

    toast('🏁 You have arrived at your destination!', 'success', 5000);
    speak('You have arrived at your destination.');

    const overlay = $('arrival-overlay');
    const destName = $('arrival-dest-name');
    if (overlay && destName) {
        destName.textContent = S.destination ? S.destination.label : 'Destination';
        overlay.style.display = 'flex';
    } else {
        setTimeout(() => showResultsScreen(), 1500);
    }
}

// ============================================================
// RESULTS SCREEN
// ============================================================
function showResultsScreen() {
    showScreen('results');
    const r = session.getResults();
    T('res-blackout', `${r.gpsBlackoutDuration} s`);
    T('res-distance', `${r.travelledDistanceKm} km`);
    T('res-pos-error', `${r.positionErrorM} m`);
    T('res-max-drift', `${r.maxDriftM} m`);
    T('res-speed-error', `${r.speedErrorKmh} km/h`);
    T('res-heading-error', `${r.headingErrorDeg}°`);
    T('res-recovery', `${r.recoveryTimeSecs} s`);
    T('res-confidence', `${r.overallConfidence}%`);
    T('res-score', r.overallScore);
    drawResultsChart();
}

function drawResultsChart() {
    const canvas = $('results-chart');
    if (!canvas || !canvas.getContext) return;
    const ctx = canvas.getContext('2d');
    const W = canvas.width = canvas.offsetWidth;
    const H = canvas.height = canvas.offsetHeight || 80;
    ctx.clearRect(0, 0, W, H);

    const pts = session.drPoints;
    if (pts.length < 2) {
        ctx.fillStyle = '#64748B'; ctx.font = '12px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('DR phase telemetry will appear here', W / 2, H / 2);
        return;
    }

    ctx.strokeStyle = '#10B981'; ctx.lineWidth = 2; ctx.setLineDash([]);
    ctx.beginPath();
    pts.forEach((p, i) => {
        const x = (i / (pts.length - 1)) * W;
        const y = H - p.confidence * (H - 16) - 8;
        i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.stroke();

    const maxD = Math.max(...pts.map(p => p.drift || 0), 1);
    ctx.strokeStyle = '#EF4444'; ctx.lineWidth = 1.5; ctx.setLineDash([4, 2]);
    ctx.beginPath();
    pts.forEach((p, i) => {
        const x = (i / (pts.length - 1)) * W;
        const y = H - ((p.drift || 0) / maxD) * (H - 16) - 8;
        i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.stroke();
    ctx.setLineDash([]);
}

// ============================================================
// GPS ENGINE CALLBACKS (Real Authoritative Position Update)
// ============================================================
function onGPSUpdate(pos) {
    S.gpsIsLive = true;
    S.currentLat = pos.lat;
    S.currentLng = pos.lng;
    S.currentSpeed = pos.speed || 0;
    S.currentRawSpeed = pos.rawSpeed || 0;
    S.currentHeading = pos.bearing || 0;
    S.currentAccuracy = pos.accuracy || 5;
    S.lastGPSTimestamp = pos.ts || Date.now();

    // FIRST GPS FIX
    if (!S._hasFlownToGPS) {
        S._hasFlownToGPS = true;
        S.origin = { lat: pos.lat, lng: pos.lng, label: 'Current Location' };
        mapEngine.flyTo(pos.lat, pos.lng, 16);
        toast('📍 GPS Locked to your current location', 'success', 2500);

        routing.reverseGeocode(pos.lat, pos.lng).then(label => {
            if (S.origin) S.origin.label = label;
            const ro = $('route-origin-display');
            if (ro) ro.value = `📍 ${label}`;
        });
    }

    // When on home or route screen, keep vehicle marker updated at exact real coordinates
    if (!S.isNavigating) {
        mapEngine.updateVehicle(pos.lat, pos.lng, pos.bearing || 0, pos.accuracy, 'GPS');
    }

    // LIVE POSITION UPDATE DURING ACTIVE NAVIGATION
    if (S.isNavigating && !S.isGPSLost && S.route && S.route.coords) {
        const coords = S.route.coords;

        // Map matching to find nearest segment index
        const matched = mapMatch(pos.lat, pos.lng, coords);
        if (matched && lastMatchedRouteIdx > S.routeIdx) {
            S.routeIdx = lastMatchedRouteIdx;
            S.routeFraction = S.routeIdx / Math.max(coords.length - 1, 1);
        }

        // Route Deviation Check (50m threshold + 4s persistence)
        if (matched) {
            const distToRoute = haversine(pos.lat, pos.lng, matched.lat, matched.lng);
            S.deviationDistance = distToRoute;

            if (distToRoute > 50) {
                S.deviationTime += 1.0;
                if (S.deviationTime >= 4.0 && Date.now() - S.lastRerouteTime > 12000 && !S.isRerouting) {
                    recalculateRouteLive(pos.lat, pos.lng);
                }
            } else {
                S.deviationTime = 0;
            }
        }
    }
}

async function recalculateRouteLive(curLat, curLng) {
    if (!S.destination || S.isOffline || S.isRerouting) {
        if (S.isOffline) toast('Network offline: Continuing on existing route', 'warning');
        return;
    }

    S.isRerouting = true;
    S.lastRerouteTime = Date.now();
    S.deviationTime = 0;
    toast('Route deviation detected — recalculating route...', 'warning', 2500);

    try {
        const newRoute = await routing.calculateRoute(curLat, curLng, S.destination.lat, S.destination.lng);
        if (newRoute && newRoute.coords && !newRoute.isFallback) {
            S.route = newRoute;
            S.routeIdx = 0;
            S.routeProgress = 0;
            S.routeFraction = 0;
            lastMatchedRouteIdx = 0;
            mapEngine.setRoute(newRoute.coords);
            toast('✅ Route updated to destination', 'success', 2500);
        }
    } catch (e) {
        console.warn('[Reroute] Live recalculation failed:', e);
    } finally {
        S.isRerouting = false;
    }
}

function onGPSStatus(status) {
    S.gpsStatus = status;
    const badge = $('gps-badge');
    if (!badge) return;

    switch (status) {
        case 'GPS_GOOD':
            badge.textContent = '🟢 GPS GOOD';
            badge.className = 'gps-badge good';
            break;
        case 'GPS_WEAK':
            badge.textContent = '🟡 GPS WEAK';
            badge.className = 'gps-badge weak';
            break;
        case 'GPS_LOST':
            badge.textContent = '🔴 GPS LOST';
            badge.className = 'gps-badge lost';
            if (S.isNavigating && !S.isGPSLost) {
                triggerGPSLoss();
            }
            break;
        case 'GPS_RECOVERING':
            badge.textContent = '🔵 GPS RECOVERING';
            badge.className = 'gps-badge recovering';
            if (S.isGPSLost && !S.manualGPSLoss) {
                triggerGPSRecovery();
            }
            break;
        case 'GPS_WAITING':
        default:
            badge.textContent = '⏳ GPS WAITING';
            badge.className = 'gps-badge waiting';
            break;
    }
}

function onGPSError(code, message) {
    toast(message, 'error', 4500);
}

function onSensorUpdate(data) {
    S.imu = data;
}

// ============================================================
// ROUTING & DESTINATION PLANNING
// ============================================================
async function planRoute(outageZones = null) {
    if (!S.currentLat && !S.origin) {
        toast('Waiting for GPS location...', 'warning');
        return;
    }
    if (!S.origin) {
        S.origin = { lat: S.currentLat, lng: S.currentLng, label: 'Current Location' };
    }
    if (!S.destination || S.destination.lat == null) {
        toast('Select a destination first', 'warning');
        return;
    }

    T('route-status', '🔍 Calculating routes from Current GPS...');
    const ro = $('route-origin-display');
    if (ro) ro.value = `📍 ${S.origin.label || 'Current Location'}`;
    const rd = $('route-dest-display');
    if (rd) rd.value = `📍 ${S.destination.label}`;

    showScreen('route');

    try {
        const routes = await routing.getAlternativeRoutes(S.origin, S.destination);
        S.altRoutes = routes;

        if (outageZones) {
            routes.forEach(r => { r.outageZones = outageZones; });
        }

        renderRouteCards(routes);
        mapEngine.showAltRoutes(routes, 0);
        mapEngine.setMarkers(S.origin, S.destination);
        selectRoute(0);

        const hasFallback = routes.some(r => r.isFallback);
        const fbWarn = $('fallback-route-warning');
        if (fbWarn) fbWarn.style.display = hasFallback ? 'block' : 'none';

        T('route-status', hasFallback ? '⚠️ Offline direct line used' : `✅ ${routes.length} route(s) calculated`);
    } catch (e) {
        toast('Route calculation failed', 'error');
        console.error(e);
    }
}

function renderRouteCards(routes) {
    const list = $('route-list');
    if (!list) return;
    list.innerHTML = routes.map((r, i) => `
        <div class="route-card ${i === 0 ? 'selected' : ''}" data-idx="${i}" style="background: rgba(255,255,255,0.06); border-radius: 12px; padding: 10px; margin-bottom: 8px; cursor: pointer;">
            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 4px;">
                <span style="font-weight: 700; font-size: 14px; color: white;">${r.name}</span>
                <span class="route-tag ${r.tagClass}" style="font-size: 10px; padding: 2px 6px; border-radius: 6px; background: rgba(52,199,89,0.2); color: #34C759;">${r.tag}</span>
            </div>
            <div style="display: flex; gap: 14px; font-size: 12px; color: var(--text2);">
                <span>📏 ${fmtDist(r.distanceM)}</span>
                <span>⏱ ${fmtTime(r.durationS)}</span>
                <span>⚡ ${Math.round(r.avgSpeedKmh || 40)} km/h</span>
            </div>
        </div>`
    ).join('');

    list.querySelectorAll('.route-card').forEach(card => {
        card.addEventListener('click', () => selectRoute(parseInt(card.dataset.idx)));
    });
}

function selectRoute(idx) {
    S.selectedRouteIdx = idx;
    S.route = S.altRoutes[idx];
    document.querySelectorAll('.route-card').forEach((c, i) => c.classList.toggle('selected', i === idx));
    mapEngine.showAltRoutes(S.altRoutes, idx);
    if (S.route.outageZones) {
        mapEngine.markOutageZones(S.route.coords, S.route.outageZones);
    }
    const info = $('selected-route-info');
    if (info) info.textContent = `${fmtTime(S.route.durationS)} (${fmtDist(S.route.distanceM)})`;
}

// ============================================================
// START NAVIGATION (Requirement I)
// ============================================================
function startNavigation() {
    // 1. Verify real GPS exists
    if (S.currentLat === null || S.currentLng === null) {
        toast('Waiting for GPS location. Navigation cannot start yet.', 'warning', 3500);
        return;
    }
    // 2. Verify destination coordinates exist
    if (!S.destination || S.destination.lat == null) {
        toast('Select a destination first.', 'warning', 3500);
        return;
    }
    // 3. Verify route exists
    if (!S.route) {
        toast('Calculating route first...', 'info');
        planRoute().then(() => startNavigation());
        return;
    }

    // Reset navigation state
    S.isNavigating = true;
    S.isSimulation = false;
    S.isGPSLost = false;
    S.isRecovering = false;
    S.isArrived = false;
    S.routeIdx = 0;
    S.routeProgress = 0;
    S.routeFraction = 0;
    S.totalDistanceM = 0;
    S.maxDrift = 0;
    S.speedErrors = [];
    S.blackoutSeconds = 0;
    S.manualGPSLoss = false;
    datasetIdx = 0;
    lastMatchedRouteIdx = 0;
    clearInterval(S.blackoutTimerId);

    // SPEED MUST NOT BE HARDCODED! Use real GPS speed or 0 if stopped
    // HEADING MUST NOT BE FORCED FROM ROUTE! Use real GPS heading
    // Current location = authoritative real GPS coordinates

    aiModel.reset();
    session.startSession();

    // Map setup
    mapEngine.setRoute(S.route.coords);
    mapEngine.clearPath();
    mapEngine.setMarkers(S.origin, S.destination);
    mapEngine.setFollowCamera(true);

    showScreen('navigation');
    T('nav-dest-name', S.destination.label);

    const sigText = $('nav-signal-text');
    if (sigText) sigText.textContent = 'LIVE GPS';

    const simGrp = $('sim-speed-wrapper');
    if (simGrp) simGrp.style.opacity = '0.35';

    toast('▶ Navigation Started — Live GPS Tracking', 'success', 2500);
    speak('Navigation started. Proceed to highlighted route.');

    startMainLoop();
}

// ============================================================
// DESTINATION SEARCH COMPONENT (Requirements E & F)
// ============================================================
let searchDebounceTimer = null;

function setupSearchComponent() {
    const input = $('dest-input');
    const resultsContainer = $('search-results');
    const clearBtn = $('btn-search-clear');

    if (!input || !resultsContainer) return;

    input.addEventListener('input', () => {
        const query = input.value.trim();
        if (clearBtn) clearBtn.style.display = query ? 'block' : 'none';

        clearTimeout(searchDebounceTimer);
        if (query.length < 2) {
            resultsContainer.style.display = 'none';
            resultsContainer.innerHTML = '';
            return;
        }

        resultsContainer.style.display = 'block';
        resultsContainer.innerHTML = '<div class="search-state-msg">🔍 Searching destinations...</div>';

        // 300ms query debounce
        searchDebounceTimer = setTimeout(async () => {
            try {
                const results = await routing.searchDestination(query);
                if (results.length === 0) {
                    resultsContainer.innerHTML = '<div class="search-state-msg">No matching locations found</div>';
                    return;
                }

                resultsContainer.innerHTML = results.map((item, idx) => {
                    const icon = getPlaceIcon(item.type);
                    const subtitle = [item.locality, item.district, item.state].filter(Boolean).join(', ');
                    return `
                        <div class="search-result-item" data-idx="${idx}">
                            <div class="search-result-icon">${icon}</div>
                            <div class="search-result-body">
                                <div class="search-result-title">${escapeHtml(item.name)}</div>
                                <div class="search-result-subtitle">${escapeHtml(subtitle || item.country)}</div>
                            </div>
                            <span class="search-result-type-tag">${escapeHtml(item.type)}</span>
                        </div>
                    `;
                }).join('');

                resultsContainer.querySelectorAll('.search-result-item').forEach(el => {
                    el.addEventListener('click', () => {
                        const idx = parseInt(el.dataset.idx);
                        const sel = results[idx];
                        if (sel) {
                            selectSearchResult(sel);
                        }
                    });
                });
            } catch (err) {
                resultsContainer.innerHTML = '<div class="search-state-msg">Search unavailable</div>';
            }
        }, 300);
    });

    if (clearBtn) {
        clearBtn.addEventListener('click', () => {
            input.value = '';
            clearBtn.style.display = 'none';
            resultsContainer.style.display = 'none';
            input.focus();
        });
    }

    // Close search dropdown if tapped outside
    document.addEventListener('click', (e) => {
        if (!e.target.closest('#search-container') && !e.target.closest('#search-results')) {
            resultsContainer.style.display = 'none';
        }
    });
}

function getPlaceIcon(type) {
    const t = (type || '').toLowerCase();
    if (t.includes('village')) return '🏡';
    if (t.includes('city') || t.includes('town')) return '🏙️';
    if (t.includes('hospital')) return '🏥';
    if (t.includes('school') || t.includes('university') || t.includes('college')) return '🏫';
    if (t.includes('station') || t.includes('railway') || t.includes('bus')) return '🚉';
    if (t.includes('airport')) return '✈️';
    if (t.includes('temple') || t.includes('church') || t.includes('mosque')) return '🏛️';
    return '📍';
}

function escapeHtml(str) {
    if (!str) return '';
    return str.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
}

function selectSearchResult(item) {
    const input = $('dest-input');
    const resultsContainer = $('search-results');
    if (input) input.value = `📍 ${item.name}`;
    if (resultsContainer) resultsContainer.style.display = 'none';

    // Store authoritative exact coordinates
    S.destination = {
        lat: item.lat,
        lng: item.lng,
        label: item.name,
        address: item.fullAddress,
        type: item.type
    };

    // Ensure origin is set to real GPS coordinates if available
    if (S.currentLat !== null && S.currentLng !== null) {
        S.origin = { lat: S.currentLat, lng: S.currentLng, label: 'Current Location' };
    }

    mapEngine.setMarkers(S.origin, S.destination);
    toast(`📍 Selected: ${item.name}`, 'info');

    // Calculate route immediately
    planRoute();
}

// ============================================================
// PRESETS & MAP TAP
// ============================================================
function buildPresets() {
    const c = $('preset-buttons');
    if (!c) return;
    c.innerHTML = PRESETS.map(p => `
        <button class="preset-btn" data-id="${p.id}">${p.name}</button>`
    ).join('');
    c.querySelectorAll('.preset-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            const p = PRESETS.find(x => x.id === btn.dataset.id);
            if (p) loadPreset(p);
        });
    });
}

async function loadPreset(p) {
    toast(`Loading simulation corridor: ${p.name}...`, 'info');
    S.isSimulation = true;

    // In simulation mode, preset origin is allowed if no GPS exists
    if (!S.origin) {
        S.origin = { ...p.origin };
    }
    S.destination = { ...p.dest };

    const di = $('dest-input');
    if (di) di.value = `📍 ${S.destination.label}`;

    mapEngine.setMarkers(S.origin, S.destination);
    await planRoute(p.outageZones);
}

async function onMapTap(lat, lng) {
    if (!MAP_SCREENS.has(S.screen) || S.isNavigating) return;
    const label = await routing.reverseGeocode(lat, lng);
    selectSearchResult({
        name: label,
        locality: '',
        district: '',
        state: '',
        country: '',
        lat,
        lng,
        type: 'Pinned Location',
        fullAddress: label
    });
}

// ============================================================
// DEBUG PANEL (Refreshed every 200ms)
// ============================================================
function updateDebug() {
    if (S.screen !== 'debug') return;

    const lat = S.isGPSLost ? S.drLat : S.currentLat;
    const lng = S.isGPSLost ? S.drLng : S.currentLng;

    T('dbg-gps-status', S.gpsStatus);
    T('dbg-lat', lat !== null ? lat.toFixed(6) : '—');
    T('dbg-lng', lng !== null ? lng.toFixed(6) : '—');
    T('dbg-accuracy', S.currentAccuracy ? `±${Math.round(S.currentAccuracy)}m` : '—');
    T('dbg-raw-speed', `${S.currentRawSpeed.toFixed(1)} km/h`);
    T('dbg-calc-speed', `${(S.isGPSLost ? S.drSpeed : S.currentSpeed).toFixed(1)} km/h`);
    T('dbg-heading', `${S.currentHeading.toFixed(1)}°`);
    T('dbg-timestamp', S.lastGPSTimestamp ? new Date(S.lastGPSTimestamp).toLocaleTimeString() : '—');
    T('dbg-source', S.isGPSLost ? 'DR (IO-VNBD + AI)' : S.isRecovering ? 'RECOVERY' : S.gpsIsLive ? 'LIVE GPS' : 'SIMULATION');

    T('dbg-dest-coords', S.destination ? `${S.destination.lat.toFixed(5)}, ${S.destination.lng.toFixed(5)}` : 'None');
    T('dbg-route-dist', S.route ? fmtDist(S.route.distanceM) : 'None');
    T('dbg-route-dur', S.route ? fmtTime(S.route.durationS) : 'None');
    T('dbg-route-deviation', `${S.deviationDistance.toFixed(1)} m`);
    T('dbg-network', S.isOffline ? '🔴 OFFLINE' : '🟢 ONLINE');

    T('dbg-ax', `${S.imu.ax.toFixed(3)} m/s²`);
    T('dbg-ay', `${S.imu.ay.toFixed(3)} m/s²`);
    T('dbg-az', `${S.imu.az.toFixed(3)} m/s²`);
    T('dbg-gx', `${S.imu.gx.toFixed(3)} rad/s`);
    T('dbg-gy', `${S.imu.gy.toFixed(3)} rad/s`);
    T('dbg-gz', `${S.imu.gz.toFixed(3)} rad/s`);
    T('dbg-sensor-type', S.imu.isReal ? '📱 Device IMU' : '📊 IO-VNBD');

    T('dbg-dr-active', S.isGPSLost ? `✅ ON` : '❌ OFF');
    T('dbg-blackout-time', `${S.blackoutSeconds.toFixed(1)}s`);
    T('dbg-drift', `${S.drDrift.toFixed(1)}m`);
    T('dbg-ai-conf', `${Math.round(S.drConfidence * 100)}%`);
}

// ============================================================
// EVENT WIRING
// ============================================================
function wireEvents() {
    // "My Location" FAB Button (Requirement D)
    const btnMyLoc = $('btn-my-location');
    if (btnMyLoc) {
        btnMyLoc.addEventListener('click', () => {
            if (S.currentLat !== null && S.currentLng !== null) {
                mapEngine.flyTo(S.currentLat, S.currentLng, 16);
                if (S.isNavigating) mapEngine.setFollowCamera(true);
                toast('🎯 Centered on your real GPS location', 'info', 1500);
            } else {
                toast('Waiting for GPS location...', 'warning', 2500);
            }
        });
    }

    // Start Navigation Button (Requirement I)
    const bs = $('btn-start-nav');
    if (bs) bs.addEventListener('click', startNavigation);

    // End Navigation Button
    const be = $('btn-end-nav');
    if (be) be.addEventListener('click', () => {
        stopMainLoop();
        clearInterval(S.blackoutTimerId);
        S.isGPSLost = false;
        S.isNavigating = false;
        showScreen('route');
    });

    // Manual GPS Toggle Action (for testing outage & recovery)
    const bg = $('btn-sim-gps-action');
    if (bg) bg.addEventListener('click', () => {
        if (!S.isNavigating) {
            toast('Start navigation to test GPS Loss/DR', 'warning');
            return;
        }
        if (S.isGPSLost) {
            triggerGPSRecovery();
            bg.textContent = '📵 GPS Loss';
            bg.className = 'btn-danger btn-sm';
        } else {
            triggerGPSLoss();
            bg.textContent = '🛰️ Restore GPS';
            bg.className = 'btn-success btn-sm';
        }
    });

    // Diagnostics Panel Toggle
    const drBadge = document.querySelector('.dr-floating-badge');
    const diagPanel = $('diagnostics-panel');
    if (drBadge && diagPanel) {
        drBadge.addEventListener('click', () => {
            diagPanel.style.display = (diagPanel.style.display === 'none' || diagPanel.style.display === '') ? 'block' : 'none';
        });
    }

    // Route back to home
    const rb = $('btn-route-back');
    if (rb) rb.addEventListener('click', () => showScreen('home'));

    // Debug Panel Toggle
    const bd = $('btn-debug');
    if (bd) bd.addEventListener('click', () => showScreen('debug'));

    const bdb = $('btn-debug-back');
    if (bdb) bdb.addEventListener('click', () => {
        showScreen(S.isNavigating ? (S.isGPSLost ? 'dr' : 'navigation') : 'home');
    });

    // Network Connectivity Listeners
    window.addEventListener('offline', () => {
        S.isOffline = true;
        const netBadge = $('net-badge');
        if (netBadge) { netBadge.textContent = '● OFFLINE'; netBadge.className = 'net-badge offline'; }
        toast('📡 Network Offline — Continuing on loaded route', 'warning', 4000);
    });
    window.addEventListener('online', () => {
        S.isOffline = false;
        const netBadge = $('net-badge');
        if (netBadge) { netBadge.textContent = '● ONLINE'; netBadge.className = 'net-badge online'; }
        toast('📡 Network Restored', 'success', 2500);
    });

    // Results Actions
    const btnResults = $('btn-view-results');
    if (btnResults) btnResults.addEventListener('click', () => {
        const overlay = $('arrival-overlay');
        if (overlay) overlay.style.display = 'none';
        showResultsScreen();
    });

    const rh = $('btn-results-home');
    if (rh) rh.addEventListener('click', () => {
        session.reset();
        S.isNavigating = false;
        S.isGPSLost = false;
        S.route = null;
        S.destination = null;
        S.isArrived = false;
        showScreen('home');
        mapEngine.clearPath();
    });

    const ra = $('btn-results-again');
    if (ra) ra.addEventListener('click', () => {
        S.isArrived = false;
        showScreen('route');
        selectRoute(S.selectedRouteIdx);
    });

    // Simulation Speed Multipliers (Explicitly disabled in Live GPS mode)
    document.querySelectorAll('.sim-speed-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            if (S.gpsIsLive) {
                toast('Simulation speed is disabled during Live GPS tracking', 'warning');
                return;
            }
            S.simSpeed = parseFloat(btn.dataset.speed);
            document.querySelectorAll('.sim-speed-btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
        });
    });
}

// ============================================================
// APPLICATION INITIALIZATION
// ============================================================
async function init() {
    // 1. Initialize map at neutral overview
    mapEngine.init(20.5937, 78.9629, 5);
    mapEngine.enableDestinationPick(onMapTap);

    showScreen('home');

    // 2. Setup Destination Search Component
    setupSearchComponent();

    // 3. Start Geolocation Watcher
    gpsEngine.start(onGPSUpdate, onGPSStatus, onGPSError);

    // 4. Start Device Motion Sensors
    sensors.start(onSensorUpdate);

    // 5. Build Demo Corridor Buttons
    buildPresets();

    // 6. Wire Events
    wireEvents();

    // 7. Background Clock & Debug Refresh Loop
    setInterval(() => {
        const now = new Date();
        T('status-clock', `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}`);
        updateDebug();
    }, 200);
}

document.addEventListener('DOMContentLoaded', init);

// app.js — NAVISENSE-IDR Main Application
// SIH 2026 — Problem Statement 26168
//
// CORE PRINCIPLE: One continuous rAF loop drives everything.
// GPS Active  → Replay engine advances route position → map updated every frame
// GPS Lost    → IO-VNBD dataset IMU → AI GRU → EKF DR → map matching → position updated every frame
// GPS Recover → Smooth blend (DR pos → GPS pos over 3 sec) → map updated every frame

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

// IO-VNBD dataset cycling index (loops the 500 samples during DR)
let datasetIdx = 0;

// ============================================================
// APPLICATION STATE
// ============================================================
const S = {
    screen: 'home',
    // Navigation
    isNavigating: false,
    isGPSLost: false,
    isRecovering: false,
    isArrived: false,
    // Position
    currentLat: 28.6139,
    currentLng: 77.2090,
    currentSpeed: 0,
    currentHeading: 0,
    currentAccuracy: 5,
    // Route
    origin: null,
    destination: null,
    route: null,
    altRoutes: [],
    selectedRouteIdx: 0,
    // Route traversal
    routeIdx: 0,          // Current segment index in route coords
    routeProgress: 0.0,   // Progress within current segment [0..1]
    routeFraction: 0.0,   // Overall route fraction [0..1]
    // DR State
    drLat: 0, drLng: 0,
    drSpeed: 0, drHeading: 0,
    drConfidence: 0.95,
    drDrift: 0,
    blackoutSeconds: 0,
    blackoutTimerId: null,
    // Recovery
    recoveryStartTime: 0,
    recoveryDuration: 3000,
    drLatAtRecovery: 0, drLngAtRecovery: 0,
    gpsLatAtRecovery: 0, gpsLngAtRecovery: 0,
    // Session metrics
    totalDistanceM: 0,
    lastPosForDist: null,
    maxDrift: 0,
    speedErrors: [],
    outageStartTime: 0,
    sessionStartTime: 0,
    // Sensor data (live or synthetic)
    imu: { ax: 0, ay: 0, az: 9.81, gx: 0, gy: 0, gz: 0, isReal: false },
    // Simulation speed multiplier
    simSpeed: 1.0,
    // GPS engine status
    gpsStatus: 'ACQUIRING',
    gpsIsLive: false,
    // Manual GPS loss
    manualGPSLoss: false,
    // AI GRU state (hidden states)
    aiH1: null, aiH2: null,
};

// ============================================================
// ENGINE INSTANCES
// ============================================================
const gpsEngine    = new GPSEngine();
const sensors      = new SensorEngine();
const aiModel      = new AIModel();
const mapEngine    = new MapEngine('map');
const routing      = new RoutingEngine();
const session      = new SessionLog();

// ============================================================
// PRESET CORRIDORS
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
const MAP_SCREENS = new Set(['home','route','navigation','dr','recovery']);

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
    if (MAP_SCREENS.has(name)) setTimeout(() => mapEngine.invalidateSize(), 150);
}

const $  = id => document.getElementById(id);
const T  = (id, v) => { const el = $(id); if (el) el.textContent = v; };
const H  = (id, v) => { const el = $(id); if (el) el.innerHTML = v; };

function toast(msg, type = 'info', ms = 3000) {
    const el = $('toast');
    if (!el) return;
    el.textContent = msg;
    el.className = `toast toast-${type} show`;
    clearTimeout(el._t);
    el._t = setTimeout(() => el.classList.remove('show'), ms);
}

function fmtDist(m) {
    return m >= 1000 ? `${(m/1000).toFixed(1)} km` : `${Math.round(m)} m`;
}
function fmtTime(s) {
    if (s < 60) return `${Math.round(s)}s`;
    if (s < 3600) return `${Math.round(s/60)} min`;
    return `${Math.floor(s/3600)}h ${Math.round((s%3600)/60)}m`;
}
function fmtETA(s) {
    const d = new Date(Date.now() + s * 1000);
    const h = d.getHours(), m = d.getMinutes().toString().padStart(2,'0');
    return `${(h%12)||12}:${m} ${h>=12?'PM':'AM'}`;
}
function speak(t) {
    if (!window.speechSynthesis) return;
    const u = new SpeechSynthesisUtterance(t);
    u.lang = 'en-IN'; u.rate = 0.95;
    window.speechSynthesis.cancel();
    window.speechSynthesis.speak(u);
}
function beep(type) {
    try {
        const ctx = new (window.AudioContext || window.webkitAudioContext)();
        const now = ctx.currentTime, osc = ctx.createOscillator(), g = ctx.createGain();
        osc.type = type === 'loss' ? 'triangle' : 'sine';
        if (type === 'loss') {
            osc.frequency.setValueAtTime(440, now);
            osc.frequency.linearRampToValueAtTime(220, now+0.4);
        } else {
            osc.frequency.setValueAtTime(523, now);
            osc.frequency.setValueAtTime(784, now+0.2);
        }
        g.gain.setValueAtTime(0.3, now);
        g.gain.exponentialRampToValueAtTime(0.001, now+0.5);
        osc.connect(g); g.connect(ctx.destination);
        osc.start(now); osc.stop(now+0.5);
    } catch(e) {}
}

// ============================================================
// MAIN NAVIGATION LOOP (single rAF loop drives everything)
// ============================================================
let lastFrameTime = null;
let rafId = null;

function startMainLoop() {
    lastFrameTime = performance.now();
    function loop(now) {
        if (!S.isNavigating) return;
        const dt = Math.min((now - lastFrameTime) / 1000, 0.1); // cap at 100ms
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
}

function tick(dt) {
    if (!S.route || !S.route.coords) return;

    // -------- ROUTE ADVANCEMENT --------
    // Advance route position based on current speed
    const coords = S.route.coords;
    const speedMs = (S.currentSpeed / 3.6) * S.simSpeed;
    const distThisFrame = speedMs * dt;

    advanceRoute(coords, distThisFrame);

    // Feed vehicle state to sensor engine so synthetic IMU works
    sensors.updateVehicleState(S.currentSpeed, S.currentHeading, S.isGPSLost);

    // -------- POSITION BASED ON MODE --------
    let displayLat, displayLng, displayMode;

    if (S.isRecovering) {
        // Smooth blend from DR position back to GPS
        const elapsed = Date.now() - S.recoveryStartTime;
        const t = Math.min(1, elapsed / S.recoveryDuration);
        const eased = t < 0.5 ? 2*t*t : -1+(4-2*t)*t;

        displayLat = S.drLatAtRecovery + (S.gpsLatAtRecovery - S.drLatAtRecovery) * eased;
        displayLng = S.drLngAtRecovery + (S.gpsLngAtRecovery - S.drLngAtRecovery) * eased;
        displayMode = 'RECOVERY';

        S.drConfidence = 0.4 + eased * 0.55;

        if (t >= 1) {
            // Recovery complete
            S.isRecovering = false;
            S.currentLat = S.gpsLatAtRecovery;
            S.currentLng = S.gpsLngAtRecovery;
            session.endOutage(haversine(S.drLatAtRecovery, S.drLngAtRecovery, S.gpsLatAtRecovery, S.gpsLngAtRecovery), S.recoveryDuration);
            showScreen('navigation');
            toast('✅ GPS Restored — Realigning complete', 'success');
        }

    } else if (S.isGPSLost) {
        // ======== DEAD RECKONING USING IO-VNBD DATASET + AI + EKF ========
        // 1. Get next sensor sample from IO-VNBD dataset (cycled)
        const sample = IOVNBD_DATA[datasetIdx % IOVNBD_DATA.length];
        datasetIdx++;

        // 2. Use real device sensors if available, else IO-VNBD data
        const imu = S.imu.isReal ? S.imu : sample;

        // 3. Run AI GRU model (input: IMU + speed + dt)
        const aiOut = aiModel.infer(imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, dt, S.drSpeed);

        // 4. Physics integration (EKF-style)
        //    Speed update: physics from accel + AI correction
        const axForward = imu.ax * Math.cos(S.drHeading * DEG2RAD)
                        + imu.ay * Math.sin(S.drHeading * DEG2RAD);
        const physDv = axForward * dt * 3.6 * 0.3;  // physical contribution (30%)
        const aiDv   = aiOut.deltaSpeed * dt * 10;   // AI contribution scaled

        let newSpeed = S.drSpeed + physDv + aiDv;
        newSpeed = Math.max(5, Math.min(120, newSpeed)); // Clamp: never stop, never fly

        //    Heading update: gyroscope + AI
        const physDh = imu.gz * dt * RAD2DEG * 0.4;   // gyro contribution
        const aiDh   = aiOut.deltaHeading * dt * 5;    // AI contribution
        let newHeading = (S.drHeading + physDh + aiDh + 360) % 360;

        //    Position integration
        const avgSpeedMs = (S.drSpeed + newSpeed) / 2 / 3.6;
        const distM = avgSpeedMs * dt * S.simSpeed;

        const hRad = newHeading * DEG2RAD;
        const dlat = (distM * Math.cos(hRad)) / 111320;
        const dlng = (distM * Math.sin(hRad)) / (111320 * Math.cos(S.drLat * DEG2RAD));

        let newLat = S.drLat + dlat;
        let newLng = S.drLng + dlng;

        // 5. Map matching — soft-snap to nearest route point within 60m
        const matched = mapMatch(newLat, newLng, coords);
        if (matched) {
            newLat = newLat * 0.2 + matched.lat * 0.8;
            newLng = newLng * 0.2 + matched.lng * 0.8;
            newHeading = newHeading * 0.3 + matched.heading * 0.7;
        }

        // 6. Update DR state
        S.drLat = newLat;
        S.drLng = newLng;
        S.drSpeed = newSpeed;
        S.drHeading = newHeading;
        S.drDrift = 0.5 * 0.08 * S.blackoutSeconds * S.blackoutSeconds; // EKF drift model
        S.drConfidence = Math.max(0.25, aiOut.confidence * Math.max(0.3, 1 - S.blackoutSeconds / 90));
        if (S.drDrift > S.maxDrift) S.maxDrift = S.drDrift;

        // Log DR point for session metrics
        session.logDR(newLat, newLng, newSpeed, newHeading, S.drConfidence, S.drDrift, S.blackoutSeconds);

        S.currentSpeed = newSpeed;
        S.currentHeading = newHeading;

        displayLat = newLat;
        displayLng = newLng;
        displayMode = 'DR';

        // Update DR HUD
        updateDRHud(imu);

    } else {
        // GPS Active: use interpolated route position
        const pt = getRoutePosition(coords);
        S.currentLat = pt.lat;
        S.currentLng = pt.lng;
        displayLat = pt.lat;
        displayLng = pt.lng;
        displayMode = 'GPS';

        // Update heading from route direction
        const nextIdx = Math.min(S.routeIdx + 1, coords.length - 1);
        S.currentHeading = computeBearing(
            coords[S.routeIdx].lat, coords[S.routeIdx].lng,
            coords[nextIdx].lat, coords[nextIdx].lng
        );
    }

    // -------- UPDATE MAP --------
    mapEngine.updateVehicle(displayLat, displayLng, S.currentHeading, S.currentAccuracy, displayMode);

    // -------- UPDATE NAVIGATION HUD --------
    updateNavHUD(displayLat, displayLng, coords);

    // -------- CHECK OUTAGE ZONE (auto trigger) --------
    if (!S.isGPSLost && !S.isRecovering && S.route.outageZones) {
        let inside = false;
        for (const z of S.route.outageZones) {
            if (S.routeFraction >= z.range[0] && S.routeFraction < z.range[1]) {
                inside = true; break;
            }
        }
        if (inside && !S.manualGPSLoss) {
            triggerGPSLoss();
        }
    } else if (S.isGPSLost && !S.manualGPSLoss && S.route.outageZones) {
        let inside = false;
        for (const z of S.route.outageZones) {
            if (S.routeFraction >= z.range[0] && S.routeFraction < z.range[1]) {
                inside = true; break;
            }
        }
        if (!inside) {
            triggerGPSRecovery();
        }
    }

    // -------- CHECK ARRIVAL --------
    const remaining = remainingDistance(displayLat, displayLng, coords);
    if (remaining < 25 && !S.isArrived) {
        onArrived();
    }
}

// ============================================================
// ROUTE ADVANCEMENT
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
    if (!coords || coords.length < 2) return { lat: S.currentLat, lng: S.currentLng };
    const i = Math.min(S.routeIdx, coords.length - 2);
    const p = Math.min(1, S.routeProgress);
    return interpolate(coords[i].lat, coords[i].lng, coords[i+1].lat, coords[i+1].lng, p);
}

function remainingDistance(lat, lng, coords) {
    if (!coords || coords.length === 0) return 0;
    let d = 0;
    for (let i = S.routeIdx; i < coords.length - 1; i++) {
        d += haversine(coords[i].lat, coords[i].lng, coords[i+1].lat, coords[i+1].lng);
    }
    return d;
}

// ============================================================
// MAP MATCHING — find nearest route point within threshold
// ============================================================
let lastMatchedRouteIdx = 0;
function mapMatch(lat, lng, coords) {
    if (!coords || coords.length < 2) return null;
    let minDist = 80; // 80m max snap radius
    let bestIdx = -1;
    const start = Math.max(0, lastMatchedRouteIdx - 3);
    const end   = Math.min(coords.length - 1, lastMatchedRouteIdx + 40);

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
// GPS LOSS & RECOVERY
// ============================================================
function triggerGPSLoss() {
    if (S.isGPSLost) return;
    S.isGPSLost = true;
    S.manualGPSLoss = true;
    S.outageStartTime = Date.now();
    S.blackoutSeconds = 0;
    datasetIdx = 0; // Reset dataset to start fresh

    // Initialise DR from current GPS position
    S.drLat = S.currentLat;
    S.drLng = S.currentLng;
    S.drSpeed = Math.max(S.currentSpeed, 20); // minimum 20 km/h in DR
    S.drHeading = S.currentHeading;
    S.drConfidence = 0.95;
    S.drDrift = 0;
    lastMatchedRouteIdx = S.routeIdx;

    aiModel.reset();
    session.startOutage();

    // Start blackout timer (runs independently of rAF)
    clearInterval(S.blackoutTimerId);
    S.blackoutTimerId = setInterval(() => {
        S.blackoutSeconds += 0.1;
        T('dr-blackout-timer', S.blackoutSeconds.toFixed(1) + 's');
        T('dr-drift-display', S.drDrift.toFixed(2) + 'm');
    }, 100);

    showScreen('dr');
    toast('🔴 GPS SIGNAL LOST — Dead Reckoning Active', 'error', 5000);
    beep('loss');
    speak('GPS signal lost. Switching to intelligent dead reckoning using IO-VNBD sensor data.');
}

function triggerGPSRecovery() {
    if (!S.isGPSLost) return;
    S.isGPSLost = false;

    clearInterval(S.blackoutTimerId);
    S.blackoutTimerId = null;

    const finalDrift = haversine(S.drLat, S.drLng, S.currentLat, S.currentLng);
    const dur = (Date.now() - S.outageStartTime) / 1000;

    // Set up recovery interpolation
    S.isRecovering = true;
    S.recoveryStartTime = Date.now();
    S.drLatAtRecovery  = S.drLat;
    S.drLngAtRecovery  = S.drLng;

    // GPS target: current route position (where we'd actually be)
    const pt = getRoutePosition(S.route.coords);
    S.gpsLatAtRecovery = pt.lat;
    S.gpsLngAtRecovery = pt.lng;

    showScreen('recovery');
    T('recovery-duration', dur.toFixed(1) + ' s');
    T('recovery-drift',    finalDrift.toFixed(1) + ' m');
    T('recovery-dist',     (S.totalDistanceM / 1000).toFixed(2) + ' km');

    toast('✅ GPS Signal Restored — Realigning...', 'success', 4000);
    beep('restore');
    speak('NavIC signal restored. Realigning dead reckoning position.');
}

// ============================================================
// HUD UPDATES
// ============================================================
function updateNavHUD(lat, lng, coords) {
    const rem  = remainingDistance(lat, lng, coords);
    const spd  = S.isGPSLost ? S.drSpeed : S.currentSpeed;
    const etaS = spd > 1 ? (rem / (spd / 3.6)) : (rem / (50/3.6));

    if (!S.isGPSLost && !S.isRecovering) {
        T('bm-1-lbl', 'Speed');
        T('bm-1-val', Math.round(spd) + ' km/h');
        T('bm-2-lbl', 'Distance');
        T('bm-2-val', fmtDist(rem));
        T('bm-3-lbl', 'ETA');
        T('bm-3-val', fmtTime(etaS));
        
        T('current-mode-pill', 'Normal GPS');
        const pill = $('current-mode-pill');
        if (pill) { pill.className = 'mode-pill gps'; }
        
        const confContainer = $('conf-info-container');
        if (confContainer) confContainer.style.display = 'none';
    }

    // Maneuver from route steps
    if (S.route && S.route.steps) {
        const stepIdx = Math.min(Math.floor(S.routeFraction * S.route.steps.length), S.route.steps.length - 1);
        const step = S.route.steps[stepIdx];
        if (step) {
            let instr = step.instruction;
            if (step.distance && step.distance !== '0 m') instr += ' in ' + step.distance;
            T('maneuver-icon', maneuverIcon(step.instruction));
            T('maneuver-instruction', instr);
        }
    }

    // IMU sensor display
    const imu = S.imu;
    T('hud-ax', imu.ax.toFixed(2));
    T('hud-ay', imu.ay.toFixed(2));
    T('hud-gz', imu.gz.toFixed(2));
}

function formatDuration(sec) {
    const m = Math.floor(sec / 60);
    const s = Math.floor(sec % 60);
    return m.toString().padStart(2, '0') + ':' + s.toString().padStart(2, '0');
}

function updateDRHud(imu) {
    // DR Screen HUD -> Unified Bottom Sheet
    const dur = (Date.now() - S.outageStartTime) / 1000;
    const conf = Math.round(S.drConfidence * 100);
    const rem = remainingDistance(S.drLat, S.drLng, S.route ? S.route.coords : []);
    
    T('bm-1-lbl', 'Speed');
    T('bm-1-val', Math.round(S.drSpeed) + ' km/h');
    T('bm-2-lbl', 'Distance');
    T('bm-2-val', fmtDist(rem));
    T('bm-3-lbl', 'Blackout Time');
    T('bm-3-val', formatDuration(dur));
    
    T('current-mode-pill', 'Intelligent DR');
    const pill = $('current-mode-pill');
    if (pill) { pill.className = 'mode-pill dr'; }
    
    const confContainer = $('conf-info-container');
    if (confContainer) confContainer.style.display = 'flex';
    T('bm-conf-val', conf + '%');

    // Also keep maneuver HUD updated in DR screen
    updateNavHUD(S.drLat, S.drLng, S.route ? S.route.coords : []);
}



function maneuverIcon(instr) {
    const t = (instr || '').toLowerCase();
    if (t.includes('right')) return '↱';
    if (t.includes('left'))  return '↰';
    if (t.includes('u-turn'))return '↩';
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

    toast('🏁 Arrived at destination!', 'success', 5000);
    speak('You have arrived at your destination.');
    setTimeout(() => showResultsScreen(), 1500);
}

// ============================================================
// RESULTS SCREEN
// ============================================================
function showResultsScreen() {
    showScreen('results');
    const r = session.getResults();
    T('res-blackout',    r.gpsBlackoutDuration + ' s');
    T('res-distance',    r.travelledDistanceKm + ' km');
    T('res-pos-error',   r.positionErrorM + ' m');
    T('res-max-drift',   r.maxDriftM + ' m');
    T('res-speed-error', r.speedErrorKmh + ' km/h');
    T('res-heading-error',r.headingErrorDeg + '°');
    T('res-recovery',    r.recoveryTimeSecs + ' s');
    T('res-confidence',  r.overallConfidence + '%');
    T('res-score',       r.overallScore);
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
        ctx.fillText('DR phase data will appear here', W/2, H/2);
        return;
    }

    // Confidence line (green)
    ctx.strokeStyle = '#10B981'; ctx.lineWidth = 2; ctx.setLineDash([]);
    ctx.beginPath();
    pts.forEach((p, i) => {
        const x = (i / (pts.length - 1)) * W;
        const y = H - p.confidence * (H - 16) - 8;
        i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.stroke();

    // Drift line (red dashed)
    const maxD = Math.max(...pts.map(p => p.drift || 0), 1);
    ctx.strokeStyle = '#EF4444'; ctx.lineWidth = 1.5; ctx.setLineDash([4,2]);
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
// GPS ENGINE CALLBACKS
// ============================================================
function onGPSUpdate(pos) {
    S.gpsIsLive = true;
    S.currentLat = pos.lat;
    S.currentLng = pos.lng;
    S.currentSpeed = pos.speed || 0;
    S.currentHeading = pos.bearing || 0;
    S.currentAccuracy = pos.accuracy || 5;

    // Update home screen coords display
    if (S.screen === 'home') {
        T('gps-lat', pos.lat.toFixed(5));
        T('gps-lng', pos.lng.toFixed(5));
        T('gps-acc', `±${Math.round(pos.accuracy)}m`);
        T('gps-status-text', `📍 GPS Locked (±${Math.round(pos.accuracy)}m)`);
        const el = $('gps-status-text');
        if (el) el.classList.add('locked');
        mapEngine.updateVehicle(pos.lat, pos.lng, pos.bearing||0, pos.accuracy, 'GPS');
    }

    if (!S.origin) {
        S.origin = { lat: pos.lat, lng: pos.lng, label: 'My Location' };
        routing.reverseGeocode(pos.lat, pos.lng).then(label => {
            S.origin.label = label;
            const el = $('origin-input');
            if (el && el.value.startsWith('📍')) el.value = `📍 ${label}`;
        });
        const el = $('origin-input');
        if (el) el.value = `📍 ${pos.lat.toFixed(4)}, ${pos.lng.toFixed(4)}`;
    }
}

function onGPSStatus(status) {
    S.gpsStatus = status;
    const badge = $('gps-badge');
    if (!badge) return;
    if (status === 'LOCKED') {
        badge.textContent = '📡 GPS LOCKED'; badge.className = 'gps-badge locked';
    } else if (status === 'ACQUIRING') {
        badge.textContent = '🔍 ACQUIRING...'; badge.className = 'gps-badge acquiring';
    } else {
        badge.textContent = '📱 SIMULATION'; badge.className = 'gps-badge simulation';
    }
}

function onSensorUpdate(data) {
    S.imu = data;
}

// ============================================================
// ROUTING & NAVIGATION START
// ============================================================
async function planRoute(outageZones) {
    if (!S.origin || !S.destination) {
        toast('Set origin & destination first', 'warning'); return;
    }
    T('route-status', '🔍 Calculating routes via OSRM...');
    showScreen('route');

    try {
        const routes = await routing.getAlternativeRoutes(S.origin, S.destination);
        S.altRoutes = routes;
        routes.forEach(r => { 
            if (!r.outageZones) {
                r.outageZones = outageZones || [{ range: [0.35, 0.65], type: 'urban' }];
            }
        });

        renderRouteCards(routes);
        mapEngine.showAltRoutes(routes, 0);
        mapEngine.setMarkers(S.origin, S.destination);
        selectRoute(0);
        T('route-status', `✅ ${routes.length} routes found`);
    } catch(e) {
        toast('Route failed. Using fallback.', 'error');
        console.error(e);
    }
}

function renderRouteCards(routes) {
    const list = $('route-list');
    if (!list) return;
    list.innerHTML = routes.map((r, i) => `
        <div class="route-card ${i===0?'selected':''}" data-idx="${i}">
            <div class="route-card-header">
                <span class="route-name">${r.name}</span>
                <span class="route-tag ${r.tagClass}">${r.tag}</span>
            </div>
            <div class="route-card-info">
                <span>📏 ${fmtDist(r.distanceM)}</span>
                <span>⏱ ${fmtTime(r.durationS)}</span>
                <span>⚡ ${Math.round(r.avgSpeedKmh)} km/h</span>
            </div>
            <div class="route-outage-warning">
                ${r.outageZones.map(z => `⚠️ <b>${z.type.toUpperCase()}</b>: ~${((z.range[1]-z.range[0])*r.distanceM/1000).toFixed(1)} km`).join('<br>')}
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
    document.querySelectorAll('.route-card').forEach((c,i) => c.classList.toggle('selected', i===idx));
    mapEngine.showAltRoutes(S.altRoutes, idx);
    mapEngine.markOutageZones(S.route.coords, S.route.outageZones);
    const info = $('selected-route-info');
    if (info) info.textContent = `${S.route.name} · ${fmtDist(S.route.distanceM)} · ${fmtTime(S.route.durationS)}`;
}

function startNavigation() {
    if (!S.route) { toast('Select a route first', 'warning'); return; }

    // Reset state
    S.isNavigating = false;
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

    // Starting position = origin
    S.currentLat = S.origin.lat;
    S.currentLng = S.origin.lng;
    S.currentSpeed = S.route.avgSpeedKmh || 40; // Start moving immediately
    S.currentHeading = computeBearing(
        S.route.coords[0].lat, S.route.coords[0].lng,
        S.route.coords[1].lat, S.route.coords[1].lng
    );

    aiModel.reset();
    session.startSession();

    // Setup map
    mapEngine.setRoute(S.route.coords);
    mapEngine.clearPath();
    mapEngine.setMarkers(S.origin, S.destination);
    mapEngine.markOutageZones(S.route.coords, S.route.outageZones);
    // Removed flyTo to prevent animation conflict with the tick loop's setView

    showScreen('navigation');
    T('nav-dest-name', S.destination.label);
    T('nav-route-name', S.route.name);
    T('nav-total-dist', fmtDist(S.route.distanceM));

    const badge = $('nav-mode-badge');
    if (badge) { badge.textContent = '📡 GPS ACTIVE'; badge.className = 'mode-badge live'; }

    toast('▶ Navigation Started — Vehicle Moving', 'success');
    speak('Navigation started. Proceed along the highlighted route.');

    // Start the continuous main loop
    S.isNavigating = true;
    startMainLoop();
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
    toast(`Loading ${p.name}...`, 'info');
    S.origin = { ...p.origin };
    S.destination = { ...p.dest };
    const oi = $('origin-input'), di = $('dest-input');
    if (oi) oi.value = `📍 ${p.origin.label}`;
    if (di) di.value = `📍 ${p.dest.label}`;
    mapEngine.setMarkers(S.origin, S.destination);
    mapEngine.flyTo(p.origin.lat, p.origin.lng, 13);
    await planRoute(p.outageZones);
}

async function onMapTap(lat, lng) {
    if (!MAP_SCREENS.has(S.screen)) return;
    const label = await routing.reverseGeocode(lat, lng);
    S.destination = { lat, lng, label };
    const di = $('dest-input');
    if (di) di.value = `📍 ${label}`;
    mapEngine.setMarkers(S.origin, S.destination);
    toast('📍 Destination set — tap again or search', 'info');
    if (S.origin) await planRoute();
}

// ============================================================
// DEBUG PANEL (refreshed every 200ms)
// ============================================================
function updateDebug() {
    if (S.screen !== 'debug') return;
    const lat = S.isGPSLost ? S.drLat : S.currentLat;
    const lng = S.isGPSLost ? S.drLng : S.currentLng;
    T('dbg-lat',   lat.toFixed(6));
    T('dbg-lng',   lng.toFixed(6));
    T('dbg-speed', `${(S.isGPSLost ? S.drSpeed : S.currentSpeed).toFixed(1)} km/h`);
    T('dbg-heading', `${S.currentHeading.toFixed(1)}°`);
    T('dbg-source', S.isGPSLost ? 'DR (IO-VNBD + AI)' : S.isRecovering ? 'RECOVERY' : 'GPS');
    T('dbg-ax', `${S.imu.ax.toFixed(3)} m/s²`);
    T('dbg-ay', `${S.imu.ay.toFixed(3)} m/s²`);
    T('dbg-az', `${S.imu.az.toFixed(3)} m/s²`);
    T('dbg-gx', `${S.imu.gx.toFixed(3)} rad/s`);
    T('dbg-gy', `${S.imu.gy.toFixed(3)} rad/s`);
    T('dbg-gz', `${S.imu.gz.toFixed(3)} rad/s`);
    T('dbg-sensor-type', S.imu.isReal ? '📱 Real Device' : '📊 IO-VNBD Dataset');
    T('dbg-gps-status', S.gpsStatus);
    T('dbg-dr-active', S.isGPSLost ? `✅ ${S.blackoutSeconds.toFixed(1)}s` : '❌ Off');
    T('dbg-session-gps', session.gpsPoints.length);
    T('dbg-session-dr',  session.drPoints.length);
}

// ============================================================
// EVENT WIRING
// ============================================================
function wireEvents() {
    // Destination search
    const di = $('dest-input');
    if (di) di.addEventListener('keydown', async e => {
        if (e.key !== 'Enter') return;
        const q = di.value.trim();
        if (!q || q.startsWith('📍')) return;
        toast('🔍 Searching...', 'info');
        const res = await routing.geocode(q);
        if (res.length) {
            S.destination = res[0];
            di.value = `📍 ${res[0].label}`;
            mapEngine.setMarkers(S.origin, S.destination);
            await planRoute();
        } else toast('Location not found', 'warning');
    });

    // Origin search
    const oi = $('origin-input');
    if (oi) oi.addEventListener('keydown', async e => {
        if (e.key !== 'Enter') return;
        const q = oi.value.trim();
        if (!q || q.startsWith('📍')) return;
        const res = await routing.geocode(q);
        if (res.length) {
            S.origin = res[0];
            oi.value = `📍 ${res[0].label}`;
        }
    });

    // Start Navigation
    const bs = $('btn-start-nav');
    if (bs) bs.addEventListener('click', startNavigation);

    // End Navigation
    const be = $('btn-end-nav');
    if (be) be.addEventListener('click', () => {
        stopMainLoop();
        clearInterval(S.blackoutTimerId);
        S.isGPSLost = false;
        showScreen('route');
    });

    // Manual GPS Toggle Action
    const bg = $('btn-sim-gps-action');
    if (bg) bg.addEventListener('click', () => {
        if (!S.isNavigating) return;
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
            if (diagPanel.style.display === 'none' || diagPanel.style.display === '') {
                diagPanel.style.display = 'block';
            } else {
                diagPanel.style.display = 'none';
            }
        });
    }

    // Route back
    const rb = $('btn-route-back');
    if (rb) rb.addEventListener('click', () => showScreen('home'));

    // Recenter
    const rc = $('btn-recenter');
    if (rc) rc.addEventListener('click', () => {
        const lat = S.isGPSLost ? S.drLat : S.currentLat;
        const lng = S.isGPSLost ? S.drLng : S.currentLng;
        mapEngine.flyTo(lat, lng);
    });

    // Debug
    const bd = $('btn-debug');
    if (bd) bd.addEventListener('click', () => showScreen('debug'));

    const bdb = $('btn-debug-back');
    if (bdb) bdb.addEventListener('click', () => showScreen(S.isNavigating ? (S.isGPSLost ? 'dr' : 'navigation') : 'home'));

    // Results actions
    const rh = $('btn-results-home');
    if (rh) rh.addEventListener('click', () => {
        session.reset(); S.isNavigating=false; S.isGPSLost=false;
        S.route=null; S.destination=null; S.isArrived=false;
        showScreen('home');
        mapEngine.clearPath();
    });

    const ra = $('btn-results-again');
    if (ra) ra.addEventListener('click', () => {
        S.isArrived = false;
        showScreen('route');
        selectRoute(S.selectedRouteIdx);
    });

    // Sim speed buttons
    document.querySelectorAll('.sim-speed-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            S.simSpeed = parseFloat(btn.dataset.speed);
            document.querySelectorAll('.sim-speed-btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
        });
    });
}

// ============================================================
// INIT
// ============================================================
async function init() {
    // Boot map immediately
    mapEngine.init(28.6139, 77.2090, 14);
    mapEngine.enableDestinationPick(onMapTap);

    showScreen('home');

    // Start GPS
    T('gps-status-text', '🔍 Acquiring GPS...');
    gpsEngine.start(onGPSUpdate, onGPSStatus);

    // Start sensors
    sensors.start(onSensorUpdate);

    // Build preset buttons
    buildPresets();

    // Wire all UI events
    wireEvents();

    // Clock + debug refresh
    setInterval(() => {
        const now = new Date();
        T('status-clock', `${now.getHours().toString().padStart(2,'0')}:${now.getMinutes().toString().padStart(2,'0')}`);
        updateDebug();
    }, 200);
}

document.addEventListener('DOMContentLoaded', init);

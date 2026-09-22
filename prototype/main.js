import { Config } from './services/config.js';

document.addEventListener('DOMContentLoaded', () => {
    const setupScreen = document.getElementById('setup-screen');
    
    // Bypass the API key screen entirely and force SIMULATION mode for demo
    // The user wants a working prototype that simulates movement from A to B (e.g. Chennai to Bangalore)
    if (setupScreen) {
        setupScreen.style.display = 'none';
    }
    
    Config.setLiveMode(false);
    document.getElementById('speed-source').innerText = 'SIMULATED';
    document.getElementById('speed-source').style.background = '#D97706';
    console.log("Auto-started in SIMULATION Mode.");
});

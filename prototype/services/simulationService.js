export const SimulationService = {
    isOutageActive: false,
    isManualGpsOff: false,
    blackoutSeconds: 0.0,
    blackoutTimerInterval: null,
    
    toggleManualGpsCutoff: (isCutoff) => {
        SimulationService.isManualGpsOff = isCutoff;
        return SimulationService.isManualGpsOff;
    },
    
    toggleOutage: (isActive) => {
        SimulationService.isOutageActive = isActive;
        return SimulationService.isOutageActive;
    },
    
    handleBlackoutTimer: (isActive, updateCallback) => {
        if (isActive) {
            if (!SimulationService.blackoutTimerInterval) {
                SimulationService.blackoutSeconds = 0.0;
                SimulationService.blackoutTimerInterval = setInterval(() => {
                    SimulationService.blackoutSeconds += 0.1;
                    if (updateCallback) updateCallback(SimulationService.blackoutSeconds);
                }, 100);
            }
        } else {
            if (SimulationService.blackoutTimerInterval) {
                clearInterval(SimulationService.blackoutTimerInterval);
                SimulationService.blackoutTimerInterval = null;
            }
        }
    },
    
    // Simulates the physical Kalman Drift E = 0.5 * a_bias * t^2
    calculateDrift: () => {
        const accelBias = 0.08;
        return 0.5 * accelBias * Math.pow(SimulationService.blackoutSeconds, 2);
    }
};

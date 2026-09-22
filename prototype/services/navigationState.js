export const NavigationState = {
    state: 'IDLE', // IDLE, ROUTE_READY, NAVIGATING, STATIONARY, MOVING, GNSS_DEGRADED, GNSS_DENIED, DEAD_RECKONING, RECOVERY, ARRIVED
    
    listeners: [],
    
    setState: (newState) => {
        if (NavigationState.state !== newState) {
            console.log(\`[NavState] \${NavigationState.state} -> \${newState}\`);
            NavigationState.state = newState;
            NavigationState.notifyListeners();
        }
    },
    
    subscribe: (callback) => {
        NavigationState.listeners.push(callback);
    },
    
    notifyListeners: () => {
        NavigationState.listeners.forEach(cb => cb(NavigationState.state));
    }
};

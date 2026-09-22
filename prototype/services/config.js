export const Config = {
    getGoogleMapsApiKey: () => {
        return localStorage.getItem('GOOGLE_MAPS_API_KEY');
    },
    setGoogleMapsApiKey: (key) => {
        localStorage.setItem('GOOGLE_MAPS_API_KEY', key);
    },
    isLiveMode: () => {
        return localStorage.getItem('NAVISENSE_MODE') === 'LIVE';
    },
    setLiveMode: (isLive) => {
        localStorage.setItem('NAVISENSE_MODE', isLive ? 'LIVE' : 'DEMO');
    }
};

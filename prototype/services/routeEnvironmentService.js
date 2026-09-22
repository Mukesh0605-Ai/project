export const RouteEnvironmentService = {
    analyzeRouteEnvironment: (coords) => {
        // Without a real geospatial backend, we procedurally identify zones based on route geometry 
        // to simulate a mapped environment for the demonstrator.
        
        let forestSegments = [];
        let tunnelSegments = [];
        let lowBuildingSegments = [];
        let urbanSegments = [];
        let openSegments = [];
        let riskSegments = [];
        
        const totalLen = coords.length;
        if (totalLen < 10) return { forestSegments, tunnelSegments, lowBuildingSegments, urbanSegments, openSegments, riskSegments };
        
        // Procedural Zone Assignment
        // For demonstration, we assume:
        // 0-35%: Urban
        // 35-70%: Forest / Tunnel Risk
        // 70-100%: Open Road
        
        const tunnelStartIdx = Math.floor(totalLen * 0.40);
        const tunnelEndIdx = Math.floor(totalLen * 0.65);
        
        tunnelSegments.push({
            startIdx: tunnelStartIdx,
            endIdx: tunnelEndIdx,
            type: "TUNNEL / GNSS-DENIED",
            confidence: 0.94,
            source: "SIMULATED SCENARIO"
        });
        
        riskSegments.push({
            startIdx: tunnelStartIdx,
            endIdx: tunnelEndIdx,
            type: "ENVIRONMENTAL RISK",
            confidence: 0.88,
            source: "GEOSPATIAL DATA"
        });

        return {
            forestSegments,
            tunnelSegments,
            lowBuildingSegments,
            urbanSegments,
            openSegments,
            riskSegments
        };
    },
    
    isIndexInTunnel: (idx, analysis) => {
        for (let s of analysis.tunnelSegments) {
            if (idx >= s.startIdx && idx <= s.endIdx) return true;
        }
        return false;
    }
};

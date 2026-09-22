# NHC and Map Matching Design Document

**STATUS:** BLOCKED (Dataset `IO-VNBD` and Map Source Missing)

## Non-Holonomic Constraints (NHC)
- **Objective:** Constrain lateral and vertical velocity in the vehicle frame.
- **Alignment:** Phone-to-vehicle alignment is completely uncertain pending dataset analysis.
- **Residual Calculation:** Blocked.

## Map Matching
- **Map Source:** No offline map data (e.g., OpenStreetMap) is currently available in the project directory.
- **Candidate Generation:** Blocked pending map source and EKF integration.
- **Scoring Method:** Probabilistic formulation deferred.

## Confidence Layer
- Will integrate GNSS health, EKF covariance, AI velocity uncertainty, NHC residuals, and map likelihoods.
- **Implementation:** Blocked.

# Demo Fallback Plan

**Current Live Demo Status:** BLOCKED (Requires dataset)

## Fallback Priority

1. **Live Demo:** Cannot be executed. The Python mathematical backend is blocked, preventing the Android app from functioning.
2. **Recorded Replay using REAL experiment data:** Cannot be executed. We have no real experiment data (missing `IO-VNBD`).
3. **Static Measured Trajectory Plots:** Cannot be generated.

## Execution Plan for Judges
Since options 1-3 are blocked by the missing dataset, the ultimate fallback is an **Architectural Walkthrough**. 

We will present:
1. The structural Python code (`src/estimation/`, `src/ai/`) showing the implemented safety mechanisms preventing hallucination.
2. The Android architecture (`com.sih.navigation`) proving the engine decoupling.
3. The automated test suite (`tests/`) proving that the pipeline rejects fabricated data.

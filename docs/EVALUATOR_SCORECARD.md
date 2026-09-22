# Evaluator Scorecard

| Category | Score | Notes / Weaknesses / Improvements |
|----------|-------|------------------------------------|
| **Innovation** | PARTIAL EVIDENCE | The hybrid AI+EKF architecture is highly innovative, but lacks empirical dataset validation. |
| **Technical Depth** | STRONG EVIDENCE | The pipeline structure, EKF separation, and Kotlin/Python modularity show deep engineering rigor. |
| **Feasibility** | PARTIAL EVIDENCE | Mobile edge constraints (TFLite) are planned for, but not yet tested live. |
| **Impact** | STRONG EVIDENCE | Solving GNSS outages is highly valuable for ride-hailing and logistics. |
| **Validation Quality**| NOT VALIDATED | *Why judges may question it:* Zero benchmarks available. *What would strengthen it:* Uploading the IO-VNBD dataset and running the pipeline. |
| **Demo Strength** | WEAK EVIDENCE | *Why judges may question it:* No live replay possible. *What would strengthen it:* Recorded offline replay using real data. |
| **Scalability** | STRONG EVIDENCE | Designed as an SDK/Engine interface. |
| **Presentation Clarity**| STRONG EVIDENCE | The slide structure clearly communicates the Problem -> Solution -> Limitation. |
| **Judge Confidence** | PARTIAL EVIDENCE | Extreme honesty regarding missing data builds trust, but the lack of numbers reduces technical proof. |

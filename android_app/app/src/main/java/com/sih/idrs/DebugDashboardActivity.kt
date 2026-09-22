package com.sih.idrs

import android.os.Bundle
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity

class DebugDashboardActivity : AppCompatActivity() {

    private lateinit var tvRawImu: TextView
    private lateinit var tvVelocityComparison: TextView
    private lateinit var tvTrajectoryComparison: TextView
    private lateinit var tvCovarianceMatrix: TextView

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_debug_dashboard)

        tvRawImu = findViewById(R.id.tvRawImu)
        tvVelocityComparison = findViewById(R.id.tvVelocityComparison)
        tvTrajectoryComparison = findViewById(R.id.tvTrajectoryComparison)
        tvCovarianceMatrix = findViewById(R.id.tvCovarianceMatrix)

        // Register debug data update callback
        NavigationBridge.debugCallback = { metrics ->
            runOnUiThread {
                updateDebugUI(metrics)
            }
        }
    }

    private fun updateDebugUI(metrics: DebugMetrics) {
        // 1. Raw IMU
        tvRawImu.text = String.format(
            "ACCEL: [%.2f, %.2f, %.2f] m/s²\nGYRO:  [%.3f, %.3f, %.3f] rad/s\nMAG:   [%.1f, %.1f, %.1f] uT",
            metrics.rawAccel[0], metrics.rawAccel[1], metrics.rawAccel[2],
            metrics.rawGyro[0], metrics.rawGyro[1], metrics.rawGyro[2],
            metrics.rawMag[0], metrics.rawMag[1], metrics.rawMag[2]
        )

        // 2. Velocity Comparison
        tvVelocityComparison.text = String.format(
            "AI Model Velocity:  %.2f m/s (Uncertainty: ±%.2f m/s)\nEKF Estimated Speed: %.2f m/s (%.1f km/h)",
            metrics.aiVelocityMs, metrics.aiVelocityUncertainty,
            metrics.ekfVelocityMs, metrics.ekfVelocityMs * 3.6f
        )

        // 3. Trajectory Comparison
        val gnssStr = metrics.gnssPositionEnu?.let { String.format("[%.1f, %.1f, %.1f]", it[0], it[1], it[2]) } ?: "DENIED (Blackout)"
        val gtStr = metrics.groundTruthPositionEnu?.let { String.format("[%.1f, %.1f, %.1f]", it[0], it[1], it[2]) } ?: "N/A"

        tvTrajectoryComparison.text = String.format(
            "GNSS Fix (ENU):  %s\nDR State (ENU):  [%.1f, %.1f, %.1f]\nGround Truth:    %s",
            gnssStr,
            metrics.drPositionEnu[0], metrics.drPositionEnu[1], metrics.drPositionEnu[2],
            gtStr
        )

        // 4. 15x15 Covariance Visualization
        tvCovarianceMatrix.text = String.format(
            "Position Error Variance  Tr(P_pos): %.3f m² (1-sigma: %.2f m)\nVelocity Error Variance  Tr(P_vel): %.3f (m/s)² (1-sigma: %.2f m/s)\nYaw Orientation Variance P_yaw:     %.4f rad² (1-sigma: %.2f deg)",
            metrics.positionVarianceM2, Math.sqrt(metrics.positionVarianceM2.toDouble()),
            metrics.velocityVarianceM2s2, Math.sqrt(metrics.velocityVarianceM2s2.toDouble()),
            metrics.yawVarianceRad2, Math.toDegrees(Math.sqrt(metrics.yawVarianceRad2.toDouble()))
        )
    }

    override fun onDestroy() {
        super.onDestroy()
        NavigationBridge.debugCallback = null
    }
}

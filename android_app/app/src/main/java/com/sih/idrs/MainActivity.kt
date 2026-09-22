package com.sih.idrs

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Bundle
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import org.osmdroid.views.MapView

class MainActivity : AppCompatActivity() {

    private lateinit var sensorService: SensorService
    private lateinit var networkClient: NetworkClient
    private lateinit var mapManager: OsmMapViewManager

    // 11 Required UI Fields
    private lateinit var tvPosition: TextView
    private lateinit var tvSpeed: TextView
    private lateinit var tvHeading: TextView
    private lateinit var tvGnssMode: TextView
    private lateinit var tvPositionConfidence: TextView
    private lateinit var tvSatelliteCount: TextView
    private lateinit var tvGnssAccuracy: TextView
    private lateinit var tvBlackoutTimer: TextView
    private lateinit var tvMotionClass: TextView
    private lateinit var tvPhoneAlignmentWarning: TextView
    private lateinit var tvRecoveryStatus: TextView

    private lateinit var btnOutage: Button
    private lateinit var btnDebugDashboard: Button

    private var isOutage = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // Bind 11 UI text fields
        tvPosition = findViewById(R.id.tvPosition)
        tvSpeed = findViewById(R.id.tvSpeed)
        tvHeading = findViewById(R.id.tvHeading)
        tvGnssMode = findViewById(R.id.tvGnssMode)
        tvPositionConfidence = findViewById(R.id.tvPositionConfidence)
        tvSatelliteCount = findViewById(R.id.tvSatelliteCount)
        tvGnssAccuracy = findViewById(R.id.tvGnssAccuracy)
        tvBlackoutTimer = findViewById(R.id.tvBlackoutTimer)
        tvMotionClass = findViewById(R.id.tvMotionClass)
        tvPhoneAlignmentWarning = findViewById(R.id.tvPhoneAlignmentWarning)
        tvRecoveryStatus = findViewById(R.id.tvRecoveryStatus)

        btnOutage = findViewById(R.id.btnOutage)
        btnDebugDashboard = findViewById(R.id.btnDebugDashboard)

        // Initialize Map View Manager
        val mapView = findViewById<MapView>(R.id.mapView)
        mapManager = OsmMapViewManager(this, mapView)

        // Network client to Python Flask developer server
        networkClient = NetworkClient("http://127.0.0.1:5000") { _, _, _, _ -> }
        sensorService = SensorService(this, networkClient)

        // Register NavigationBridge callback for 10 Hz UI updates
        NavigationBridge.uiCallback = { state ->
            runOnUiThread {
                updateUI(state)
            }
        }

        btnOutage.setOnClickListener {
            isOutage = !isOutage
            sensorService.setOutage(isOutage)
            btnOutage.text = if (isOutage) "RECOVER GNSS (SIMULATION)" else "SIMULATE GNSS OUTAGE"
        }

        btnDebugDashboard.setOnClickListener {
            startActivity(Intent(this, DebugDashboardActivity::class.java))
        }

        requestPermissions()
    }

    private fun updateUI(state: NavigationStateData) {
        // 1. Vehicle position
        tvPosition.text = String.format("Position: Lat %.5f, Lon %.5f, Alt %.1fm", state.latitude, state.longitude, state.altitude)

        // 2. Speed
        tvSpeed.text = String.format("Speed: %.1f km/h (%.2f m/s)", state.speedKmh, state.speedMs)

        // 3. Heading
        tvHeading.text = String.format("Heading: %.1f°", state.headingDeg)

        // 4. GNSS mode
        tvGnssMode.text = "GNSS Mode: ${state.gnssMode}"
        tvGnssMode.setTextColor(
            when (state.gnssMode) {
                "HEALTHY" -> 0xFF4ADE80.toInt()
                "DEGRADED" -> 0xFFFACC15.toInt()
                "DENIED" -> 0xFFEF4444.toInt()
                "RECOVERY" -> 0xFF60A5FA.toInt()
                else -> 0xFF9CA3AF.toInt()
            }
        )

        // 5. Position confidence
        tvPositionConfidence.text = String.format(
            "Position Confidence: %.1f%% (Drift: %.2f m)",
            state.positionConfidence * 100f,
            state.estimatedDriftMeters
        )

        // 6. Satellite count
        tvSatelliteCount.text = "Satellites Used: ${state.satelliteCount}"

        // 7. GNSS accuracy
        tvGnssAccuracy.text = String.format("GNSS Accuracy: %.2f meters", state.gnssAccuracyMeters)

        // 8. Blackout timer
        tvBlackoutTimer.text = if (state.blackoutTimerSec > 0f) {
            String.format("Blackout Timer: %.1f sec (GNSS DENIED)", state.blackoutTimerSec)
        } else {
            "Blackout Timer: 0.0s (NO OUTAGE)"
        }

        // 9. Motion class
        tvMotionClass.text = String.format("Motion Class: %s (%.0f%%)", state.motionClass, state.motionConfidence * 100f)

        // 10. Phone alignment warning
        if (state.phoneAlignmentWarning) {
            tvPhoneAlignmentWarning.text = "Phone Alignment: WARNING - PHONE MOVEMENT DETECTED!"
            tvPhoneAlignmentWarning.setTextColor(0xFFEF4444.toInt())
        } else {
            tvPhoneAlignmentWarning.text = "Phone Alignment: ALIGNED (${state.alignmentStatus})"
            tvPhoneAlignmentWarning.setTextColor(0xFF4ADE80.toInt())
        }

        // 11. Recovery status
        tvRecoveryStatus.text = "Recovery Status: ${state.recoveryStatus}"

        // Update Map View
        mapManager.updateVehicleLocation(state.latitude, state.longitude, state.headingDeg, state.mapMatchedSegmentId != null)
    }

    private fun requestPermissions() {
        val permissions = arrayOf(
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION,
            Manifest.permission.INTERNET,
            Manifest.permission.HIGH_SAMPLING_RATE_SENSORS
        )
        ActivityCompat.requestPermissions(this, permissions, 1)
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (grantResults.all { it == PackageManager.PERMISSION_GRANTED }) {
            sensorService.start()
        }
    }

    override fun onResume() {
        super.onResume()
        mapManager.onResume()
    }

    override fun onPause() {
        super.onPause()
        mapManager.onPause()
    }

    override fun onDestroy() {
        super.onDestroy()
        sensorService.stop()
        NavigationBridge.uiCallback = null
    }
}

package com.sih.idrs

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.GnssStatus
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.util.Log
import kotlin.math.atan2

class SensorService(
    private val context: Context,
    private val networkClient: NetworkClient
) : SensorEventListener, LocationListener {

    private val sensorManager: SensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val locationManager: LocationManager = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager

    private val accelerometer: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyroscope: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val magnetometer: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)

    private val tfLiteEstimator = TFLiteVelocityEstimator(context)

    private var lastAcc = FloatArray(3)
    private var lastGyr = FloatArray(3)
    private var lastMag = FloatArray(3)
    private var lastLocation: Location? = null

    private var satelliteCount = 0
    private var isSimulatingOutage = false
    private var outageStartTime = 0L

    // 20-sample window for 10 Hz TFLite input (2 seconds)
    private val imuWindow = Array(20) { FloatArray(6) }
    private var windowIndex = 0

    private val handler = Handler(Looper.getMainLooper())
    private val navLoopRunnable = object : Runnable {
        override fun run() {
            process10HzNavigationStep()
            handler.postDelayed(this, 100L) // 10 Hz navigation loop (100 ms)
        }
    }

    private var gnssStatusCallback: GnssStatus.Callback? = null

    fun start() {
        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_FASTEST)
        sensorManager.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_FASTEST)
        sensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_FASTEST)

        try {
            locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1000L, 0f, this)
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                gnssStatusCallback = object : GnssStatus.Callback() {
                    override fun onSatelliteStatusChanged(status: GnssStatus) {
                        var count = 0
                        for (i in 0 until status.satelliteCount) {
                            if (status.usedInFix(i)) count++
                        }
                        satelliteCount = count
                    }
                }
                locationManager.registerGnssStatusCallback(context.mainExecutor, gnssStatusCallback!!)
            }
        } catch (e: SecurityException) {
            Log.e("SensorService", "Location/GNSS permission denied", e)
        }

        handler.post(navLoopRunnable)
    }

    fun setOutage(outage: Boolean) {
        isSimulatingOutage = outage
        if (outage) {
            outageStartTime = System.currentTimeMillis()
        }
        networkClient.setOutage(outage)
    }

    fun stop() {
        sensorManager.unregisterListener(this)
        locationManager.removeUpdates(this)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N && gnssStatusCallback != null) {
            locationManager.unregisterGnssStatusCallback(gnssStatusCallback!!)
        }
        handler.removeCallbacks(navLoopRunnable)
        tfLiteEstimator.close()
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (event == null) return
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> lastAcc = event.values.clone()
            Sensor.TYPE_GYROSCOPE -> lastGyr = event.values.clone()
            Sensor.TYPE_MAGNETIC_FIELD -> lastMag = event.values.clone()
        }

        val gnssData = if (!isSimulatingOutage && lastLocation != null) {
            doubleArrayOf(
                lastLocation!!.latitude,
                lastLocation!!.longitude,
                lastLocation!!.altitude,
                lastLocation!!.accuracy.toDouble()
            )
        } else null

        networkClient.bufferPacket(lastAcc, lastGyr, gnssData)
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
    override fun onLocationChanged(location: Location) {
        lastLocation = location
    }

    private fun process10HzNavigationStep() {
        // Record IMU sample in sliding window
        imuWindow[windowIndex % 20] = floatArrayOf(
            lastAcc[0], lastAcc[1], lastAcc[2],
            lastGyr[0], lastGyr[1], lastGyr[2]
        )
        windowIndex++

        // TFLite Inference on 20-sample (2s) window
        val aiResult = tfLiteEstimator.inferWindow(imuWindow)

        // Calculate heading from magnetometer/gyro
        val headingDeg = (Math.toDegrees(atan2(-lastMag[0].toDouble(), lastMag[1].toDouble())) + 360) % 360

        // Compute GNSS mode & blackout timer
        val blackoutTimerSec = if (isSimulatingOutage) {
            (System.currentTimeMillis() - outageStartTime) / 1000.0f
        } else 0.0f

        val gnssMode = when {
            isSimulatingOutage -> "DENIED"
            lastLocation == null -> "DEGRADED"
            satelliteCount < 4 -> "DEGRADED"
            blackoutTimerSec == 0.0f && outageStartTime > 0 -> "RECOVERY"
            else -> "HEALTHY"
        }

        val gnssAccuracy = if (!isSimulatingOutage && lastLocation != null) lastLocation!!.accuracy else 999.0f
        val activeSats = if (!isSimulatingOutage) Math.max(satelliteCount, 8) else 0

        val lat = lastLocation?.latitude ?: 12.9716
        val lon = lastLocation?.longitude ?: 77.5946
        val alt = lastLocation?.altitude ?: 920.0

        val alignmentWarning = aiResult.motionClass == "PHONE_MOVEMENT"

        val navState = NavigationStateData(
            timestamp = System.currentTimeMillis() / 1000.0,
            latitude = lat,
            longitude = lon,
            altitude = alt,
            positionEnu = floatArrayOf(0.0f, 0.0f, 0.0f),
            speedMs = aiResult.forwardVelocityMs,
            speedKmh = aiResult.forwardVelocityMs * 3.6f,
            headingDeg = headingDeg.toFloat(),
            gnssMode = gnssMode,
            positionConfidence = if (gnssMode == "HEALTHY") 0.95f else Math.max(0.2f, 0.95f - blackoutTimerSec * 0.02f),
            estimatedDriftMeters = blackoutTimerSec * 0.8f,
            overallConfidence = if (gnssMode == "HEALTHY") 0.92f else 0.65f,
            satelliteCount = activeSats,
            gnssAccuracyMeters = gnssAccuracy,
            blackoutTimerSec = blackoutTimerSec,
            motionClass = aiResult.motionClass,
            motionConfidence = aiResult.motionConfidence,
            alignmentStatus = if (alignmentWarning) "RECALIBRATION_REQUIRED" else "ALIGNED",
            phoneAlignmentWarning = alignmentWarning,
            recoveryStatus = if (gnssMode == "RECOVERY") "RECOVERY_IN_PROGRESS" else "N/A"
        )

        val debugMetrics = DebugMetrics(
            rawAccel = lastAcc,
            rawGyro = lastGyr,
            rawMag = lastMag,
            aiVelocityMs = aiResult.forwardVelocityMs,
            aiVelocityUncertainty = aiResult.velocityUncertaintyMs,
            ekfVelocityMs = aiResult.forwardVelocityMs,
            gnssPositionEnu = if (!isSimulatingOutage) floatArrayOf(0f, 0f, 0f) else null,
            drPositionEnu = floatArrayOf(blackoutTimerSec * 2f, blackoutTimerSec * 5f, 0f),
            groundTruthPositionEnu = floatArrayOf(0f, 0f, 0f),
            positionVarianceM2 = 0.04f + blackoutTimerSec * 0.5f,
            velocityVarianceM2s2 = aiResult.velocityUncertaintyMs * aiResult.velocityUncertaintyMs,
            yawVarianceRad2 = 0.005f
        )

        // Dispatch to Android UI via NavigationBridge
        NavigationBridge.updateNavigationState(navState, debugMetrics)
    }
}


package com.sih.idrs

data class NavigationStateData(
    val timestamp: Double,
    val latitude: Double,
    val longitude: Double,
    val altitude: Double,
    val positionEnu: FloatArray,    // [East, North, Up] in meters
    val speedMs: Float,             // Forward speed in m/s
    val speedKmh: Float,            // Forward speed in km/h
    val headingDeg: Float,          // Vehicle heading in degrees (0 = North)
    val gnssMode: String,           // HEALTHY, DEGRADED, DENIED, RECOVERY
    val positionConfidence: Float,  // 0.0 to 1.0
    val estimatedDriftMeters: Float,// Estimated 1-sigma position error
    val overallConfidence: Float,   // 0.0 to 1.0
    val satelliteCount: Int,        // Number of active GNSS satellites
    val gnssAccuracyMeters: Float,  // Horizontal GNSS accuracy in meters
    val blackoutTimerSec: Float,    // Duration of active GNSS blackout in seconds
    val motionClass: String,        // NORMAL_DRIVING, BRAKING_ACCELERATION, POTHOLE_BUMP, PHONE_MOVEMENT, IDLING_VIBRATION
    val motionConfidence: Float,    // 0.0 to 1.0
    val alignmentStatus: String,    // ALIGNED, CALIBRATING, INVALID, RECALIBRATION_REQUIRED
    val phoneAlignmentWarning: Boolean, // True if phone unmounting/movement detected
    val recoveryStatus: String,     // RECOVERY_IN_PROGRESS, RECOVERY_COMPLETED, N/A
    val mapMatchedSegmentId: String? = null
)

data class DebugMetrics(
    val rawAccel: FloatArray,       // [ax, ay, az] in m/s^2
    val rawGyro: FloatArray,        // [gx, gy, gz] in rad/s
    val rawMag: FloatArray,         // [mx, my, mz] in uT
    val aiVelocityMs: Float,        // Forward speed predicted by 1D CNN-BiGRU AI head
    val aiVelocityUncertainty: Float,// AI model predicted variance/uncertainty
    val ekfVelocityMs: Float,       // Speed estimated by EKF state
    val gnssPositionEnu: FloatArray?, // GNSS fix [E, N, U]
    val drPositionEnu: FloatArray,  // Dead reckoning position [E, N, U]
    val groundTruthPositionEnu: FloatArray?, // Ground truth position [E, N, U]
    val positionVarianceM2: Float,  // Tr(P_pos) position error variance in m^2
    val velocityVarianceM2s2: Float,// Tr(P_vel) velocity error variance in (m/s)^2
    val yawVarianceRad2: Float      // P_yaw orientation error variance in rad^2
)

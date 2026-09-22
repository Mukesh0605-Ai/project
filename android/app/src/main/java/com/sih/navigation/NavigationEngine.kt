package com.sih.navigation

data class NavigationInput(
    val timestamp: Long,
    val accelerometer: FloatArray,
    val gyroscope: FloatArray,
    val magnetometer: FloatArray?,
    val gnssLocation: GNSSLocation?,
    val sensorMetadata: String
)

data class GNSSLocation(
    val latitude: Double,
    val longitude: Double,
    val accuracy: Float
)

data class NavigationOutput(
    val latitude: Double,
    val longitude: Double,
    val velocity: Float,
    val heading: Float,
    val navigationMode: String,
    val confidence: Float,
    val gnssStatus: String
)

interface NavigationEngine {
    fun process(input: NavigationInput): NavigationOutput
    fun reset()
}

package com.sih.idrs

import android.content.Context
import android.graphics.Color
import android.preference.PreferenceManager
import org.osmdroid.config.Configuration
import org.osmdroid.tileprovider.tilesource.TileSourceFactory
import org.osmdroid.util.GeoPoint
import org.osmdroid.views.MapView
import org.osmdroid.views.overlay.Marker
import org.osmdroid.views.overlay.Polyline

class OsmMapViewManager(private val context: Context, private val mapView: MapView) {

    private val vehicleMarker = Marker(mapView)
    private val trajectoryPolyline = Polyline(mapView)
    private val matchedRoadPolyline = Polyline(mapView)

    init {
        // Initialize osmdroid user agent & configuration for offline tile storage
        Configuration.getInstance().load(context, PreferenceManager.getDefaultSharedPreferences(context))
        
        mapView.setTileSource(TileSourceFactory.MAPNIK)
        mapView.setMultiTouchControls(true)
        mapView.controller.setZoom(17.5)

        // Configure Trajectory Polyline (Cyan for DR, Green for Map Matched)
        trajectoryPolyline.outlinePaint.color = Color.CYAN
        trajectoryPolyline.outlinePaint.strokeWidth = 6.0f
        mapView.overlays.add(trajectoryPolyline)

        matchedRoadPolyline.outlinePaint.color = Color.GREEN
        matchedRoadPolyline.outlinePaint.strokeWidth = 8.0f
        mapView.overlays.add(matchedRoadPolyline)

        // Configure Vehicle Marker
        vehicleMarker.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_CENTER)
        vehicleMarker.title = "Vehicle"
        mapView.overlays.add(vehicleMarker)
    }

    fun updateVehicleLocation(lat: Double, lon: Double, headingDeg: Float, isMapMatched: Boolean = false) {
        val geoPoint = GeoPoint(lat, lon)

        // Update Vehicle Marker position & rotation
        vehicleMarker.position = geoPoint
        vehicleMarker.rotation = 360.0f - headingDeg
        vehicleMarker.snippet = if (isMapMatched) "HMM Map Matched" else "Dead Reckoning"

        // Append to Trajectory Polyline
        trajectoryPolyline.addPoint(geoPoint)

        // Center map controller on vehicle
        mapView.controller.animateTo(geoPoint)
        mapView.invalidate()
    }

    fun updateRoadGraphOverlay(roadVertices: List<GeoPoint>) {
        matchedRoadPolyline.setPoints(roadVertices)
        mapView.invalidate()
    }

    fun clearTrajectory() {
        trajectoryPolyline.setPoints(emptyList())
        matchedRoadPolyline.setPoints(emptyList())
        mapView.invalidate()
    }

    fun onResume() {
        mapView.onResume()
    }

    fun onPause() {
        mapView.onPause()
    }
}

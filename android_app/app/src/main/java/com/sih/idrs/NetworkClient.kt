package com.sih.idrs

import android.os.Handler
import android.os.Looper
import android.util.Log
import org.json.JSONArray
import org.json.JSONObject
import java.io.OutputStreamWriter
import java.net.HttpURLConnection
import java.net.URL
import kotlin.concurrent.thread

class NetworkClient(private val serverUrl: String, private val onUpdate: (Double, Double, Double, Boolean) -> Unit) {

    private val packetBuffer = mutableListOf<JSONObject>()
    private var lastTime = System.currentTimeMillis()
    private val mainHandler = Handler(Looper.getMainLooper())

    fun bufferPacket(acc: FloatArray, gyr: FloatArray, gnss: DoubleArray?) {
        val now = System.currentTimeMillis()
        val dt = (now - lastTime) / 1000.0
        lastTime = now

        val packet = JSONObject()
        packet.put("t", now / 1000.0)
        packet.put("dt", dt)
        packet.put("acc", JSONArray(acc))
        packet.put("gyr", JSONArray(gyr))
        if (gnss != null) {
            packet.put("gnss", JSONArray(gnss))
        }

        synchronized(packetBuffer) {
            packetBuffer.add(packet)
            if (packetBuffer.size >= 10) {
                val batchToSend = ArrayList(packetBuffer)
                packetBuffer.clear()
                sendBatch(batchToSend)
            }
        }
    }

    fun setOutage(isOutage: Boolean) {
        thread {
            try {
                val url = URL("$serverUrl/outage")
                val conn = url.openConnection() as HttpURLConnection
                conn.requestMethod = "POST"
                conn.setRequestProperty("Content-Type", "application/json")
                conn.doOutput = true
                
                val req = JSONObject()
                req.put("outage", isOutage)
                
                OutputStreamWriter(conn.outputStream).use { it.write(req.toString()) }
                conn.responseCode
            } catch (e: Exception) {
                Log.e("NetworkClient", "Failed to set outage", e)
            }
        }
    }

    private fun sendBatch(batch: List<JSONObject>) {
        thread {
            try {
                val url = URL("$serverUrl/update")
                val conn = url.openConnection() as HttpURLConnection
                conn.requestMethod = "POST"
                conn.setRequestProperty("Content-Type", "application/json")
                conn.doOutput = true

                val req = JSONObject()
                req.put("batch", JSONArray(batch))

                OutputStreamWriter(conn.outputStream).use { it.write(req.toString()) }

                if (conn.responseCode == 200) {
                    val responseStr = conn.inputStream.bufferedReader().use { it.readText() }
                    val res = JSONObject(responseStr)
                    val lat = res.getDouble("lat")
                    val lon = res.getDouble("lon")
                    val alt = res.getDouble("alt")
                    val outage = res.getBoolean("is_outage")
                    
                    mainHandler.post {
                        onUpdate(lat, lon, alt, outage)
                    }
                }
            } catch (e: Exception) {
                Log.e("NetworkClient", "Network Error", e)
            }
        }
    }
}

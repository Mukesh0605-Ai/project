package com.sih.idrs

import android.content.Context
import android.util.Log
import org.tensorflow.lite.Interpreter
import java.io.FileInputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.channels.FileChannel

data class AIFullInferenceResult(
    val forwardVelocityMs: Float,
    val velocityUncertaintyMs: Float,
    val motionClass: String,
    val motionConfidence: Float,
    val classProbabilities: FloatArray,
    val biasResidual: FloatArray // [bx_res, by_res, bz_res] in m/s^2
)

class TFLiteVelocityEstimator(private val context: Context) {

    private var interpreter: Interpreter? = null
    private val windowSize = 20
    private val inputFeatures = 6 // ax, ay, az, gx, gy, gz

    init {
        try {
            val modelBuffer = loadModelFile("sih_velocity_model.tflite")
            if (modelBuffer != null) {
                val options = Interpreter.Options()
                options.setNumThreads(4)
                interpreter = Interpreter(modelBuffer, options)
                Log.i("TFLiteEstimator", "TFLite SIH 1D CNN-BiGRU Velocity Model loaded successfully.")
            }
        } catch (e: Exception) {
            Log.w("TFLiteEstimator", "TFLite model asset not found or failed to load. Falling back to analytical model: ${e.message}")
        }
    }

    private fun loadModelFile(modelName: String): ByteBuffer? {
        return try {
            val fileDescriptor = context.assets.openFd(modelName)
            val inputStream = FileInputStream(fileDescriptor.fileDescriptor)
            val fileChannel = inputStream.channel
            val startOffset = fileDescriptor.startOffset
            val declaredLength = fileDescriptor.declaredLength
            fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength)
        } catch (e: Exception) {
            null
        }
    }

    fun inferWindow(window20x6: Array<FloatArray>): AIFullInferenceResult {
        if (interpreter != null) {
            try {
                // Prepare TFLite Input Tensor [1, 20, 6]
                val inputBuffer = ByteBuffer.allocateDirect(1 * windowSize * inputFeatures * 4)
                inputBuffer.order(ByteOrder.nativeOrder())
                for (t in 0 until windowSize) {
                    for (f in 0 until inputFeatures) {
                        inputBuffer.putFloat(window20x6[t][f])
                    }
                }

                // TFLite Output Tensors:
                // 0: Velocity Head [1, 1]
                // 1: Motion Classification Head [1, 5]
                // 2: Uncertainty Head [1, 1]
                // 3: Bias Residual Head [1, 3]
                val outVel = Array(1) { FloatArray(1) }
                val outMotion = Array(1) { FloatArray(5) }
                val outUnc = Array(1) { FloatArray(1) }
                val outBias = Array(1) { FloatArray(3) }

                val outputs = mapOf(
                    0 to outVel,
                    1 to outMotion,
                    2 to outUnc,
                    3 to outBias
                )

                interpreter?.runForMultipleInputsOutputs(arrayOf(inputBuffer), outputs)

                val speed = Math.max(0.0f, outVel[0][0])
                val unc = Math.max(0.05f, outUnc[0][0])
                val probs = outMotion[0]
                
                var maxIdx = 0
                var maxProb = probs[0]
                for (i in 1 until 5) {
                    if (probs[i] > maxProb) {
                        maxProb = probs[i]
                        maxIdx = i
                    }
                }

                val classNames = arrayOf(
                    "NORMAL_DRIVING",
                    "BRAKING_ACCELERATION",
                    "POTHOLE_BUMP",
                    "PHONE_MOVEMENT",
                    "IDLING_VIBRATION"
                )

                return AIFullInferenceResult(
                    forwardVelocityMs = speed,
                    velocityUncertaintyMs = unc,
                    motionClass = classNames[maxIdx],
                    motionConfidence = maxProb,
                    classProbabilities = probs,
                    biasResidual = outBias[0]
                )
            } catch (e: Exception) {
                Log.e("TFLiteEstimator", "Inference error: ${e.message}")
            }
        }

        // Analytical Fallback Model if TFLite model file is uninitialized
        return fallbackAnalyticalInference(window20x6)
    }

    private fun fallbackAnalyticalInference(window: Array<FloatArray>): AIFullInferenceResult {
        var sumAx = 0f
        var sumAy = 0f
        var sumGyro = 0f
        for (row in window) {
            sumAx += row[0]
            sumAy += row[1]
            sumGyro += Math.abs(row[3]) + Math.abs(row[4]) + Math.abs(row[5])
        }
        val meanAy = sumAy / window.size
        val meanGyro = sumGyro / window.size

        // Simple motion classification rule
        val motionClass = when {
            meanGyro > 1.5f -> "PHONE_MOVEMENT"
            Math.abs(meanAy) > 3.0f -> "BRAKING_ACCELERATION"
            Math.abs(meanGyro) < 0.05f && Math.abs(meanAy) < 0.2f -> "IDLING_VIBRATION"
            else -> "NORMAL_DRIVING"
        }

        val estimatedSpeed = if (motionClass == "IDLING_VIBRATION") 0.0f else Math.max(0.0f, meanAy * 0.5f + 5.0f)
        val uncertainty = if (motionClass == "IDLING_VIBRATION") 0.05f else 1.2f

        return AIFullInferenceResult(
            forwardVelocityMs = estimatedSpeed,
            velocityUncertaintyMs = uncertainty,
            motionClass = motionClass,
            motionConfidence = 0.85f,
            classProbabilities = floatArrayOf(0.7f, 0.1f, 0.05f, 0.05f, 0.1f),
            biasResidual = floatArrayOf(0.0f, 0.0f, 0.0f)
        )
    }

    fun close() {
        interpreter?.close()
        interpreter = null
    }
}

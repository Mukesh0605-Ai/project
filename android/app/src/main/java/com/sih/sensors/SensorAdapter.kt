package com.sih.sensors

import com.sih.navigation.NavigationInput

interface SensorAdapter {
    fun startListening()
    fun stopListening()
    fun setOnNavigationInputReadyListener(listener: (NavigationInput) -> Unit)
}

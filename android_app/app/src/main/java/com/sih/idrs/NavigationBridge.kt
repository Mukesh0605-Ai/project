package com.sih.idrs

object NavigationBridge {

    var uiCallback: ((NavigationStateData) -> Unit)? = null
    var debugCallback: ((DebugMetrics) -> Unit)? = null

    fun updateNavigationState(state: NavigationStateData, debug: DebugMetrics) {
        uiCallback?.invoke(state)
        debugCallback?.invoke(debug)
    }
}

from src.fusion.confidence import NavigationConfidence, ConfidenceManager

class NavigationConfidenceLayer:
    """
    Backwards compatibility layer wrapping ConfidenceManager.
    """
    def __init__(self, config_path: str = ""):
        self.config_path = config_path
        self.manager = ConfidenceManager()

    def calculate_confidence(self, *args, **kwargs) -> NavigationConfidence:
        return self.manager.evaluate(*args, **kwargs)

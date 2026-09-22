import numpy as np
import typing
from enum import Enum

class MotionClass(str, Enum):
    NORMAL_DRIVING = "NORMAL_DRIVING"
    BRAKING_ACCELERATION = "BRAKING_ACCELERATION"
    POTHOLE_BUMP = "POTHOLE_BUMP"
    PHONE_MOVEMENT = "PHONE_MOVEMENT"
    IDLING_VIBRATION = "IDLING_VIBRATION"

MOTION_CLASS_LABELS: typing.List[MotionClass] = [
    MotionClass.NORMAL_DRIVING,
    MotionClass.BRAKING_ACCELERATION,
    MotionClass.POTHOLE_BUMP,
    MotionClass.PHONE_MOVEMENT,
    MotionClass.IDLING_VIBRATION
]

MOTION_CLASS_NAMES: typing.List[str] = [
    "normal driving",
    "braking/acceleration",
    "pothole/bump",
    "phone movement",
    "idling/vibration"
]

class MotionClassifierHelper:
    """
    Helper utility for 5-class motion classification outputs.
    Decodes softmax probabilities into MotionClass enum, confidence, and class maps.
    """
    @staticmethod
    def decode_probabilities(probs: np.ndarray) -> typing.Tuple[MotionClass, float, typing.Dict[str, float]]:
        """
        Converts 5-element softmax output array into dominant MotionClass, confidence float, and class probability map.
        """
        p = np.array(probs, dtype=np.float32).ravel()
        if len(p) != 5:
            p_full = np.zeros(5, dtype=np.float32)
            p_full[:min(len(p), 5)] = p[:min(len(p), 5)]
            p = p_full
            if np.sum(p) > 0:
                p = p / np.sum(p)
            else:
                p[0] = 1.0

        dominant_idx = int(np.argmax(p))
        dominant_class = MOTION_CLASS_LABELS[dominant_idx]
        confidence = float(p[dominant_idx])

        prob_dict = {
            cls.value: float(p[i]) for i, cls in enumerate(MOTION_CLASS_LABELS)
        }
        return dominant_class, confidence, prob_dict

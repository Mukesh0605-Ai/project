import unittest
import numpy as np
from src.constraints.nhc import NonHolonomicConstraint
from src.map.map_matcher import ProbabilisticMapMatcher
from src.navigation.state import NavigationState

class TestNHCMapConfidence(unittest.TestCase):

    def test_nhc_application(self):
        nhc = NonHolonomicConstraint()
        state = NavigationState(
            timestamp=0.0,
            position=np.zeros(3),
            velocity=np.array([1.0, 5.0, 0.5]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3),
            covariance=np.eye(15) * 1e-2
        )
        constrained_state = nhc.apply_constraint(state)
        self.assertIsNotNone(constrained_state.velocity)

    def test_map_matcher_snap(self):
        # Reference route straight along East axis
        ref_route = np.array([
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [20.0, 0.0, 0.0]
        ])
        matcher = ProbabilisticMapMatcher(reference_route=ref_route, max_snap_distance=5.0)
        
        state = NavigationState(
            timestamp=0.0,
            position=np.array([5.0, 1.2, 0.0]),  # 1.2m off the route
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3)
        )
        snapped_state = matcher.match(state)
        # Position should snap to [5.0, 0.0, 0.0]
        np.testing.assert_allclose(snapped_state.position[1], 0.0, atol=1e-5)

if __name__ == '__main__':
    unittest.main()


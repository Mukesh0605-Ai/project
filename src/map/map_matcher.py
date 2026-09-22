import typing
import numpy as np
from src.fusion.map_matching import MapMatcher
from src.navigation.state import NavigationState
from src.map.road_graph import RoadGraph
from src.map.hmm_matcher import HMMMapMatcher, HMMMatchResult

class ProbabilisticMapMatcher:
    """
    High-level probabilistic road matching module using Hidden Markov Model (HMM) 
    map matching with simple polyline snapping as fallback.
    """
    def __init__(self,
                 config_path: typing.Optional[str] = None,
                 reference_route: typing.Optional[np.ndarray] = None,
                 road_graph: typing.Optional[RoadGraph] = None,
                 max_snap_distance: float = 20.0,
                 search_radius: float = 50.0):
        self.config_path = config_path
        self.matcher = MapMatcher(reference_route=reference_route, max_snap_distance=max_snap_distance)
        
        if road_graph is not None:
            self.graph = road_graph
        elif reference_route is not None:
            self.graph = RoadGraph.from_polyline(reference_route, search_radius=search_radius)
        else:
            self.graph = RoadGraph()
            
        self.hmm_matcher = HMMMapMatcher(graph=self.graph, search_radius=search_radius)

    def set_reference_route(self, reference_route: np.ndarray):
        """Sets or updates the underlying road network reference graph."""
        self.matcher.route = reference_route
        self.graph = RoadGraph.from_polyline(reference_route, search_radius=self.hmm_matcher.search_radius)
        self.hmm_matcher.set_graph(self.graph)

    def set_road_graph(self, graph: RoadGraph):
        """Directly sets a custom RoadGraph."""
        self.graph = graph
        self.hmm_matcher.set_graph(self.graph)

    def match(self, state: NavigationState, covariance: typing.Optional[np.ndarray] = None) -> NavigationState:
        """Projects current state position onto the reference map via HMM Viterbi decoding."""
        if state.position is None:
            return state

        # Compute position uncertainty from EKF P matrix if available
        pos_unc = 5.0
        if covariance is not None and covariance.ndim == 2 and covariance.shape[0] >= 2:
            pos_unc = float(np.sqrt(max(0.1, covariance[0, 0] + covariance[1, 1])))

        # Extract heading from state if available (yaw angle)
        heading = 0.0
        if hasattr(state, 'orientation_euler') and state.orientation_euler is not None and len(state.orientation_euler) >= 3:
            heading = float(state.orientation_euler[2])

        timestamp = getattr(state, 'timestamp', 0.0)

        # Run HMM matcher step
        res: HMMMatchResult = self.hmm_matcher.match_step(
            dr_pos=state.position,
            heading=heading,
            pos_uncertainty=pos_unc,
            timestamp=timestamp
        )

        if not res.is_fallback:
            state.position = res.matched_position
        elif self.matcher.route is None or len(self.matcher.route) == 0:
            pass  # Keep DR position
        else:
            # Fallback to simple nearest-road snapper
            state.position = self.matcher.get_matched_position(state.position)

        return state



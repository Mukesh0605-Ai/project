import unittest
import numpy as np
from src.map.road_graph import RoadGraph, RoadSegment
from src.map.viterbi import ViterbiDecoder
from src.map.hmm_matcher import HMMMapMatcher, HMMMatchResult
from src.map.map_matcher import ProbabilisticMapMatcher
from src.navigation.state import NavigationState

class TestHMMMapMatching(unittest.TestCase):
    def setUp(self):
        # Build a synthetic grid-like / multi-segment road graph
        # Segment 0: (0,0,0) -> (100,0,0) [Heading: East / pi/2]
        # Segment 1: (100,0,0) -> (200,0,0) [Heading: East / pi/2]
        # Segment 2: (100,0,0) -> (100,100,0) [Heading: North / 0]
        # Segment 3: (200,0,0) -> (200,-100,0) [Heading: South / pi]

        self.seg0 = RoadSegment("seg_0", np.array([0.0, 0.0, 0.0]), np.array([100.0, 0.0, 0.0]), connected_segment_ids=["seg_1", "seg_2"])
        self.seg1 = RoadSegment("seg_1", np.array([100.0, 0.0, 0.0]), np.array([200.0, 0.0, 0.0]), connected_segment_ids=["seg_3"])
        self.seg2 = RoadSegment("seg_2", np.array([100.0, 0.0, 0.0]), np.array([100.0, 100.0, 0.0]))
        self.seg3 = RoadSegment("seg_3", np.array([200.0, 0.0, 0.0]), np.array([200.0, -100.0, 0.0]))

        self.graph = RoadGraph()
        self.graph.add_segment(self.seg0)
        self.graph.add_segment(self.seg1)
        self.graph.add_segment(self.seg2)
        self.graph.add_segment(self.seg3)

        self.matcher = HMMMapMatcher(graph=self.graph, search_radius=30.0)

    def test_candidate_generation(self):
        # Query near (50, 5, 0) - should find seg_0 within search_radius=30
        cands = self.graph.find_candidate_segments(np.array([50.0, 5.0, 0.0]), search_radius=30.0)
        cand_ids = [c.segment_id for c in cands]
        self.assertIn("seg_0", cand_ids)
        self.assertNotIn("seg_3", cand_ids)

    def test_viterbi_decoding(self):
        # Trellis of 3 steps
        emissions = [
            np.array([-1.0, -5.0]),  # Step 0: state 0 preferred
            np.array([-4.0, -0.5]),  # Step 1: state 1 preferred
            np.array([-0.2, -6.0])   # Step 2: state 0 preferred
        ]
        # Transition matrices
        trans = [
            np.array([[0.0, -2.0], [-5.0, 0.0]]),  # 0->0 easy, 1->1 easy
            np.array([[0.0, -1.0], [-1.0, 0.0]])
        ]
        path, max_prob = ViterbiDecoder.decode(emissions, trans)
        self.assertEqual(len(path), 3)
        self.assertIsInstance(max_prob, float)

    def test_uncertainty_integration(self):
        # Low DR uncertainty vs High DR uncertainty
        dr_pos = np.array([50.0, 10.0, 0.0])
        
        # Emission log prob with low uncertainty (sigma=1.0 -> max(2,1)=2.0)
        log_p_low = self.matcher.compute_emission_log_prob(dr_pos, self.seg0, pos_uncertainty=1.0)
        # Emission log prob with high uncertainty (sigma=20.0)
        log_p_high = self.matcher.compute_emission_log_prob(dr_pos, self.seg0, pos_uncertainty=20.0)

        # High uncertainty gives higher log likelihood for off-road positions (wider tolerance)
        self.assertGreater(log_p_high, log_p_low)

    def test_heading_consistency_and_turn_cost(self):
        # Vehicle heading East (pi/2) matching seg_0 (heading pi/2)
        h_prob_match = self.matcher.compute_heading_log_prob(np.pi / 2.0, self.seg0)
        # Vehicle heading North (0) matching seg_0 (heading pi/2)
        h_prob_diff = self.matcher.compute_heading_log_prob(0.0, self.seg0)

        self.assertGreater(h_prob_match, h_prob_diff)

        # Turn cost for straight transition (seg0 -> seg1)
        cost_straight = self.matcher.compute_turn_cost_log_prob(self.seg0, self.seg1)
        # Turn cost for U-turn / reverse (seg0 -> reverse seg0)
        rev_seg = RoadSegment("rev", np.array([100.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))
        cost_uturn = self.matcher.compute_turn_cost_log_prob(self.seg0, rev_seg)

        self.assertGreater(cost_straight, cost_uturn)

    def test_sequence_map_matching(self):
        # Simulate trajectory along seg0 then turning into seg2 (North)
        waypoints = [
            (np.array([20.0, 2.0, 0.0]), np.pi / 2.0),
            (np.array([60.0, 1.0, 0.0]), np.pi / 2.0),
            (np.array([98.0, 30.0, 0.0]), 0.0),        # Turning North onto seg2
            (np.array([99.0, 70.0, 0.0]), 0.0),
        ]

        results = []
        for pos, head in waypoints:
            res = self.matcher.match_step(dr_pos=pos, heading=head, pos_uncertainty=3.0)
            results.append(res)

        self.assertFalse(results[-1].is_fallback)
        self.assertEqual(results[-1].matched_segment.segment_id, "seg_2")
        self.assertGreater(results[-1].confidence, 0.0)
        self.assertIn("seg_2", results[-1].candidate_probabilities)

    def test_fallback_behavior(self):
        # Position far off graph (1000, 1000, 0)
        res = self.matcher.match_step(dr_pos=np.array([1000.0, 1000.0, 0.0]))
        self.assertTrue(res.is_fallback)
        np.testing.assert_allclose(res.matched_position, np.array([1000.0, 1000.0, 0.0]))

        # Test ProbabilisticMapMatcher wrapper with fallback polyline
        poly = np.array([[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]])
        prob_matcher = ProbabilisticMapMatcher(reference_route=poly, max_snap_distance=20.0, search_radius=10.0)
        
        state = NavigationState(
            timestamp=1.0,
            position=np.array([50.0, 5.0, 0.0]),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3)
        )
        out_state = prob_matcher.match(state)
        self.assertIsNotNone(out_state.position)
        # Should snap close to y=0
        self.assertAlmostEqual(out_state.position[1], 0.0, delta=1.0)

if __name__ == '__main__':
    unittest.main()

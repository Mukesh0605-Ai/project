import numpy as np
import typing
from dataclasses import dataclass
from src.map.road_graph import RoadGraph, RoadSegment
from src.map.viterbi import ViterbiDecoder

@dataclass
class HMMMatchResult:
    """Output dataclass from HMM Map Matching."""
    matched_position: np.ndarray  # [x, y, z]
    matched_segment: typing.Optional[RoadSegment]
    confidence: float             # 0.0 to 1.0
    candidate_probabilities: typing.Dict[str, float]
    is_fallback: bool

@dataclass
class HMMObservation:
    dr_pos: np.ndarray
    heading: float
    pos_uncertainty: float
    timestamp: float
    candidates: typing.Optional[typing.List[RoadSegment]] = None
    emission_log_probs: typing.Optional[np.ndarray] = None

class HMMMapMatcher:
    """
    Hidden Markov Model (HMM) Road Network Matcher.
    
    Required Components:
      1. Candidate road generation (spatial radius search)
      2. Emission probability (Gaussian distance vs DR uncertainty)
      3. Transition probability (topological network distance vs DR step distance)
      4. Road connectivity checks
      5. Heading consistency (angle difference penalty)
      6. Turn cost (sharp turn / u-turn penalty)
      7. Viterbi decoding (global sequence likelihood maximization)
    """
    def __init__(self,
                 graph: typing.Optional[RoadGraph] = None,
                 search_radius: float = 50.0,
                 beta_transition: float = 10.0,
                 sigma_heading: float = np.pi / 6.0,  # 30 deg
                 window_size: int = 10):
        self.graph = graph if graph is not None else RoadGraph()
        self.search_radius = search_radius
        self.beta_transition = beta_transition
        self.sigma_heading = sigma_heading
        self.window_size = window_size

        self._history: typing.List[HMMObservation] = []
        self._last_matched_segment: typing.Optional[RoadSegment] = None

    def set_graph(self, graph: RoadGraph):
        self.graph = graph
        self._history.clear()
        self._last_matched_segment = None

    def compute_emission_log_prob(self, dr_pos: np.ndarray, segment: RoadSegment, pos_uncertainty: float) -> float:
        """
        Emission Probability: P(z_t | c_i)
        High DR Uncertainty -> larger sigma_z -> wider distance tolerance -> topological trust.
        Low DR Uncertainty -> smaller sigma_z -> tighter distance tolerance -> DR trust.
        """
        proj, dist, _ = self.graph.project_point_to_segment(dr_pos, segment)
        sigma_z = max(2.0, pos_uncertainty)
        
        # Gaussian emission log likelihood
        log_emit = - (dist ** 2) / (2.0 * (sigma_z ** 2)) - np.log(np.sqrt(2.0 * np.pi) * sigma_z)
        return float(log_emit)

    def compute_heading_log_prob(self, vehicle_heading: float, segment: RoadSegment) -> float:
        """
        Heading Consistency: P(theta_veh | theta_road)
        """
        road_heading = segment.heading
        # Angular difference normalized to [-pi, pi]
        diff = vehicle_heading - road_heading
        diff = float(np.arctan2(np.sin(diff), np.cos(diff)))
        
        log_heading = - (diff ** 2) / (2.0 * (self.sigma_heading ** 2))
        return float(log_heading)

    def compute_turn_cost_log_prob(self, prev_segment: RoadSegment, curr_segment: RoadSegment) -> float:
        """
        Turn Cost Penalty for sharp turns or U-turns.
        """
        if prev_segment.segment_id == curr_segment.segment_id:
            return 0.0

        diff = curr_segment.heading - prev_segment.heading
        diff = abs(float(np.arctan2(np.sin(diff), np.cos(diff))))

        if diff > 3.0 * np.pi / 4.0:  # U-turn (>135 deg)
            return -5.0
        elif diff > np.pi / 2.0:      # Sharp turn (>90 deg)
            return -2.5
        return 0.0

    def compute_transition_log_prob(self,
                                    prev_segment: RoadSegment,
                                    curr_segment: RoadSegment,
                                    dr_step_dist: float) -> float:
        """
        Transition Probability: P(c_j,t | c_i,t-1)
        Evaluates topological network distance vs DR displacement step.
        """
        net_dist = self.graph.shortest_path_distance(prev_segment, curr_segment)
        if np.isinf(net_dist):
            # Unconnected segments receive topological penalty
            return -12.0

        delta_d = abs(net_dist - dr_step_dist)
        log_trans = - delta_d / self.beta_transition

        # Turn cost integration
        turn_cost = self.compute_turn_cost_log_prob(prev_segment, curr_segment)
        return float(log_trans + turn_cost)

    def match_step(self,
                   dr_pos: np.ndarray,
                   heading: float = 0.0,
                   pos_uncertainty: float = 5.0,
                   timestamp: float = 0.0) -> HMMMatchResult:
        """
        Main matching step.
        Evaluates candidates, calculates emission/heading/transition log probabilities, 
        and decodes optimal path via Viterbi algorithm.
        """
        cands = self.graph.find_candidate_segments(dr_pos, search_radius=self.search_radius)

        if not cands:
            # Fallback to un-snapped DR position if no candidates within search radius
            return HMMMatchResult(
                matched_position=dr_pos.copy(),
                matched_segment=self._last_matched_segment,
                confidence=0.1,
                candidate_probabilities={},
                is_fallback=True
            )

        emits = []
        for seg in cands:
            e_log = self.compute_emission_log_prob(dr_pos, seg, pos_uncertainty)
            h_log = self.compute_heading_log_prob(heading, seg)
            emits.append(e_log + h_log)
        emit_arr = np.array(emits, dtype=np.float64)

        obs = HMMObservation(
            dr_pos=dr_pos,
            heading=heading,
            pos_uncertainty=pos_uncertainty,
            timestamp=timestamp,
            candidates=cands,
            emission_log_probs=emit_arr
        )
        self._history.append(obs)
        if len(self._history) > self.window_size:
            self._history.pop(0)

        # 2. Build Trellis for Sliding Window Viterbi Decoding
        T_steps = len(self._history)
        window_candidates = [h.candidates for h in self._history]
        emission_log_probs = [h.emission_log_probs for h in self._history]


        # 3. Build Transition Log Matrices between sliding window steps
        transition_matrices = []
        for t in range(T_steps - 1):
            prev_cands = window_candidates[t]
            curr_cands = window_candidates[t + 1]
            dr_step = float(np.linalg.norm(self._history[t + 1].dr_pos[:2] - self._history[t].dr_pos[:2]))

            mat = np.zeros((len(prev_cands), len(curr_cands)), dtype=np.float64)
            for i, p_seg in enumerate(prev_cands):
                for j, c_seg in enumerate(curr_cands):
                    mat[i, j] = self.compute_transition_log_prob(p_seg, c_seg, dr_step)
            transition_matrices.append(mat)

        # 4. Viterbi Sequence Decoding
        optimal_indices, max_score = ViterbiDecoder.decode(emission_log_probs, transition_matrices)

        best_cand_idx = optimal_indices[-1]
        best_segment = window_candidates[-1][best_cand_idx]
        self._last_matched_segment = best_segment

        # Project current DR position onto best segment
        snapped_pos, dist, _ = self.graph.project_point_to_segment(dr_pos, best_segment)

        # Compute candidate Softmax probabilities for output report
        curr_emits = emission_log_probs[-1]
        exp_emits = np.exp(curr_emits - np.max(curr_emits))
        cand_probs = exp_emits / np.sum(exp_emits)
        prob_dict = {
            window_candidates[-1][k].segment_id: float(cand_probs[k]) for k in range(len(cand_probs))
        }

        # 5. Map Confidence Calculation
        best_cand_prob = float(cand_probs[best_cand_idx])
        dist_factor = float(np.exp(- dist / 20.0))
        confidence = float(np.clip(best_cand_prob * dist_factor, 0.05, 0.98))

        return HMMMatchResult(
            matched_position=snapped_pos,
            matched_segment=best_segment,
            confidence=confidence,
            candidate_probabilities=prob_dict,
            is_fallback=False
        )

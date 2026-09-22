import numpy as np
import typing

class ViterbiDecoder:
    """
    Viterbi Dynamic Programming Algorithm for HMM Sequence Decoding.
    Computes the most likely sequence of hidden state candidates in log-probability domain.
    """
    @staticmethod
    def decode(emission_log_probs: typing.List[np.ndarray],
               transition_log_matrices: typing.List[np.ndarray]) -> typing.Tuple[typing.List[int], float]:
        """
        Input:
          - emission_log_probs: List of length T, where T[t] is array of shape (N_t,) with log emission probabilities.
          - transition_log_matrices: List of length T-1, where T[t] is matrix of shape (N_t, N_{t+1}) with log transition probabilities.
          
        Returns:
          - optimal_path: List of length T with index of selected candidate at each time step.
          - max_log_likelihood: Best path log likelihood score.
        """
        T = len(emission_log_probs)
        if T == 0:
            return [], -float('inf')

        if T == 1:
            best_idx = int(np.argmax(emission_log_probs[0]))
            return [best_idx], float(emission_log_probs[0][best_idx])

        # Trellis structures
        # V[t][i] stores max log prob of any path ending at state i at step t
        V = [np.array(emission_log_probs[0], dtype=np.float64)]
        backpointers = []

        for t in range(1, T):
            prev_v = V[t - 1]  # shape (N_{t-1},)
            trans_mat = transition_log_matrices[t - 1]  # shape (N_{t-1}, N_t)
            curr_emit = emission_log_probs[t]  # shape (N_t,)

            N_prev, N_curr = trans_mat.shape

            # Compute trellis scores: prev_v[:, None] + trans_mat + curr_emit[None, :]
            # Shape: (N_{t-1}, N_t)
            scores = prev_v[:, None] + trans_mat + curr_emit[None, :]

            # Max score over previous states for each current state
            max_scores = np.max(scores, axis=0)  # shape (N_t,)
            best_prev = np.argmax(scores, axis=0)  # shape (N_t,)

            V.append(max_scores)
            backpointers.append(best_prev)

        # Traceback optimal path
        best_last_idx = int(np.argmax(V[-1]))
        max_log_likelihood = float(V[-1][best_last_idx])

        optimal_path = [best_last_idx]
        curr_idx = best_last_idx
        for t in range(T - 2, -1, -1):
            curr_idx = int(backpointers[t][curr_idx])
            optimal_path.insert(0, curr_idx)

        return optimal_path, max_log_likelihood

import numpy as np


class MapMatcher:
    """
    Simulated Map Matching component for prototype demonstration.
    Projects the estimated position onto a predefined reference trajectory (road network graph proxy).
    """

    def __init__(self, reference_route: np.ndarray, max_snap_distance: float = 20.0):
        """
        reference_route: Nx3 array of [East, North, Up] points defining the road center.
        max_snap_distance: Maximum distance in meters to allow a map snap.
        """
        self.route = reference_route
        self.max_snap_distance = max_snap_distance

    def get_matched_position(self, current_pos: np.ndarray) -> np.ndarray:
        """
        Finds the closest point on the reference route to the current position.
        In a real application, this involves a Hidden Markov Model (HMM) over road segments.
        Here we project onto the nearest polyline segment.
        """
        if self.route is None or len(self.route) < 2:
            return current_pos

        best_proj = current_pos.copy()
        best_dist = np.inf

        for idx in range(len(self.route) - 1):
            p1 = self.route[idx]
            p2 = self.route[idx + 1]

            v = p2 - p1
            l2 = np.dot(v, v)
            if l2 == 0:
                dist = np.linalg.norm(current_pos - p1)
                if dist < best_dist:
                    best_dist = dist
                    best_proj = p1.copy()
                continue

            w = current_pos - p1
            t = np.clip(np.dot(w, v) / l2, 0.0, 1.0)
            proj = p1 + t * v
            dist = np.linalg.norm(current_pos - proj)

            if dist < best_dist:
                best_dist = dist
                best_proj = proj

        if best_dist <= self.max_snap_distance:
            return best_proj
        return current_pos

    @staticmethod
    def extract_route_from_gnss(gnss_enu: np.ndarray) -> np.ndarray:
        """
        Helper to simulate a map database from the ground truth GNSS trace.
        Reduces density to simulate a standard road graph.
        """
        # Take every 5th point to simulate sparse road nodes
        return gnss_enu[::5]

import numpy as np
import typing
from dataclasses import dataclass, field

@dataclass
class RoadSegment:
    """
    Representation of a single directed road segment in ENU coordinate frame.
    """
    segment_id: str
    start_node: np.ndarray  # [x, y, z]
    end_node: np.ndarray    # [x, y, z]
    connected_segment_ids: typing.List[str] = field(default_factory=list)
    speed_limit_ms: float = 13.88  # ~50 km/h

    @property
    def direction_vector(self) -> np.ndarray:
        vec = self.end_node - self.start_node
        norm = float(np.linalg.norm(vec[:2]))
        if norm < 1e-6:
            return np.array([0.0, 1.0, 0.0], dtype=np.float32)
        return vec / norm

    @property
    def heading(self) -> float:
        """Heading in radians relative to ENU North (0 rad = North, pi/2 rad = East)."""
        dx = float(self.end_node[0] - self.start_node[0])
        dy = float(self.end_node[1] - self.start_node[1])
        return float(np.arctan2(dx, dy))

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.end_node - self.start_node))

class RoadGraph:
    """
    Road Network Graph maintaining segments, connectivity, candidate search, and spatial projections.
    """
    def __init__(self):
        self.segments: typing.Dict[str, RoadSegment] = {}
        self._dist_cache: typing.Dict[typing.Tuple[str, str], float] = {}

    def add_segment(self, segment: RoadSegment):
        self.segments[segment.segment_id] = segment
        self._dist_cache.clear()

    def find_candidate_segments(self, query_pos: np.ndarray, search_radius: float = 50.0) -> typing.List[RoadSegment]:
        """
        Finds all road segments within search_radius of query_pos.
        """
        candidates = []
        for seg in self.segments.values():
            proj, dist, _ = self.project_point_to_segment(query_pos, seg)
            if dist <= search_radius:
                candidates.append(seg)
        return candidates

    def project_point_to_segment(self, query_pos: np.ndarray, segment: RoadSegment) -> typing.Tuple[np.ndarray, float, float]:
        """
        Projects query_pos onto segment.
        Returns: (projected_point_3d, perpendicular_distance_meters, offset_fraction_0_to_1)
        """
        p1 = segment.start_node
        p2 = segment.end_node
        v = p2 - p1
        l2 = float(np.dot(v[:2], v[:2]))

        if l2 < 1e-8:
            dist = float(np.linalg.norm(query_pos - p1))
            return p1.copy(), dist, 0.0

        w = query_pos - p1
        t = max(0.0, min(1.0, float(np.dot(w[:2], v[:2]) / l2)))
        projected = p1 + t * v
        dist = float(np.linalg.norm(query_pos[:2] - projected[:2]))
        return projected, dist, t

    def shortest_path_distance(self, seg_start: RoadSegment, seg_end: RoadSegment) -> float:
        """
        Calculates network topological distance between seg_start and seg_end.
        If seg_start == seg_end, distance = 0.
        If directly connected, distance = seg_start.length.
        If unconnected, returns float('inf').
        """
        key = (seg_start.segment_id, seg_end.segment_id)
        if key in self._dist_cache:
            return self._dist_cache[key]

        if seg_start.segment_id == seg_end.segment_id:
            self._dist_cache[key] = 0.0
            return 0.0

        if seg_end.segment_id in seg_start.connected_segment_ids:
            d = seg_start.length
            self._dist_cache[key] = d
            return d

        # BFS shortest path search
        queue = [(seg_start.segment_id, 0.0)]
        visited = {seg_start.segment_id}
        res_dist = float('inf')

        while queue:
            curr_id, curr_dist = queue.pop(0)
            if curr_id == seg_end.segment_id:
                res_dist = curr_dist
                break

            curr_seg = self.segments.get(curr_id)
            if not curr_seg:
                continue

            for next_id in curr_seg.connected_segment_ids:
                if next_id not in visited:
                    visited.add(next_id)
                    next_seg = self.segments.get(next_id)
                    seg_len = next_seg.length if next_seg else 10.0
                    queue.append((next_id, curr_dist + seg_len))

        self._dist_cache[key] = res_dist
        return res_dist

    @classmethod
    def from_polyline(cls, polyline: np.ndarray, search_radius: float = 50.0) -> "RoadGraph":
        """
        Constructs a connected RoadGraph from an Nx3 array of ENU trajectory vertices.
        """
        graph = cls()
        if polyline is None or len(polyline) < 2:
            return graph

        num_pts = len(polyline)
        for i in range(num_pts - 1):
            seg_id = f"seg_{i}_{i+1}"
            p1 = polyline[i]
            p2 = polyline[i+1]
            connected = [f"seg_{i+1}_{i+2}"] if i < num_pts - 2 else []
            seg = RoadSegment(
                segment_id=seg_id,
                start_node=np.array(p1, dtype=np.float32),
                end_node=np.array(p2, dtype=np.float32),
                connected_segment_ids=connected
            )
            graph.add_segment(seg)
        return graph

"""B0/B-M/B-K shadow pose chains; no global posterior or reference input.

The matcher proposal is untrusted until the caller's frozen validator accepts it.
U is the independently propagated IMU/Wheel base pose. No LiDAR updates U.
"""
from dataclasses import dataclass, field
import hashlib
import numpy as np


def transform(T, points):
    return points @ T[:3, :3].T + T[:3, 3]


def pose(T):
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError('invalid pose')
    if not np.allclose(T[3], [0, 0, 0, 1]) or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-6) or np.linalg.det(T[:3, :3]) < .999999:
        raise ValueError('non-rigid pose')
    return T.copy()


def immutable_cloud(points):
    dtype = np.float32 if np.asarray(points).dtype == np.float32 else np.float64
    p = np.array(points, dtype=dtype, copy=True)
    if p.ndim != 2 or p.shape[1] != 3 or not np.isfinite(p).all():
        raise ValueError('invalid cloud')
    p.flags.writeable = False
    return p


def cloud_hash(p):
    return hashlib.sha256(np.ascontiguousarray(p).tobytes()).hexdigest()


def voxel_centroids_reference(p, leaf=.8):
    """Retained original lexicographically ordered centroid implementation."""
    if len(p) == 0:
        return p.copy()
    _, index, count = np.unique(np.floor(p / leaf).astype(np.int64), axis=0, return_inverse=True, return_counts=True)
    out = np.zeros((len(count), 3))
    np.add.at(out, index, p)
    return out / count[:, None]


def voxel_centroids(p, leaf=.8):
    """Equivalent lexicographic grouping via checked 21-bit coordinate packing.

    No masking/truncation: coordinates outside the safe range fall back to the
    reference method. Float sums retain input sequence; resulting Gaussian map
    input remains the same .8m centroid definition, frame and ordering.
    """
    if not np.isfinite(leaf) or leaf <= 0:
        raise ValueError('invalid voxel leaf')
    if len(p) == 0:
        return p.copy()
    bins = np.floor(p / leaf)
    bound = 1 << 20
    if np.any(bins < -bound) or np.any(bins >= bound):
        return voxel_centroids_reference(p, leaf)
    coords = bins.astype(np.int64) + bound
    packed = (coords[:, 0] << 42) | (coords[:, 1] << 21) | coords[:, 2]
    _, index, count = np.unique(packed, return_inverse=True, return_counts=True)
    out = np.column_stack([np.bincount(index, weights=p[:, i], minlength=len(count)) for i in range(3)])
    return out / count[:, None]


@dataclass(frozen=True)
class Scan:
    frame_id: str
    ref_ns: int
    available_ns: int
    dense: np.ndarray
    source: np.ndarray
    source_identity: str
    dense_identity: str
    deskew_history: str
    point_ids: np.ndarray

    @staticmethod
    def create(frame_id, ref_ns, available_ns, dense, source, deskew_history, point_ids=None):
        dense, source = immutable_cloud(dense), immutable_cloud(source)
        ids = np.arange(len(dense), dtype=np.int64) if point_ids is None else np.array(point_ids, dtype=np.int64, copy=True)
        if len(ids) != len(dense) or len(np.unique(ids)) != len(ids):
            raise ValueError('invalid dense point IDs')
        ids.flags.writeable = False
        if available_ns < ref_ns:
            raise ValueError('scan unavailable at reference time')
        return Scan(str(frame_id), int(ref_ns), int(available_ns), dense, source, cloud_hash(source), cloud_hash(dense), str(deskew_history), ids)


@dataclass
class Anchor:
    scan: Scan
    O_T_base: np.ndarray
    predicted: bool
    overlap_cache: dict = field(default_factory=dict, repr=False)


@dataclass(frozen=True)
class Config:
    mode: str = 'B0'
    radius: float = 120.
    low_z: float = -10.
    high_z: float = 30.
    leaf: float = .8
    max_frames: int = 3
    max_history_ns: int = 1_000_000_000
    overlap_resolution: float = 1.5
    insert_overlap: float = .7
    remove_overlap: float = .01
    fast_overlap: bool = True

    def __post_init__(self):
        if self.mode not in ('B-P', 'B0', 'B-M', 'B-K', 'B-K-MR'):
            raise ValueError('unknown variant')
        if self.max_frames < 1 or self.max_history_ns <= 0:
            raise ValueError('invalid history bound')


def occupied(points, resolution):
    return set(map(tuple, np.floor(points / resolution).astype(np.int64)))


def support(points, keys, resolution):
    if not len(points):
        return 0.
    return sum(tuple(k) in keys for k in np.floor(points / resolution).astype(np.int64)) / len(points)


@dataclass(frozen=True)
class PackedOccupancy:
    keys: object
    packed: bool


def packed_occupancy(points, resolution):
    bins = np.floor(points / resolution)
    bound = 1 << 20
    if np.any(bins < -bound) or np.any(bins >= bound):
        return PackedOccupancy(occupied(points, resolution), False)
    c = bins.astype(np.int64) + bound
    keys = (c[:, 0] << 42) | (c[:, 1] << 21) | c[:, 2]
    return PackedOccupancy(np.unique(keys), True)


def unpack_set(occupancy):
    if not occupancy.packed:return occupancy.keys
    k = occupancy.keys;mask=(1 << 21)-1;bound=1 << 20
    c = np.column_stack([(k >> 42)-bound, ((k >> 21)&mask)-bound, (k&mask)-bound])
    return set(map(tuple,c))


def packed_union(items):
    items=list(items)
    if not items:return PackedOccupancy(np.empty(0,dtype=np.int64), True)
    if all(x.packed for x in items):
        return PackedOccupancy(np.unique(np.concatenate([x.keys for x in items])), True)
    return PackedOccupancy(set().union(*(unpack_set(x) for x in items)), False)


def packed_support(points, occupancy, resolution):
    if not len(points):return 0.
    bins=np.floor(points/resolution);bound=1 << 20
    if not occupancy.packed or np.any(bins < -bound) or np.any(bins >= bound):
        return support(points,unpack_set(occupancy),resolution)
    c=bins.astype(np.int64)+bound
    q=(c[:,0]<<42)|(c[:,1]<<21)|c[:,2]
    # Count each source point (including repeated coordinates) while the Target
    # union contains distinct occupied keys, exactly as the reference set code.
    idx=np.searchsorted(occupancy.keys,q)
    valid=idx<len(occupancy.keys)
    found=np.zeros(len(q),dtype=bool)
    found[valid]=occupancy.keys[idx[valid]]==q[valid]
    return int(found.sum())/len(points)


class LocalOdometry:
    def __init__(self, config=Config()):
        self.config = config
        self.previous = None
        self.history = []
        self.U = None
        self.O = np.eye(4)
        self.version = 0
        self.O_T_U = None

    def _target(self, scan, prediction):
        cfg = self.config
        anchors = [self.previous] if cfg.mode == 'B0' else [a for a in self.history if 0 < scan.ref_ns - a.scan.ref_ns <= cfg.max_history_ns]
        if not anchors or anchors == [None]:
            return np.empty((0, 3)), [], []
        parts, identities = [], []
        for a in anchors:
            # Transform the cloud, not merely the crop center: tilted historical
            # base axes must not rotate the O-vertical cylinder or height limits.
            p = transform(a.O_T_base, a.scan.dense)
            d = p - prediction[:3, 3]
            mask = (np.sum(d[:, :2] ** 2, axis=1) <= cfg.radius ** 2) & (d[:, 2] >= cfg.low_z) & (d[:, 2] <= cfg.high_z)
            parts.append(p[mask])
            selected_ids=a.scan.point_ids[mask]
            identities.append(dict(frame_id=a.scan.frame_id, age_ns=scan.ref_ns-a.scan.ref_ns, dense_identity=a.scan.dense_identity, predicted_anchor=a.predicted, O_T_base=a.O_T_base.tolist(), roi_center_O=prediction[:3,3].tolist(), roi_radius=cfg.radius, roi_relative_z=[cfg.low_z,cfg.high_z], voxel_leaf=cfg.leaf, selected_points=len(selected_ids), selected_ids_sha256=hashlib.sha256(selected_ids.tobytes()).hexdigest()))
        return voxel_centroids(np.vstack(parts), cfg.leaf), anchors, identities

    def _update_history(self, anchor):
        if not self.config.fast_overlap or self.config.mode not in ('B-K','B-K-MR'):
            return self._update_history_reference(anchor)
        cfg=self.config;res=cfg.overlap_resolution
        self.history=[a for a in self.history if anchor.scan.ref_ns-a.scan.ref_ns<=cfg.max_history_ns]
        def cached(a):
            if res not in a.overlap_cache:
                p=transform(a.O_T_base,a.scan.dense)
                a.overlap_cache[res]=(p,packed_occupancy(p,res))
            return a.overlap_cache[res]
        p,_=cached(anchor)
        union=packed_union(cached(a)[1] for a in self.history)
        if self.history and packed_support(p,union,res)>cfg.insert_overlap:return
        self.history.append(anchor)
        if len(self.history)<=cfg.max_frames:return
        self.history=[a for a in self.history if a is anchor or packed_support(p,cached(a)[1],res)>=cfg.remove_overlap]
        while len(self.history)>cfg.max_frames:
            scores=[]
            for a in self.history[:-1]:
                ap,ak=cached(a)
                others=packed_union(cached(b)[1] for b in self.history[:-1] if b is not a)
                scores.append(packed_support(p,ak,res)*(1-packed_support(ap,others,res)))
            self.history.pop(int(np.argmin(scores)))

    def _update_history_reference(self, anchor):
        cfg = self.config
        self.history = [a for a in self.history if anchor.scan.ref_ns-a.scan.ref_ns <= cfg.max_history_ns]
        if cfg.mode not in ('B-K','B-K-MR'):
            self.history = (self.history + [anchor])[-cfg.max_frames:]
            return
        p = transform(anchor.O_T_base, anchor.scan.dense)
        res = cfg.overlap_resolution
        keys = set().union(*(occupied(transform(a.O_T_base, a.scan.dense), res) for a in self.history))
        if self.history and support(p, keys, res) > cfg.insert_overlap:
            return
        self.history.append(anchor)
        if len(self.history) <= cfg.max_frames:
            return
        self.history = [a for a in self.history if a is anchor or support(p, occupied(transform(a.O_T_base, a.scan.dense), res), res) >= cfg.remove_overlap]
        while len(self.history) > cfg.max_frames:
            scores = []
            for a in self.history[:-1]:
                ap = transform(a.O_T_base, a.scan.dense)
                others = set().union(*(occupied(transform(b.O_T_base, b.scan.dense), res) for b in self.history[:-1] if b is not a))
                scores.append(support(p, occupied(ap, res), res) * (1-support(ap, others, res)))
            self.history.pop(int(np.argmin(scores)))

    def step(self, scan, U_T_base, matcher, validator):
        U = pose(U_T_base)
        if self.previous is not None and (scan.ref_ns <= self.previous.scan.ref_ns or scan.available_ns < self.previous.scan.available_ns or scan.frame_id == self.previous.scan.frame_id):
            raise ValueError('noncausal or duplicate scan')
        if self.U is None:
            self.U = U
            yaw = np.arctan2(U[1, 0], U[0, 0])
            c, s = np.cos(yaw), np.sin(yaw)
            self.O_T_U = np.eye(4)
            self.O_T_U[:3, :3] = [[c, s, 0], [-s, c, 0], [0, 0, 1]]
            self.O_T_U[:3, 3] = -self.O_T_U[:3, :3] @ U[:3, 3]
            self.O = self.O_T_U @ U
            anchor = Anchor(scan, self.O.copy(), True)
            self.previous = anchor
            if self.config.mode != 'B-P':
                self._update_history(anchor)
            return dict(frame_id=scan.frame_id, status='INITIALIZED', O_T_base=self.O.tolist(), O_T_U=self.O_T_U.tolist(), input_correlation='shared map-feedback Deskew', variant=self.config.mode)
        delta_U = np.linalg.inv(self.U) @ U
        predicted = self.O @ delta_U
        target, anchors, identities = (np.empty((0, 3)), [], []) if self.config.mode == 'B-P' else self._target(scan, predicted)
        self.version += 1
        request = dict(source=scan.source, source_identity=scan.source_identity, target=target, target_identity=cloud_hash(target), initial_guess=predicted.copy(), target_frame='O', source_frame='base', target_version=self.version, frame_id=scan.frame_id)
        result, accepted, failure = None, False, None
        if self.config.mode != 'B-P' and len(target):
            try:
                result = matcher(request)
                candidate = pose(result['O_T_base'])
                accepted = bool(validator(request, result))
            except Exception as exc:
                failure = type(exc).__name__ + ': ' + str(exc)
        self.O = candidate if accepted else predicted
        self.U = U  # Never replace U with a LiDAR pose.
        anchor = Anchor(scan, self.O.copy(), not accepted)
        self.previous = anchor  # Strict preceding real scan, including failure.
        if self.config.mode != 'B-P':
            self._update_history(anchor)
        return dict(frame_id=scan.frame_id, variant=self.config.mode, status='ACCEPTED' if accepted else 'PREDICTION_ONLY', O_T_base=self.O.tolist(), predicted_O_T_base=predicted.tolist(), delta_U=delta_U.tolist(), target_version=self.version, target_points=len(target), targets=identities, source_identity=scan.source_identity, target_identity=request['target_identity'], accepted=accepted, proposal=result, failure=failure, conditional_only=True)

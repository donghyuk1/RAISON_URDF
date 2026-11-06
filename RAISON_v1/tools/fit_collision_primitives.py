"""Fit conservative compound boxes/cylinders; no SciPy dependency.

Every input triangle belongs to one convex proxy that encloses all three
vertices. Splitting never clips triangles, so full surface coverage is retained.
The objective is summed primitive volume, not the volume of their union.
"""
import math
import numpy as np

PADDING = 0.0003


def subdivide(triangles, max_edge=0.006):
    """Split long faces so broad CAD triangles do not defeat spatial fitting."""
    pending = triangles.astype(float)
    complete = []
    for _ in range(24):
        lengths = np.stack([np.linalg.norm(pending[:, i] - pending[:, (i+1) % 3], axis=1)
                            for i in range(3)], axis=1)
        split = lengths.max(1) > max_edge
        complete.append(pending[~split])
        if not split.any():
            return np.concatenate(complete)
        selected = pending[split]
        edge = lengths[split].argmax(1)
        rows = np.arange(len(selected))
        a, b, c = selected[rows, edge], selected[rows, (edge+1) % 3], selected[rows, (edge+2) % 3]
        mid = (a+b)/2
        pending = np.concatenate([np.stack([a, mid, c], axis=1), np.stack([mid, b, c], axis=1)])
    raise RuntimeError('Triangle subdivision did not converge')


def rpy(matrix):
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1, 1)))
    if abs(math.cos(pitch)) > 1e-8:
        return [math.atan2(matrix[2, 1], matrix[2, 2]), pitch,
                math.atan2(matrix[1, 0], matrix[0, 0])]
    return [0, pitch, math.atan2(-matrix[0, 1], matrix[1, 1])]


def fit(triangles):
    points = triangles.reshape(-1, 3).astype(float)
    sample = points[::max(1, len(points) // 3000)]
    _, axes = np.linalg.eigh(np.cov(sample.T))
    if np.linalg.det(axes) < 0:
        axes[:, 0] *= -1
    best = None
    for frame in [np.eye(3), axes]:
        q = points @ frame
        lo, hi = q.min(0), q.max(0)
        center = (lo + hi) / 2
        size = hi - lo + 2 * PADDING
        candidate = dict(shape='box', volume=float(np.prod(size)),
                         xyz=center @ frame.T, rotation=frame.copy(), size=size)
        if best is None or candidate['volume'] < best['volume']:
            best = candidate
        for axis in range(3):
            other = [i for i in range(3) if i != axis]
            radius = float(np.linalg.norm((q - center)[:, other], axis=1).max() + PADDING)
            length = float(size[axis])
            volume = math.pi * radius ** 2 * length
            if volume >= best['volume']:
                continue
            z = frame[:, axis]
            x = frame[:, other[0]]
            rotation = np.column_stack([x, np.cross(z, x), z])
            best = dict(shape='cylinder', volume=volume, xyz=center @ frame.T,
                        rotation=rotation, radius=radius, length=length)
    return best


def partition(triangles, max_parts=5, return_regions=False):
    # Choose splits on a representative sample, then accept only reductions
    # verified on the complete source triangles.
    triangles = subdivide(triangles)
    regions = [(triangles, fit(triangles))]
    while len(regions) < max_parts:
        best = None
        for index, (tris, proxy) in enumerate(regions):
            if len(tris) < 50:
                continue
            sample = tris[::max(1, len(tris) // 1200)]
            coords = sample.mean(1) @ proxy['rotation']
            choices = []
            for axis in range(3):
                cuts = list(np.quantile(coords[:, axis], [0.1, 0.3, 0.5, 0.7, 0.9]))
                cuts += list(np.linspace(coords[:, axis].min(), coords[:, axis].max(), 7)[1:-1])
                for cut in cuts:
                    cut = float(cut)
                    mask = coords[:, axis] < cut
                    if min(mask.sum(), (~mask).sum()) < 10:
                        continue
                    score = fit(sample[mask])['volume'] + fit(sample[~mask])['volume']
                    choices.append((score, axis, cut))
            for _, axis, cut in sorted(choices):
                mask = (tris.mean(1) @ proxy['rotation'])[:, axis] < cut
                if not mask.any() or mask.all():
                    continue
                a, b = tris[mask], tris[~mask]
                pa, pb = fit(a), fit(b)
                gain = proxy['volume'] - pa['volume'] - pb['volume']
                if gain > proxy['volume'] * 0.015 and (best is None or gain > best[0]):
                    best = (gain, index, [(a, pa), (b, pb)])
        if best is None:
            break
        regions[best[1]:best[1]+1] = best[2]
    return regions if return_regions else [proxy for _, proxy in regions]

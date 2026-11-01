"""Pure numerical helpers for independent accuracy reporting."""

import numpy as np


def error_statistics(values):
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return None
    if not np.isfinite(values).all() or np.any(values < 0):
        raise ValueError('Errors must be finite and non-negative')
    return {'count': int(values.size), 'mean': float(values.mean()),
            'rms': float(np.sqrt(np.mean(values ** 2))),
            'maximum': float(values.max()),
            'p95': float(np.percentile(values, 95))}


def repeatability_statistics(points):
    """Centroid-based 3D repeatability; all distances are in input units."""
    points = np.asarray(points, dtype=float)
    if points.size == 0:
        return None
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('Repeatability samples must be an N x 3 finite array')
    centroid = points.mean(axis=0)
    radial = np.linalg.norm(points - centroid, axis=1)
    result = error_statistics(radial)
    result.update({'centroid': centroid.tolist(),
                   'axisStd': np.std(points, axis=0, ddof=1).tolist() if len(points) > 1 else [0., 0., 0.]})
    return result


def absolute_accuracy_statistics(measured, ground_truth):
    measured, ground_truth = np.asarray(measured, float), np.asarray(ground_truth, float)
    if measured.shape != ground_truth.shape or measured.ndim != 2 or measured.shape[1] != 3:
        raise ValueError('Measured and ground-truth points must be matching N x 3 arrays')
    return error_statistics(np.linalg.norm(measured - ground_truth, axis=1))

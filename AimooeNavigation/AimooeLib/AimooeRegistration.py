"""Rigid surface refinement, runnable in a separate PythonSlicer process."""
import json
import sys
import traceback
from pathlib import Path

import numpy as np
import vtk


def rows(matrix):
    return [[matrix.GetElement(i, j) for j in range(4)] for i in range(4)]


def matrix(values):
    result = vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            result.SetElement(i, j, float(values[i][j]))
    return result


def transform(points, values):
    m = np.asarray(values, float)
    return np.asarray(points, float) @ m[:3, :3].T + m[:3, 3]


def statistics(values):
    a = np.asarray(values, float)
    if not len(a):
        return None
    return dict(count=len(a), rms=float(np.sqrt(np.mean(a*a))),
                mean=float(a.mean()), maximum=float(a.max()), p95=float(np.percentile(a, 95)))


def distances(points, surface):
    locator = vtk.vtkStaticCellLocator()
    locator.SetDataSet(surface)
    locator.BuildLocator()
    output = []
    for point in points:
        closest = [0., 0., 0.]
        cell, sub, squared = vtk.reference(0), vtk.reference(0), vtk.reference(0.)
        locator.FindClosestPoint(point, closest, cell, sub, squared)
        output.append(float(squared)**.5)
    return output


def refine(points, surface, initial):
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 3 or not 20 <= len(points) <= 2000 or not np.isfinite(points).all():
        raise ValueError('需要 20–2000 个有限表面采样点')
    singular = np.linalg.svd(points-points.mean(0), compute_uv=False)
    if singular[1] < 1 or singular[1]/singular[0] < .05:
        raise ValueError('表面采点范围过窄或接近共线，请分散采样')
    if surface.GetNumberOfCells() == 0:
        raise ValueError('目标表面为空')
    initialized = transform(points, initial)
    source = vtk.vtkPolyData()
    pts, verts = vtk.vtkPoints(), vtk.vtkCellArray()
    for p in initialized:
        index = pts.InsertNextPoint(*p)
        verts.InsertNextCell(1)
        verts.InsertCellPoint(index)
    source.SetPoints(pts)
    source.SetVerts(verts)
    icp = vtk.vtkIterativeClosestPointTransform()
    icp.SetSource(source)
    icp.SetTarget(surface)
    icp.GetLandmarkTransform().SetModeToRigidBody()
    icp.StartByMatchingCentroidsOff()  # Corresponding landmarks provide initialization.
    icp.SetMaximumNumberOfIterations(60)
    icp.SetMaximumNumberOfLandmarks(2000)
    icp.SetMaximumMeanDistance(.001)
    icp.CheckMeanDistanceOn()
    icp.Update()
    result = vtk.vtkMatrix4x4()
    vtk.vtkMatrix4x4.Multiply4x4(icp.GetMatrix(), matrix(initial), result)
    before = statistics(distances(initialized, surface))
    after = statistics(distances(transform(points, rows(result)), surface))
    if not np.isfinite(rows(result)).all() or after['rms'] > before['rms'] + 1e-5:
        raise ValueError('表面拟合未改善，保留原始点配准')
    return dict(matrix=rows(result), before=before, after=after,
                iterations=icp.GetNumberOfIterations(),
                metric='sample-to-surface distance; not target registration accuracy')


if __name__ == '__main__':
    job = Path(sys.argv[1])
    try:
        data = json.loads(job.read_text(encoding='utf-8'))
        reader = vtk.vtkXMLPolyDataReader()
        reader.SetFileName(data['surface'])
        reader.Update()
        result = refine(data['points'], reader.GetOutput(), data['initial'])
        result['ok'] = True
    except Exception:
        result = dict(ok=False, error=traceback.format_exc())
    job.with_suffix('.result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')

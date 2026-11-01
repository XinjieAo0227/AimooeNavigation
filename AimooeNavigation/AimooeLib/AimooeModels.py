"""CAD profile and MRML model helpers. No CAD kernel is loaded inside Slicer."""
import json
from pathlib import Path
import vtk
from .AimooeTransforms import pose_matrix


def matrix_from_rows(rows):
    if len(rows) != 4 or list(rows[3]) != [0, 0, 0, 1]:
        raise ValueError('Expected rigid 4 x 4 matrix')
    return pose_matrix([row[:3] for row in rows[:3]], [row[3] for row in rows[:3]])


def load_profile(folder):
    path = Path(folder) / 'LTS-B4E1.json'
    profile = json.loads(path.read_text(encoding='utf-8-sig'))
    matrix_from_rows(profile['cadToTool'])
    matrix_from_rows(profile['tipToTool'])
    if profile['rmsMm'] > 0.5 or len(profile['markersTool']) != 4:
        raise ValueError('CAD marker alignment does not match the tool definition')
    for part in profile['parts']:
        file = (path.parent / part['file']).resolve()
        if file.parent != path.parent.resolve() or not file.is_file():
            raise ValueError('Missing/invalid CAD mesh resource')
    return profile


def read_mesh(path):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    if reader.GetOutput().GetNumberOfPoints() == 0:
        raise ValueError(f'Empty mesh: {path}')
    result = vtk.vtkPolyData()
    result.DeepCopy(reader.GetOutput())
    return result


def sphere(center, radius):
    source = vtk.vtkSphereSource()
    source.SetCenter(*center)
    source.SetRadius(radius)
    source.SetThetaResolution(24)
    source.SetPhiResolution(18)
    source.Update()
    return source.GetOutput()

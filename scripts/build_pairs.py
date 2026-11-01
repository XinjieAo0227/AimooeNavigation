"""Offline CAD pair conversion; launched outside Slicer with a JSON job."""
import hashlib
import itertools
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# Install optional CAD dependencies in AIMOOE_CAD_PYTHON; no vendored binaries.
import numpy as np
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.BRep import BRep_Builder
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Sphere
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer


def build(step, definition):
    step, definition = Path(step), Path(definition)
    content = definition.read_bytes()
    try:
        lines = content.decode('utf-8-sig').splitlines()
    except UnicodeDecodeError:
        lines = content.decode('gb18030').splitlines()
    count = int(lines[2])
    if count != 4:
        raise ValueError('Current CAD auto-binding requires exactly four marker centers')
    target = np.array([[float(v) for v in line.split()[:3]] for line in lines[3:7]])
    if target.shape != (4, 3) or not np.isfinite(target).all():
        raise ValueError('Invalid tool coordinates')
    reader = STEPControl_Reader()
    if reader.ReadFile(str(step)) != IFSelect_RetDone:
        raise ValueError('Cannot read STEP: ' + str(step))
    # OpenCascade transfers STEP units to its default millimetre working units.
    reader.TransferRoots()
    shape = reader.OneShape()
    faces = TopExp_Explorer(shape, TopAbs_FACE)
    spheres = []
    while faces.More():
        face = TopoDS.Face_s(faces.Current())
        surface = BRepAdaptor_Surface(face, True)
        if surface.GetType() == GeomAbs_Sphere:
            sp = surface.Sphere()
            c = sp.Location()
            entry = [c.X(), c.Y(), c.Z(), sp.Radius()]
            if not any(np.linalg.norm(np.array(entry)-old) < 1e-5 for old in spheres):
                spheres.append(entry)
        faces.Next()
    groups = {}
    for sphere in spheres:
        groups.setdefault(round(sphere[3], 4), []).append(sphere[:3])
    candidates = [(radius, np.array(points)) for radius, points in groups.items() if len(points) == 4]
    solutions = []
    for radius, centers in candidates:
        for order in itertools.permutations(range(4)):
            moving = centers[list(order)]
            u, _, vt = np.linalg.svd((moving-moving.mean(0)).T @ (target-target.mean(0)))
            rotation = vt.T @ np.diag([1, 1, np.linalg.det(vt.T@u.T)]) @ u.T
            translation = target.mean(0)-rotation@moving.mean(0)
            errors = np.linalg.norm(moving@rotation.T+translation-target, axis=1)
            solutions.append((float(np.sqrt(np.mean(errors**2))), rotation, translation, radius, moving))
    solutions.sort(key=lambda item: item[0])
    if len(solutions) < 2 or solutions[0][0] > 0.5 or solutions[1][0] < 2.0:
        raise ValueError('CAD/aimtool mismatch or ambiguous sphere correspondence: ' + str([s[0] for s in solutions[:2]]))
    rms, rotation, translation, radius, moving = solutions[0]
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = rotation, translation
    identity = hashlib.sha256(step.read_bytes()+content+b'pair-v1').hexdigest()[:24]
    destination = ROOT / 'AimooeNavigation/Resources/Models/Pairs' / identity
    destination.mkdir(parents=True, exist_ok=True)
    mesh = BRepMesh_IncrementalMesh(shape, 0.08, False, 0.2, True)
    mesh.Perform()
    if not mesh.IsDone():
        raise ValueError('STEP triangulation failed')
    compound = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(compound)
    faces = TopExp_Explorer(shape, TopAbs_FACE)
    while faces.More():
        face = TopoDS.Face_s(faces.Current())
        surface = BRepAdaptor_Surface(face, True)
        if not (surface.GetType() == GeomAbs_Sphere and abs(surface.Sphere().Radius()-radius) < 1e-4):
            builder.Add(compound, face)
        faces.Next()
    if not StlAPI_Writer().Write(compound, str(destination / 'body.stl')):
        raise ValueError('Mesh write failed')
    shutil.copy2(definition, destination / 'definition.aimtool')
    result = {'id': identity, 'toolName': lines[0].strip(), 'sourceStep': str(step),
              'sourceDefinition': str(definition), 'units': 'mm', 'cadToTool': matrix.tolist(),
              'markersTool': target.tolist(), 'markerRadiusMm': radius, 'rmsMm': rms,
              'nextBestRmsMm': solutions[1][0], 'mesh': 'body.stl',
              'toolSHA256': hashlib.sha256(content).hexdigest(),
              'stepSHA256': hashlib.sha256(step.read_bytes()).hexdigest(),
              'tipSource': 'UNCALIBRATED_MARKER_ORIGIN'}
    # Report the tool-file tip, but never silently certify a new calibration.
    result['fileTip'] = [float(v) for v in lines[8].split()[:3]]
    small = [sp for sp in spheres if sp[3] < radius/2]
    if len(small) == 1:
        result['nominalTipTool'] = (rotation @ np.array(small[0][:3])+translation).tolist()
        result['tipSource'] = 'CAD_SPHERE_CENTER_NOMINAL_UNCALIBRATED'
    (destination / 'profile.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    return str(destination / 'profile.json')


if __name__ == '__main__':
    job = Path(sys.argv[1])
    output = job.with_suffix('.result.json')
    try:
        pairs = json.loads(job.read_text(encoding='utf-8-sig'))
        profiles = [build(pair['step'], pair['definition']) for pair in pairs]
        output.write_text(json.dumps({'profiles': profiles}, ensure_ascii=False, indent=2), encoding='utf-8')
    except Exception as exc:
        output.write_text(json.dumps({'error': str(exc)}, ensure_ascii=False), encoding='utf-8')
        raise

"""Point conversion and immutable coordinate snapshots, without GUI dependencies."""
import math
import time
import vtk


def transform_point(matrix, point):
    if len(point) != 3 or not all(math.isfinite(float(x)) for x in point):
        raise ValueError('Point must contain three finite values')
    output = matrix.MultiplyPoint([float(x) for x in point] + [1.0])
    if not all(math.isfinite(x) for x in output) or abs(output[3]) < 1e-12:
        raise ValueError('Invalid transformed homogeneous point')
    return [output[i] / output[3] for i in range(3)]


def coordinate_snapshot(tip_to_tool, tool_to_tracker, reference_to_tracker,
                        tool_valid, reference_valid, reference_to_image=None,
                        image_registered=False, source='UNCALIBRATED_MARKER_ORIGIN',
                        frame=None):
    """Matrices are copied by the caller in one GUI callback (one snapshot).

    No previous pose is emitted as current after tracking loss. CAD nominal
    coordinates are explicitly distinguished from a calibrated physical tip.
    """
    origin = [0.0, 0.0, 0.0]
    tip_tool = transform_point(tip_to_tool, origin)
    rows = [dict(space='Tip', xyz=origin, status='Local origin'),
            dict(space='Tool', xyz=tip_tool, status=source)]
    tip_tracker = transform_point(tool_to_tracker, tip_tool) if tool_valid else None
    rows.append(dict(space='Tracker', xyz=tip_tracker,
                     status=source if tool_valid else 'UNAVAILABLE: tool not TRACKING'))
    tip_reference = None
    if tool_valid and reference_valid:
        inverse = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4.Invert(reference_to_tracker, inverse)
        tip_reference = transform_point(inverse, tip_tracker)
    rows.append(dict(space='Reference', xyz=tip_reference,
                     status=source if tip_reference is not None else
                     'UNAVAILABLE: tool/reference not TRACKING'))
    tip_image = None
    if tip_reference is not None and image_registered and reference_to_image is not None:
        tip_image = transform_point(reference_to_image, tip_reference)
    rows.append(dict(space='Image', xyz=tip_image,
                     status=source if tip_image is not None else
                     'UNAVAILABLE: valid tracking and ReferenceToImage registration required'))
    return {'capturedMonotonic': time.monotonic(), 'frame': frame,
            'source': source, 'rows': rows,
            'toolOriginInTracker': transform_point(tool_to_tracker, origin) if tool_valid else None}


def read_tool_definition(path):
    """Read the vendor-documented .aimtool format; never infer tip calibration."""
    from pathlib import Path
    content = Path(path).read_bytes()
    try:
        text = content.decode('utf-8-sig')
    except UnicodeDecodeError:
        text = content.decode('gb18030')
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    count = int(lines[2])
    if count < 3 or count > 200 or len(lines) < count + 6:
        raise ValueError('Invalid AimTool definition')
    markers = [[float(v) for v in lines[3+i].split()[:3]] for i in range(count)]
    tip = [float(v) for v in lines[4+count].split()]
    mid = [float(v) for v in lines[5+count].split()]
    if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in markers+[tip,mid]):
        raise ValueError('Invalid AimTool coordinates')
    return {'name': lines[0], 'type': lines[1], 'markers': markers,
            'fileTip': tip, 'fileMid': mid, 'tipCalibrationVerified': False}

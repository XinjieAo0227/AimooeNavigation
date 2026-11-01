import math
import vtk


def pose_matrix(rotation, translation):
    """Tool coordinates to tracker coordinates; no axis flips."""
    if len(rotation) != 3 or any(len(row) != 3 for row in rotation):
        raise ValueError('Rotation must be 3 x 3')
    if len(translation) != 3:
        raise ValueError('Translation must have 3 elements')
    values = [float(x) for row in rotation for x in row] + list(translation)
    if not all(math.isfinite(x) for x in values):
        raise ValueError('Non-finite pose')
    for i in range(3):
        for j in range(3):
            dot = sum(rotation[k][i] * rotation[k][j] for k in range(3))
            if abs(dot - (1 if i == j else 0)) > 0.01:
                raise ValueError('Rotation is not orthonormal')
    r = rotation
    det = (r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])
           - r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])
           + r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]))
    if abs(det - 1) > 0.01:
        raise ValueError('Rotation must be right-handed')
    matrix = vtk.vtkMatrix4x4()
    matrix.Identity()
    for i in range(3):
        for j in range(3):
            matrix.SetElement(i, j, rotation[i][j])
        matrix.SetElement(i, 3, translation[i])
    return matrix


def relative_matrix(tool, reference):
    """ToolToReference = inverse(ReferenceToTracker) * ToolToTracker."""
    inverse = vtk.vtkMatrix4x4()
    vtk.vtkMatrix4x4.Invert(reference, inverse)
    result = vtk.vtkMatrix4x4()
    vtk.vtkMatrix4x4.Multiply4x4(inverse, tool, result)
    return result


def simulation_pose(seconds):
    angle = 0.3 * math.sin(seconds)
    c, s = math.cos(angle), math.sin(angle)
    return pose_matrix([[c, 0, s], [0, 1, 0], [-s, 0, c]],
                       [40*math.sin(seconds), 25*math.cos(seconds),
                        15*math.sin(seconds*0.7)])

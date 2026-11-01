"""Project-relative resource paths and optional environment overrides."""
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def project_path(value):
    path = Path(value)
    return path if path.is_absolute() else ROOT / path


def stored_path(value):
    path = project_path(value).resolve()
    try:
        return path.relative_to(ROOT).as_posix()
    except ValueError:
        return str(path)


SDK_SOURCE = Path(os.environ.get(
    'AIMOOE_SDK_SOURCE',
    str(ROOT / 'local-sdk-source')))


def cad_python():
    import slicer
    executable = Path(os.environ.get(
        'AIMOOE_CAD_PYTHON', str(Path(slicer.app.slicerHome) / 'bin/PythonSlicer.exe')))
    if not executable.is_file():
        raise FileNotFoundError('CAD Python runtime missing: ' + str(executable))
    return executable

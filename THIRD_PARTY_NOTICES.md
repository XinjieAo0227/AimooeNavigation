# Third-party dependency and asset status

Review date: 2026-11-01. No external dependency binary/source is bundled.

| Component | Use / evidence | Distribution / status |
|---|---|---|
| 3D Slicer 5.12.0 | Runtime; PythonQt, CTK, VTK, SimpleITK supplied by Slicer. Official [about page](https://slicer.readthedocs.io/en/v5.12.0/user_guide/about.html) and [license](https://github.com/Slicer/Slicer/blob/main/License.txt). | User installs separately; BSD-style license and dependency terms remain applicable. |
| NumPy 2.4.6 | Numerical routines; [official license](https://numpy.org/doc/stable/license.html). | Installed separately; BSD-3-Clause plus bundled component notices. |
| VTK 9.6.2 | Matrix/mesh/ICP; [official copyright](https://vtk.org/about/). | Installed separately; check wheel LICENSE and bundled component notices. |
| AimPosition binding (adapter expects Python 3.12 x64; original docstring says 3.12) | Dynamic Python binding in a separate local process. No LICENSE/EULA found in supplied SDK directories or searched parent SDK package. | SDK, DLL/PYD, drivers, examples and .aimtool definitions excluded. Vendor redistribution rights remain unknown, so no vendor material is shipped. The separately authored adapter is included based on the copyright holder’s declaration; static comparison found no identical function bodies in the supplied Python demos. SDK users must obtain their own license. |
| CAD meshes, STEP, marker definitions and profiles | Original Resources/Models and SDK tool assets have no confirmed grants. | Excluded, including derived STLs/profiles. User supplies independently authorized files. |
| cadquery-ocp / OpenCascade | Optional STEP converter in scripts/build_pairs.py. Original local cadquery_ocp/LICENSE is evidence only, not a complete dependency audit. | No CAD kernel bundled. Independently install in AIMOOE_CAD_PYTHON; full pinned environment and CAD conversion are unverified. |
| FFmpeg | Optional recording process; independently set AIMOOE_FFMPEG. [Official license guidance](https://ffmpeg.org/legal.html). | No binary bundled; obligations depend on actual build/configuration. Recording not verified. |
| PyYAML / jsonschema | Optional metadata validation tooling only. | Independently installed; not vendored. |

Do not infer permission from the presence of an SDK, a public upstream repository,
or the new project LICENSE. Local audit details stay outside the candidate.

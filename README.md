# Aimooe Navigation — LaunchOptimized082

[简体中文](README.zh-CN.md) · [Installation](docs/INSTALL.md) · [Workflow](docs/USAGE.md)

**v0.1.0 source candidate, prepared for public release under the MIT License.**
Only the source and synthetic examples described below are included; SDK/runtime assets
must be independently obtained under their own terms. See validation limits before use.

Author and software copyright holder: **XinjieAo0227** (sole author; no institutional
affiliation or co-contributors, as confirmed by the author).

A 3D Slicer scripted module for optical tracking, point registration, rigid surface ICP,
image-space guidance, and independent accuracy reporting. This candidate contains only
the current LaunchOptimized082 implementation (upstream 0.8.2), not previous launchers,
snapshots, SDK binaries, tool definitions, CAD meshes, patient images or original logs.
Current implementation mixins are retained even when their internal comments mention earlier versions.

The intended GUI runtime is Windows x64 / 3D Slicer 5.12.0 / Python 3.12.
Standalone tests require Python 3.12, NumPy 2.4.6 and VTK 9.6.2.
See [validation status](docs/VALIDATION.md) for actual results and limits.

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-test.txt
.\.venv\Scripts\python.exe tests/run_tests.py
.\.venv\Scripts\python.exe examples/precision_demo.py
```

The deterministic synthetic example prints repeatability RMS **1 mm**, absolute mean
error **0.5 mm** and absolute RMS **sqrt(0.5) mm**. This validates arithmetic, not a paper
result or device precision. No downloads, SDK, hardware, secrets or patient data are needed
after installing test dependencies.

For GUI operation follow [installation](docs/INSTALL.md), set AIMOOE_SLICER_EXE and run
`LaunchOptimized082.cmd`. Simulation uses a synthetic sphere when licensed CAD is absent.
Hardware integration requires an independently obtained vendor SDK, tool definitions,
and physical calibration. CAD conversion/recording are optional and unverified.

Workflow: `NO_IMAGE → IMAGE_READY → POINT_REGISTERED → SURFACE_REGISTERED`.
Repeatability measures spread about a centroid. Absolute accuracy requires independently
established truth. FRE, ICP fitting residual and SDK errors are separate metrics.
Live coordinates with `AMBIGUOUS_SHARED_MARKERS` are diagnostic only.

Source lives in `AimooeNavigation/`, mutable/default configuration in `Config/`,
tests in `tests/`, synthetic examples in `examples/`, tooling in `scripts/`.
CI runs SDK-free tests on Windows/Linux with read-only repository permissions and has no
deployment, upload or release job. Its remote status is not yet verified.

This project source is licensed under the [MIT License](LICENSE).
[Third-party notices](THIRD_PARTY_NOTICES.md) describe excluded components.
[CITATION.cff](CITATION.cff) records the confirmed sole author and passes the official
CFF 1.2.0 schema. No DOI or published release is asserted. [Release preparation](docs/RELEASE_v0.1.0.md)
records the release scope, validation limits and publication procedure.

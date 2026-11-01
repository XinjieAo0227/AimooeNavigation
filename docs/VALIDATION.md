# Validation / 验证记录

Validation date: 2026-11-01 (Asia/Shanghai). Remote CI passed for source commit `aa7f76c314a180957632350d5241224019961d50`: [run evidence](https://github.com/XinjieAo0227/AimooeNavigation/actions/runs/41306453025).

| Check | Result / scope |
|---|---|
| Runtime | Windows, Slicer 5.12.0 bundled Python 3.12.10 x64, NumPy 2.4.6, VTK 9.6.2 |
| `PythonSlicer.exe tests/run_tests.py` | 50 tests PASS: rigid transforms, stale/invalid tracking, copied frame data, role selection, SDK mocks, precision statistics, ICP invalid inputs and known rigid translation recovery |
| `PythonSlicer.exe examples/precision_demo.py` | PASS: repeatability RMS 1.0; absolute mean 0.5, RMS 0.7071067811865476 mm |
| Slicer `integration/precision_slicer.py` | PASS with no SDK/CAD: synthetic volume and truth; point registration; manual stop/restart and automatic 30 samples; 30 absolute records; 5,610 visualization points |
| Slicer `integration/ui_slicer.py` | PASS: current 082 controls, live-coordinate refresh, synthetic precision visualization, display hold and case-node cleanup |
| Simulation frame rate | Baseline 31.7919 Hz; capture 31.9865 Hz in this local run; descriptive diagnostic, not a performance guarantee |
| Syntax/JSON/doc links/version/CI checks | `scripts/check_project.py`; YAML parsed separately with PyYAML, permissions read-only; no release job |
| CFF 1.2.0 official schema | YAML PASS; schema PASS after the author confirmed XinjieAo0227 as sole author. Earlier missing-author failure is resolved |
| License | MIT selected explicitly by XinjieAo0227; official license text and copyright notice included |
| Standard CPython 3.12 dependency installation | PASS in fresh Windows and Ubuntu GitHub Actions jobs: pinned dependencies, 50 tests, demo, structure checks and source build. Local Slicer `venv` still fails in ensurepip; use standard CPython for standalone environments |
| Windows / Ubuntu remote GitHub Actions | PASS for SDK-free standalone functionality. This does not validate Linux Slicer GUI or hardware integration |
| SDK/hardware/CAD conversion/recording | NOT RUN; excluded licensed assets and physical measurements required |

The first sandboxed unit run failed cleaning a Windows temporary directory; the full
suite passed outside sandbox. A first GUI attempt with `--no-main-window` failed because
Slicer module selection needs the main window object. Subsequent hidden-window simulation
passed. Removing vendor definitions exposed identical empty Tool/Reference choices;
the candidate now provides distinct synthetic role labels for SDK-free startup.

The original VERIFICATION_082.md contains prior device claims. Those are not adopted as
new results and the original logs are not distributed. No paper metrics or physical
accuracy are reproduced here. Simulated truth and anatomical arrays are generated in code.

To repeat the GUI checks from an extracted source directory:

```powershell
New-Item -ItemType Directory Logs -Force
& $env:AIMOOE_SLICER_EXE --no-splash --ignore-slicerrc --additional-module-paths "$PWD/AimooeNavigation" --python-script "$PWD/integration/precision_slicer.py"
& $env:AIMOOE_SLICER_EXE --no-splash --ignore-slicerrc --additional-module-paths "$PWD/AimooeNavigation" --python-script "$PWD/integration/ui_slicer.py"
```

Each script exits Slicer; inspect `Logs/precision082-test.json` and `Logs/ui082-test.json`.
Create Logs before running (private runtime output, never packaged).
For citation checks install requirements-metadata.txt and run
`python scripts/validate_metadata.py <official-1.2.0-schema.json>`; CFF validation and the MIT license gate pass for this source candidate.

本地 Windows 合成 Slicer 流程已通过；远端 Windows/Ubuntu 标准 Python 3.12 的依赖安装、50 项测试、示例和构建均通过。
Linux Slicer GUI、真实硬件、物理精度、CAD 转换和录像仍未验证。作者信息已确认、CFF 校验通过且已选择 MIT；第三方 SDK 与资产继续排除，硬件使用须独立获授权。

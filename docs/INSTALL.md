# Installation / 安装

## SDK-free numerical checks / 无 SDK 数值验证

Use Python 3.12 x64 in a fresh virtual environment. These commands apply to Windows PowerShell:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-test.txt
.\.venv\Scripts\python.exe tests/run_tests.py
.\.venv\Scripts\python.exe examples/precision_demo.py
.\.venv\Scripts\python.exe scripts/check_project.py
.\.venv\Scripts\python.exe scripts/build_source.py
```

On Linux use `python3.12 -m venv .venv` and `.venv/bin/python` for subsequent commands.
Windows/Linux CI is configured; remote CI has not run. Python wheels are used for
standalone tests only. Do not pip-install `slicer`, `qt` or `ctk` into this environment.

全新 Python 3.12 虚拟环境执行以上命令。Linux 将解释器路径换为 `.venv/bin/python`。
独立测试使用 NumPy/VTK；图形界面必须由真正的 3D Slicer 提供。

## Windows 3D Slicer runtime / Windows 图形运行环境

Install [3D Slicer 5.12.0](https://download.slicer.org/) independently. The original
workflow reports revision 34621; confirm your downloaded runtime under Help → About.
Keep the full extracted directory writable: Config/Logs and generated assets are runtime state.

```powershell
$env:AIMOOE_SLICER_EXE = 'C:\Applications\Slicer\Slicer.exe'
.\LaunchOptimized082.cmd
```

Replace the example path with your own installed executable. Alternatively add the
`AimooeNavigation` directory to Slicer Settings → Modules → Additional module paths,
restart, and select Aimooe Navigation. The first launch defaults to Simulation Mode.
Start tracking with Simulation Mode checked: a synthetic sphere replaces omitted CAD.
This is a display example with an uncalibrated origin, not physical tool geometry.

独立安装 Slicer，将示例路径改为实际路径。默认模拟模式；勾选模拟后开始跟踪，
缺少授权 CAD 时显示合成球体，针尖标记为未标定。此模式不能验证真实硬件精度。

## Optional hardware / 可选硬件

Obtain the exact AimPosition Python 3.12 x64 runtime, drivers and tool definitions
directly from the vendor under your own license. They are not in this archive.
Set `AIMOOE_SDK_RUNTIME` to its directory containing `AimPosition.pyd` and dependent DLLs.
In the UI set Tool Path to your authorized .aimtool directory, connect, and choose
distinct Tool/Reference roles. Exit other SDK clients before connecting.
Copy `Config/tracking.example.json` to `Config/tracking.json` only if you want saved defaults;
set network addresses yourself. The latter file is private and excluded from source packages.

硬件需要自行获授权的 SDK、驱动、工具定义及校准。设置 `AIMOOE_SDK_RUNTIME`；
UI 中选择工具目录、连接并分配不同角色。网络地址留空，按实际设备填写。
不得把真实病例、工具序列标识、标定数据或私有配置加入候选包。

Optional `AIMOOE_SDK_SOURCE` points to an authorized SDK source layout for CAD discovery.
Optional `AIMOOE_CAD_PYTHON` points to a separate Python with NumPy and OCP installed.
Optional `AIMOOE_FFMPEG` points to an independently licensed ffmpeg executable.
CAD conversion and recording have no reproducible locked environment yet and remain unverified.
Importing a user-owned mesh is supported without STEP conversion.

## Troubleshooting / 故障排查

- Launcher fails: define AIMOOE_SLICER_EXE and verify the path exists.
- `qt/ctk/slicer` import fails: launch inside 3D Slicer, not standalone Python.
- Missing AimPosition: check ABI, x64, dependent DLLs and AIMOOE_SDK_RUNTIME; simulation needs none.
- Missing CAD/definitions: expected exclusions; provide files you are permitted to use.
- Permission errors: extract into a writable user directory.
- 跟踪无效或共享标记歧义：停止使用该坐标作配准，检查遮挡、定义身份和角色选择。

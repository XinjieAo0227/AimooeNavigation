# Aimooe Navigation — LaunchOptimized082

[English](README.md) · [安装教程](docs/INSTALL.md) · [使用流程](docs/USAGE.md)

**v0.1.0 源码候选，已按作者选择采用 MIT 许可证，准备公开发布。**
仅包含下述源码和合成示例；SDK、运行库及资产须独立取得并遵守其自身条款。使用前请阅读验证限制。

作者与软件权利人：**XinjieAo0227**。根据作者确认，为唯一作者，无所属单位、无共同贡献者。

本工程为 3D Slicer 脚本模块，支持光学跟踪、对应点配准、刚体表面 ICP、影像空间导航和
独立精度统计。仅保留当前 LaunchOptimized082 实现（原应用版本 0.8.2），不包含旧启动器、
历史快照、SDK 二进制、工具定义、CAD 网格、患者影像和原始日志。
当前依赖的 mixin 均保留，内部注释提及早期版本不代表打包了旧版程序。

GUI 目标运行环境为 Windows x64 / 3D Slicer 5.12.0 / Python 3.12。
独立测试使用 Python 3.12、NumPy 2.4.6 和 VTK 9.6.2。
实际结果及限制见 [验证记录](docs/VALIDATION.md)。

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-test.txt
.\.venv\Scripts\python.exe tests/run_tests.py
.\.venv\Scripts\python.exe examples/precision_demo.py
```

确定性合成示例输出重复性 RMS **1 mm**、绝对误差均值 **0.5 mm**、绝对 RMS
**sqrt(0.5) mm**。该示例只验证数值计算，不复现论文指标或证明硬件精度。
安装测试依赖后不需额外下载、SDK、硬件、秘密或患者数据。

GUI 使用见 [安装教程](docs/INSTALL.md)：配置 AIMOOE_SLICER_EXE 后运行
`LaunchOptimized082.cmd`。缺少授权 CAD 时，模拟模式显示合成球体。
硬件模式须自行取得合法 SDK、工具定义并完成实物标定；CAD 转换和录像为可选未验证功能。

工作流：`NO_IMAGE → IMAGE_READY → POINT_REGISTERED → SURFACE_REGISTERED`。
重复性是样本围绕质心的离散程度，绝对精度需要独立真值。FRE、ICP 拟合残差和 SDK 误差
属于不同指标。出现 `AMBIGUOUS_SHARED_MARKERS` 的实时坐标仅用于诊断。

源码位于 `AimooeNavigation/`，配置位于 `Config/`，测试位于 `tests/`，
合成示例位于 `examples/`，工具位于 `scripts/`。
CI 配置在 Windows/Linux 执行无 SDK 测试，仓库权限只读，无部署、上传或发布任务；
尚未执行远端 CI。

本项目源码采用 [MIT 许可证](LICENSE)。
[第三方声明](THIRD_PARTY_NOTICES.md) 列出已排除组件。
[CITATION.cff](CITATION.cff) 已填写确认的唯一作者，并通过官方 CFF 1.2.0 schema 校验；未声明 DOI 或已发布版本。
[v0.1.0 发布准备](docs/RELEASE_v0.1.0.md) 记录发布范围、验证限制和发布流程。

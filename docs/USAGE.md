# Workflow / 使用流程

1. LaunchOptimized082 starts Slicer and selects Aimooe Navigation. Leave Simulation Mode checked for synthetic tracking; start tracking to see a synthetic sphere when CAD is absent.
2. Import your own authorized CT/MRI, select its volume and surface. No patient data is supplied.
3. Acquire corresponding image/physical landmarks and perform rigid point registration. Coordinates are millimetres; image space is RAS. A tool import is not calibration.
4. Optionally acquire distributed surface points and perform rigid ICP refinement. Inspect before applying; a fit residual is not target accuracy.
5. At each stage select an independently established single-point image-RAS truth if available. The precision button collects up to 30 distinct valid frames; press again to stop. Without truth, report repeatability only.
6. Open Optical Tracking → Live coordinates for tool/coordinate diagnostics. Shared-marker matches are ambiguous and must not establish physical tool identity.

以上步骤依次为启动模拟、导入自有授权影像、对应点刚体配准、可选表面 ICP、独立真值精度采集和实时坐标诊断。
单位为 mm，影像坐标为 RAS。不得把拟合残差或合成结果当作真实设备精度。
显示保持时间 250 ms 仅抑制闪烁，计算仍采用当前有效帧；面板默认 10 Hz、可调 1–20 Hz。
真实实验需另行记录影像版本/校验和、采点协议、工具标定、设备/SDK 版本和独立真值来源。
This package has no distributable research dataset or verified paper reproduction protocol.

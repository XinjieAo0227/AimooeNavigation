"""Image-space registration, guidance and independently referenced accuracy UI."""
import json
import time
import uuid
from pathlib import Path

import numpy as np
import qt
import slicer
import vtk

from .AimooeRegistration import rows, matrix, statistics, transform
from .AimooeNavigationCore import absolute_accuracy_statistics, repeatability_statistics

ROOT = Path(__file__).resolve().parents[2]


def effective_threshold(array, lower, upper):
    """Inclusive real-valued interval mapped exactly onto the input scalar type.

    vtkImageThreshold casts bounds to the INPUT type; e.g. uint8 0.5 becomes
    zero, incorrectly including binary background. Ceil/floor before VTK.
    """
    if not np.isfinite([lower, upper]).all() or lower > upper:
        raise ValueError('阈值范围无效：下限不能大于上限')
    if not np.isfinite(array).all():
        raise ValueError('影像包含非有限数值，不能直接阈值重建')
    low, high = float(array.min()), float(array.max())
    if np.issubdtype(array.dtype, np.integer):
        lower, upper = float(np.ceil(lower)), float(np.floor(upper))
    lower, upper = max(lower, low), min(upper, high)
    if lower > upper:
        raise ValueError('阈值范围没有包含任何有效灰度值')
    selected = int(np.count_nonzero((array >= lower) & (array <= upper)))
    if selected == 0:
        raise ValueError('阈值范围没有选中体素')
    if selected == array.size:
        raise ValueError('阈值选中了整个影像（会形成长方体），请收窄阈值排除背景')
    return lower, upper, selected


class GuidanceLogicMixin:
    def workflowState(self):
        volume = slicer.mrmlScene.GetNodeByID(getattr(self, 'activeVolumeID', None) or '')
        if volume is None:
            return 'NO_IMAGE'
        registration = self.nodes.get('ReferenceToImage')
        if (not registration or registration.GetAttribute('Aimooe.RegistrationValidated') != 'True'
                or registration.GetAttribute('Aimooe.VolumeID') != volume.GetID()):
            return 'IMAGE_READY'
        return 'SURFACE_REGISTERED' if volume.GetAttribute('Aimooe.SurfaceReport') else 'POINT_REGISTERED'

    def updateDisplayFrame(self):
        request = getattr(self, 'guidanceRequest', None)
        reg = self.nodes.get('ReferenceToImage')
        if request:
            current = (getattr(self, 'activeVolumeID', None),
                       reg.GetAttribute('Aimooe.ReferenceName') if reg else None)
            if current != request or not reg or reg.GetAttribute('Aimooe.RegistrationValidated') != 'True':
                self.guidanceRequest = None
                self.imageNavigation = False
            else:
                single = reg.GetAttribute('Aimooe.SourceFrame') == 'Tracker'
                self.imageNavigation = bool(self.running and
                    (self.visualTracking('Tool') if single else
                     ((self.visualTracking('Tool') and self.visualTracking('Reference')) or self.diagnosticPair())))
        super().updateDisplayFrame()

    def updateAssetVisibility(self):
        super().updateAssetVisibility()
        self.updateImageTip()

    def updateImageTip(self):
        tip = self.nodes.get('ImageTip')
        registration = self.nodes.get('ReferenceToImage')
        single = bool(registration and registration.GetAttribute('Aimooe.SourceFrame') == 'Tracker')
        currentTracking = self.state('Tool') == 'TRACKING' if single else self.ready()
        displayTracking = (self.visualTracking('Tool') if single else
                           (self.visualTracking('Tool') and self.visualTracking('Reference')))
        diagnostic = False if displayTracking else self.diagnosticPair()
        valid = bool(getattr(self, 'imageNavigation', False) and (displayTracking or diagnostic))
        valid = valid and bool(registration and registration.GetAttribute('Aimooe.VolumeID') ==
                               getattr(self, 'activeVolumeID', None))
        if not valid:
            if hasattr(self,'tipOverlays'):self.tipOverlays.update(None,False)
            if tip and tip.GetScene() and tip.GetDisplayNode():
                tip.GetDisplayNode().SetVisibility(False)
            return
        if not tip or not tip.GetScene():
            tip = self.node('ImageTip', 'vtkMRMLMarkupsFiducialNode')
            tip.CreateDefaultDisplayNodes()
            tip.AddControlPoint(vtk.vtkVector3d(0, 0, 0))
            tip.SetLocked(True)
            display = tip.GetDisplayNode()
            display.SetSelectedColor(1, 0, 0)
            display.SetColor(1, 0, 0)
            display.SetTextScale(0)
            display.SetGlyphTypeFromString('Sphere3D')
            display.SetUseGlyphScale(False)
            display.SetGlyphSize(3.)
            display.SetSliceProjection(True)
            display.SetSliceProjectionColor(1, 0, 0)
            display.SetSliceProjectionOpacity(1.)
        # TipToTool -> ToolToTracker -> TrackerToDisplay already yields Image RAS.
        world = vtk.vtkGeneralTransform()
        self.nodes['TipToTool'].GetTransformToWorld(world)
        xyz = world.TransformPoint(0, 0, 0)
        # A transient invalid frame keeps the last valid transform on screen,
        # while all registration/navigation calculations remain frame-strict.
        held = displayTracking and not currentTracking
        if not single and not self.ready() and not held:
            xyz = self.liveCoordinates()['rows'][4]['xyz']
            if xyz is None:
                if hasattr(self,'tipOverlays'):self.tipOverlays.update(None,False)
                tip.GetDisplayNode().SetVisibility(False)
                return
        if not hasattr(self,'tipOverlays'):
            from .AimooeTipOverlay import TipOverlays
            self.tipOverlays=TipOverlays()
        display=tip.GetDisplayNode()
        manager=slicer.app.layoutManager()
        viewIDs=[manager.threeDWidget(i).threeDView().mrmlViewNode().GetID() for i in range(manager.threeDViewCount)]
        if [display.GetNthViewNodeID(i) for i in range(display.GetNumberOfViewNodeIDs())]!=viewIDs:
            display.RemoveAllViewNodeIDs()
            for identity in viewIDs:display.AddViewNodeID(identity)
        tip.SetNthControlPointPositionWorld(0, *xyz)
        tip.GetDisplayNode().SetVisibility(True)
        tip.SetAttribute('Aimooe.ImageOwner', self.activeVolumeID)
        tip.SetAttribute('Aimooe.Quality', 'HELD_LAST_VALID' if held else
                         ('TRACKING' if currentTracking else 'DIAGNOSTIC'))
        # Follow only at display frequency; never recreate nodes or reset the 3D camera.
        now = time.monotonic()
        if getattr(self, 'followSlices', True) and now-getattr(self, '_lastSliceJump', 0) >= .1:
            slicer.modules.markups.logic().JumpSlicesToLocation(*xyz, False)
            self._lastSliceJump = now
        self.tipOverlays.update(xyz)


class GuidanceWidgetMixin:
    def setupGuidance(self):
        self.surfaceProcess = None
        self.surfaceCandidate = None
        self.surfaceCollecting = False
        self.placementState = 'IDLE'
        self.placementNodeID = None
        self.placementObserverTag = None
        self.surfaceTimer = qt.QTimer()
        self.surfaceTimer.setInterval(100)
        self.surfaceTimer.connect('timeout()', self.sampleSurface)
        self.logic.followSlices = False
        self.precisionStage = None
        self.precisionTimer = qt.QTimer()
        self.precisionTimer.setInterval(250)
        self.precisionTimer.connect('timeout()', self.collectPrecisionFrame)
        # Old controls remain available in an explicit advanced area.
        self.advancedBoxes = []
        for child in slicer.util.findChildren(self.parent):
            if child.inherits('ctkCollapsibleButton'):
                if child.text not in ('光学定位', '工具配置', '跟踪状态', '截图与录屏', '帮助与致谢'):
                    self.advancedBoxes.append(child)
                    child.hide()
                else:
                    child.collapsed = True
        form = self.section('导航工作流 · 0.8.2')
        self.layout.insertWidget(0, form.parentWidget())
        self.button(form, '使用说明', self.showHelp)
        self.button(form, '中文 / English', self.toggleLanguage)
        self.advancedCheck = qt.QCheckBox('显示高级工具与旧版控制')
        self.advancedCheck.connect('toggled(bool)', self.showAdvancedGuidance)
        form.addRow(self.advancedCheck)
        form.addRow(qt.QLabel('1 影像重建 → 2 点配准 → 3 表面精配准（可选）→ 4 导航'))
        form.addRow('当前影像', self.volumeSelector)
        self.button(form, '导入影像文件', self.importMedicalImage)
        self.button(form, '导入 DICOM 文件夹', self.importDicomFolder)
        self.lowerThreshold = qt.QDoubleSpinBox()
        self.upperThreshold = qt.QDoubleSpinBox()
        for spin in (self.lowerThreshold, self.upperThreshold):
            spin.setRange(-100000, 100000)
        self.lowerThreshold.setValue(.5)
        self.upperThreshold.setValue(100000)
        form.addRow('阈值下限', self.lowerThreshold)
        form.addRow('阈值上限', self.upperThreshold)
        self.button(form, '阈值重建（原生分割）', self.nativeThreshold)
        self.reconstructionStatus = qt.QLabel('整数影像阈值会按实际可取灰度值处理，避免把背景纳入分割。')
        self.reconstructionStatus.wordWrap = True
        form.addRow(self.reconstructionStatus)
        self.button(form, '在分割编辑器中调整', self.openNativeEditor)
        form = self.section('点配准')
        self.layout.insertWidget(1, form.parentWidget())
        self.registrationMode = qt.QComboBox()
        self.registrationMode.addItems(['双工具参考配准','长针单工具配准（Tracker 必须固定）'])
        self.registrationMode.connect('currentIndexChanged(int)', self.registrationModeChanged)
        form.addRow('配准模式',self.registrationMode)
        form.addRow('影像点（按顺序）', self.imagePoints)
        form.addRow('针尖实物点（同顺序）', self.referencePoints)
        self.button(form, '在三维表面选影像点', self.placeImagePoints)
        self.button(form, '结束选点', self.endPlacement)
        self.button(form, '采集对应针尖点', self.collectGuidancePoint)
        self.button(form, '撤销最后影像点', lambda: self.removeLast(self.imagePoints))
        self.button(form, '撤销最后实物点', lambda: self.removeLast(self.referencePoints))
        self.button(form, '计算点配准', self.registerSelected)
        form.addRow(self.registrationLabel)
        self.pointFitLabel = qt.QLabel('拟合质量：尚未完成点配准（FRE 不是绝对定位精度）')
        form.addRow(self.pointFitLabel)
        self.pointTruthSelector = self.precisionTruthSelector(form)
        self.pointPrecisionButton = self.button(form, '开始点配准精度采集', lambda: self.togglePrecisionCollection('POINT'))
        self.pointPrecisionStatus = qt.QLabel('固定针尖位置并启动采集；独立真值点可选，未提供时仅统计重复性。')
        self.pointPrecisionStatus.wordWrap = True
        form.addRow(self.pointPrecisionStatus)
        form = self.section('表面精配准')
        self.layout.insertWidget(2, form.parentWidget())
        self.surfaceSelector = slicer.qMRMLNodeComboBox()
        self.surfaceSelector.nodeTypes = ['vtkMRMLModelNode']
        self.surfaceSelector.noneEnabled = True
        self.surfaceSelector.addEnabled = False
        self.surfaceSelector.removeEnabled = False
        self.surfaceSelector.setMRMLScene(slicer.mrmlScene)
        form.addRow('目标影像表面', self.surfaceSelector)
        self.button(form, '开始针尖扫面', self.startSurfaceCollection)
        self.button(form, '停止扫面', self.stopSurfaceCollection)
        self.button(form, '清空本组表面采点', self.clearSurfacePoints)
        self.surfaceMinimumDistance=qt.QDoubleSpinBox();self.surfaceMinimumDistance.setRange(.1,20);self.surfaceMinimumDistance.setValue(1);self.surfaceMinimumDistance.setSuffix(' mm')
        self.surfaceMaximumJump=qt.QDoubleSpinBox();self.surfaceMaximumJump.setRange(1,200);self.surfaceMaximumJump.setValue(30);self.surfaceMaximumJump.setSuffix(' mm / 100 ms')
        form.addRow('最小采样间距（工程参数）',self.surfaceMinimumDistance)
        form.addRow('最大允许跳变（工程参数）',self.surfaceMaximumJump)
        self.button(form, '计算精配准', self.computeSurface)
        self.button(form, '取消后台计算', self.cancelSurface)
        self.button(form, '应用精配准结果', self.applySurface)
        self.button(form, '恢复点配准', self.restoreLandmarkRegistration)
        self.surfaceStatus = qt.QLabel('先完成点配准；扫面点需覆盖多个方向与曲面区域。')
        self.surfaceStatus.wordWrap = True
        form.addRow(self.surfaceStatus)
        self.surfaceFitLabel = qt.QLabel('拟合质量：尚未应用表面精配准（ICP 残差不是绝对定位精度）')
        form.addRow(self.surfaceFitLabel)
        self.surfaceTruthSelector = self.precisionTruthSelector(form)
        self.surfacePrecisionButton = self.button(form, '开始面配准精度采集', lambda: self.togglePrecisionCollection('SURFACE'))
        self.surfacePrecisionStatus = qt.QLabel('固定针尖位置并启动采集；独立真值点可选，未提供时仅统计重复性。')
        self.surfacePrecisionStatus.wordWrap = True
        form.addRow(self.surfacePrecisionStatus)
        form = self.section('导航与独立验证')
        self.layout.insertWidget(3, form.parentWidget())
        self.followTip = qt.QCheckBox('切片跟随针尖（关闭后显示红色投影点）')
        self.followTip.checked = False
        self.followTip.connect('toggled(bool)', lambda value: setattr(self.logic, 'followSlices', value))
        form.addRow(self.followTip)
        self.button(form, '开始共空间导航', self.startGuidance)
        self.button(form, '停止导航', self.stopImageDisplay)
        self.button(form, '居中显示影像与工具', self.fitGuidanceView)
        self.guidanceStatus = qt.QLabel('尚未导航。')
        self.guidanceStatus.wordWrap = True
        form.addRow(self.guidanceStatus)
        self.button(form, '导出本组误差报告', self.exportRegistrationReport)
        self.guidanceTabs = qt.QTabWidget()
        titles = ['导航工作流 · 0.8.2', '点配准', '表面精配准', '导航与独立验证']
        boxes = slicer.util.findChildren(self.parent)
        for title, short in zip(titles, ['影像', '点配准', '面配准', '导航']):
            box = next(child for child in boxes if child.inherits('ctkCollapsibleButton') and child.text == title)
            self.layout.removeWidget(box)
            box.collapsed = False
            self.guidanceTabs.addTab(box, short)
        self.layout.insertWidget(0, self.guidanceTabs)
        self.guidanceTabs.setMinimumHeight(530)
        self.guidanceTabs.connect('currentChanged(int)', self.guidanceTabChanged)
        for child in slicer.util.findChildren(self.parent):
            if getattr(child, 'text', '') == '艾目易导航 · 优化版 0.3.4':
                child.text = '艾目易导航 · 0.8.2'
        translations = {
            '导航工作流 · 0.8.2': 'Navigation workflow · 0.8.2',
            '使用说明': 'User guide', '显示高级工具与旧版控制': 'Show advanced controls',
            '当前影像': 'Current image', '导入影像文件': 'Import image',
            '导入 DICOM 文件夹': 'Import DICOM folder', '阈值下限': 'Lower threshold',
            '阈值上限': 'Upper threshold', '阈值重建（原生分割）': 'Reconstruct (native threshold)',
            '在分割编辑器中调整': 'Open Segment Editor', '点配准': 'Landmark registration',
            '影像点（按顺序）': 'Image landmarks (ordered)', '针尖实物点（同顺序）': 'Physical tip points (same order)',
            '在三维表面选影像点': 'Place image landmarks on 3D surface', '结束选点': 'Finish placement',
            '采集对应针尖点': 'Capture corresponding tip', '撤销最后影像点': 'Undo image point',
            '撤销最后实物点': 'Undo physical point', '计算点配准': 'Compute landmark registration',
            '表面精配准': 'Surface refinement', '目标影像表面': 'Target image surface',
            '开始针尖扫面': 'Start surface sampling', '停止扫面': 'Stop sampling',
            '清空本组表面采点': 'Clear surface samples', '计算精配准': 'Compute refinement',
            '取消后台计算': 'Cancel computation', '应用精配准结果': 'Apply refinement',
            '恢复点配准': 'Restore landmark registration', '导航与独立验证': 'Guidance and validation',
            '开始共空间导航': 'Start image-space guidance', '停止导航': 'Stop guidance',
            '居中显示影像与工具': 'Fit image and tools', '导出本组误差报告': 'Export error report',
            '切片跟随针尖（关闭后显示红色投影点）': 'Follow tip in slices (otherwise show projection)',
            '1 影像重建 → 2 点配准 → 3 表面精配准（可选）→ 4 导航': '1 Reconstruction → 2 Landmarks → 3 Surface (optional) → 4 Guidance'}
        known = {id(entry[0]) for entry in self.languageWidgets}
        self.english.update(translations)
        for child in slicer.util.findChildren(self.parent):
            label = getattr(child, 'text', None)
            if isinstance(label, str) and label in translations and id(child) not in known:
                self.languageWidgets.append((child, label, translations[label]))

    def toggleLanguage(self):
        super().toggleLanguage()
        if hasattr(self, 'guidanceTabs'):
            from .AimooeNext import LANGUAGE
            for index, name in enumerate(['Image', 'Landmarks', 'Surface', 'Guidance'] if LANGUAGE == 'en' else ['影像', '点配准', '面配准', '导航']):
                self.guidanceTabs.setTabText(index, name)

    def showAdvancedGuidance(self, visible):
        for box in self.advancedBoxes:
            box.setVisible(visible)
            box.collapsed = True

    def requireVolume(self):
        volume = self.volumeSelector.currentNode()
        if volume is None:
            raise ValueError('请先导入并选择影像')
        return volume

    def volumeLayout(self):
        # Switching layout must not replace an explicitly segmented surface with
        # automatic volume rendering or queue a later camera reset.
        self.logic.imageView = True
        self.logic.setActiveVolume(self.volumeSelector.currentNode())
        slicer.app.layoutManager().setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutFourUpView)
        self.logic.publish()
        self.logic.applyWorkspaceVisibility()

    def contextKey(self):
        volume = self.requireVolume()
        tip = vtk.vtkMatrix4x4()
        self.logic.nodes['TipToTool'].GetMatrixTransformToParent(tip)
        reference = 'TRACKER_FIXED_SINGLE_TOOL' if self.singleToolMode() else self.reference.currentText
        return json.dumps([volume.GetID(), self.registrationMode.currentIndex, reference,
                           self.tool.currentText, rows(tip)])

    def singleToolMode(self):
        return hasattr(self,'registrationMode') and self.registrationMode.currentIndex == 1

    def registrationModeChanged(self, index):
        self.endPlacement()
        self.stopSurfaceCollection()
        mode='长针单工具：Tracker 与患者必须全程刚性固定，且针尖必须有物理标定。' if index==1 else '双工具：物理点保存在 Reference 局部坐标。'
        self.registrationLabel.text=mode+' 切换模式后必须重新采点和配准。'

    def registrationReferenceName(self):
        return 'TRACKER_FIXED_SINGLE_TOOL' if self.singleToolMode() else self.reference.currentText

    def currentReferenceTip(self):
        if self.singleToolMode():
            source=self.logic.nodes['TipToTool'].GetAttribute('Aimooe.TipCalibration')
            if source!='CALIBRATED':
                raise ValueError('长针单工具配准要求已验证的物理针尖标定；当前为 '+str(source))
            if not self.logic.running or self.logic.state('Tool')!='TRACKING':
                raise ValueError('长针当前位姿无效，未采集旧坐标')
            if self.logic.simulation:
                tool,tip=[vtk.vtkMatrix4x4() for _ in range(2)]
                self.logic.nodes['ToolToTracker'].GetMatrixTransformToParent(tool)
                self.logic.nodes['TipToTool'].GetMatrixTransformToParent(tip)
                xyz=np.array(rows(tool))@np.array(rows(tip))@[0,0,0,1]
                return xyz[:3].tolist()
            sample=getattr(self.logic.tracker,'samples',{}).get('Tool')
            if (not sample or sample.get('ambiguous') or sample.get('quality') or
                    time.monotonic()-sample.get('monotonic',0)>=.2 or
                    sample.get('frame')!=self.logic.tracker.last_frame):
                raise ValueError('长针帧无效、过期、存在质量告警或不是最新帧')
            point=self.logic.liveCoordinates()['rows'][2]['xyz']
            if point is None:
                raise ValueError('当前针尖 Tracker 坐标不可用')
            return point
        if not self.logic.running or not (self.logic.ready() or self.logic.diagnosticPair()):
            raise ValueError('当前双工具位姿不可用，未采集旧坐标')
        if self.logic.simulation:
            tool, ref, tip = [vtk.vtkMatrix4x4() for _ in range(3)]
            self.logic.nodes['ToolToTracker'].GetMatrixTransformToParent(tool)
            self.logic.nodes['ReferenceToTracker'].GetMatrixTransformToParent(ref)
            self.logic.nodes['TipToTool'].GetMatrixTransformToParent(tip)
            xyz = np.linalg.inv(rows(ref)) @ np.array(rows(tool)) @ np.array(rows(tip)) @ [0, 0, 0, 1]
            return xyz[:3].tolist()
        pair = self.logic.diagnosticPair()
        # Even the ordinary path needs a matched, current acquisition frame.
        if not pair:
            raise ValueError('需要同帧、完整且无歧义的双工具数据')
        self.logic.referenceEnabled = True
        point = self.logic.liveCoordinates()['rows'][3]['xyz']
        if point is None:
            raise ValueError('当前针尖参考坐标不可用')
        return point

    def collectGuidancePoint(self):
        point = self.currentReferenceTip()
        self.createPointLists()
        physical = self.referencePoints.currentNode()
        if physical.GetTransformNodeID():
            raise ValueError('实物点必须保存在 Reference 局部坐标')
        if physical.GetNumberOfControlPoints() and physical.GetAttribute('Aimooe.SampleContext') != self.contextKey():
            raise ValueError('已有实物点的工具或针尖来源不同，请删除旧实物点重新采集')
        physical.SetAttribute('Aimooe.SampleContext', self.contextKey())
        if physical.GetNumberOfControlPoints() >= self.imagePoints.currentNode().GetNumberOfControlPoints():
            raise ValueError('请先选择尚未采集的对应影像点')
        physical.AddControlPoint(vtk.vtkVector3d(*point))
        self.registrationLabel.text = '已采集实物点 %d / 影像点 %d' % (physical.GetNumberOfControlPoints(), self.imagePoints.currentNode().GetNumberOfControlPoints())

    @staticmethod
    def removeLast(selector):
        node = selector.currentNode()
        if node and node.GetNumberOfControlPoints():
            node.RemoveNthControlPoint(node.GetNumberOfControlPoints()-1)

    def placeImagePoints(self):
        self.endPlacement()
        self.requireVolume()
        self.stopImageDisplay()
        self.createPointLists()
        self.volumeLayout()
        node = self.imagePoints.currentNode()
        if node is None or node.GetScene() is not slicer.mrmlScene:
            raise RuntimeError('影像点节点无效，请重新进入选点')
        node.CreateDefaultDisplayNodes()
        # Slicer 5.12 surface placement requires the visible target model to be pickable.
        volume=self.requireVolume()
        for model in slicer.util.getNodesByClass('vtkMRMLModelNode'):
            if model.GetAttribute('Aimooe.SourceVolume')==volume.GetID() and model.GetDisplayNode():
                model.SetSelectable(True)
                model.GetDisplayNode().SetSelectable(True)
        self.placementState = 'START'
        self.placementNodeID = node.GetID()
        self.placementObserverTag = node.AddObserver(
            slicer.vtkMRMLMarkupsNode.PointPositionDefinedEvent,
            self.imagePointPlaced)
        selection = slicer.app.applicationLogic().GetSelectionNode()
        selection.SetReferenceActivePlaceNodeClassName('vtkMRMLMarkupsFiducialNode')
        selection.SetActivePlaceNodeID(node.GetID())
        interaction = slicer.app.applicationLogic().GetInteractionNode()
        interaction.SetPlaceModePersistence(1)
        interaction.SetCurrentInteractionMode(interaction.Place)
        self.placementState = 'PLACING'
        self.registrationLabel.text = '在重建表面按顺序点击；完成后结束选点，再采集对应实物点。'

    def endPlacement(self):
        if not hasattr(self, 'placementState'):
            return
        self.placementState = 'FINISH'
        node = slicer.mrmlScene.GetNodeByID(self.placementNodeID or '')
        if node and self.placementObserverTag is not None:
            node.RemoveObserver(self.placementObserverTag)
        self.placementObserverTag = None
        self.placementNodeID = None
        interaction = slicer.app.applicationLogic().GetInteractionNode()
        interaction.SetCurrentInteractionMode(interaction.ViewTransform)
        interaction.SetPlaceModePersistence(0)
        selection = slicer.app.applicationLogic().GetSelectionNode()
        selection.SetActivePlaceNodeID(None)
        self.placementState = 'IDLE'

    def imagePointPlaced(self, caller, event):
        if (self.placementState != 'PLACING' or caller.GetScene() is not slicer.mrmlScene
                or caller.GetID() != self.placementNodeID):
            self.endPlacement()
            return
        self.registrationLabel.text = '已选择影像点 %d；可继续点击，完成后按“结束选点”。' % caller.GetNumberOfControlPoints()

    def guidanceTabChanged(self, index):
        if getattr(self, 'placementState', 'IDLE') == 'PLACING' and index != 1:
            self.endPlacement()

    def registerSelected(self):
        self.endPlacement()
        physical = self.referencePoints.currentNode()
        if physical and physical.GetAttribute('Aimooe.SampleContext') and physical.GetAttribute('Aimooe.SampleContext') != self.contextKey():
            raise ValueError('实物点所属工具或针尖已经改变，请重新采集')
        super().registerSelected()
        volume = self.requireVolume()
        reg=self.logic.nodes['ReferenceToImage']
        # A new landmark fit invalidates every downstream surface/navigation result.
        self.clearNavigationPresets(volume)
        volume.SetAttribute('Aimooe.SurfaceReport',None)
        volume.SetAttribute('Aimooe.PrecisionReport',None)
        for stage in ('POINT','SURFACE'): self.clearPrecisionVisualization(stage,volume)
        for key in ('Aimooe.AccuracyTarget_POINT','Aimooe.AccuracyTarget_SURFACE'):
            node=slicer.mrmlScene.GetNodeByID(volume.GetAttribute(key) or '')
            if node: slicer.mrmlScene.RemoveNode(node)
            volume.SetAttribute(key,None)
        reg.SetAttribute('Aimooe.FinalRegistrationValidated','False')
        reg.SetAttribute('Aimooe.SourceFrame','Tracker' if self.singleToolMode() else 'Reference')
        reg.SetAttribute('Aimooe.RegistrationMode','LONG_NEEDLE_SINGLE_TOOL' if self.singleToolMode() else 'DUAL_TOOL_REFERENCE')
        volume.SetAttribute('Aimooe.SavedRegistration',None)
        # Re-save after mode metadata is complete.
        m=vtk.vtkMatrix4x4();reg.GetMatrixTransformToParent(m)
        attrs={key:reg.GetAttribute(key) for key in ('Aimooe.RegistrationValidated','Aimooe.ReferenceName',
            'Aimooe.VolumeID','Aimooe.FREmm','Aimooe.SourceFrame','Aimooe.RegistrationMode')}
        volume.SetAttribute('Aimooe.SavedRegistration',json.dumps({'matrix':rows(m),'attributes':attrs}))
        volume.SetAttribute('Aimooe.LandmarkRegistration', volume.GetAttribute('Aimooe.SavedRegistration'))
        source, target = self.points(self.referencePoints.currentNode()), self.points(self.imagePoints.currentNode())
        m = vtk.vtkMatrix4x4()
        self.logic.nodes['ReferenceToImage'].GetMatrixTransformToParent(m)
        volume.SetAttribute('Aimooe.PointReport', json.dumps(statistics(np.linalg.norm(transform(source, rows(m))-target, axis=1))))
        for index in range(self.imagePoints.currentNode().GetNumberOfControlPoints()):
            self.imagePoints.currentNode().SetNthControlPointLabel(index, 'P'+str(index+1))
        volume.SetAttribute('Aimooe.GuidanceContext', self.contextKey())
        self.surfaceCandidate = None

    @staticmethod
    def clearNavigationPresets(self, volume=None):
        if getattr(self,'precisionTimer',None): self.precisionTimer.stop()
        self.precisionStage=None
        if hasattr(self,'pointPrecisionButton'): self.pointPrecisionButton.text='开始点配准精度采集'
        if hasattr(self,'surfacePrecisionButton'): self.surfacePrecisionButton.text='开始面配准精度采集'
        volume = volume or (self.volumeSelector.currentNode() if hasattr(self, 'volumeSelector') else None)
        if volume is None:
            return
        for key in ('Aimooe.NavigationTarget','Aimooe.NavigationPath','Aimooe.NavigationRoute'):
            node=slicer.mrmlScene.GetNodeByID(volume.GetAttribute(key) or '')
            if node:slicer.mrmlScene.RemoveNode(node)
            volume.SetAttribute(key,None)
        volume.SetAttribute('Aimooe.TargetPose',None)

    def cleanupNavigationPresets(self):
        if getattr(self,'precisionTimer',None): self.precisionTimer.stop()
        self.precisionStage=None
        if hasattr(self, 'endPlacement'):
            self.endPlacement()

    def ensureRegistration(self):
        volume = self.requireVolume()
        reg = self.logic.nodes.get('ReferenceToImage')
        if not reg or reg.GetAttribute('Aimooe.RegistrationValidated') != 'True' or reg.GetAttribute('Aimooe.VolumeID') != volume.GetID():
            raise ValueError('请先完成当前影像的点配准')
        if volume.GetAttribute('Aimooe.GuidanceContext') != self.contextKey():
            raise ValueError('工具、参考或针尖来源改变，请重新点配准')
        return volume, reg

    def surfaceCloud(self):
        volume, _ = self.ensureRegistration()
        node = slicer.mrmlScene.GetNodeByID(volume.GetAttribute('Aimooe.SurfaceSamples') or '')
        if node is None:
            node = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode', 'SurfaceSamples_Reference')
            node.CreateDefaultDisplayNodes()
            node.SetAttribute('Aimooe.PhysicalInput', 'True')
            node.SetAttribute('Aimooe.ImageOwner', volume.GetID())
            node.SetAttribute('Aimooe.SampleContext', self.contextKey())
            volume.SetAttribute('Aimooe.SurfaceSamples', node.GetID())
        if node.GetAttribute('Aimooe.SampleContext') != self.contextKey():
            raise ValueError('旧表面采点所属工具或针尖已改变，请清空重采')
        node.GetDisplayNode().SetVisibility(False)
        return node

    def startSurfaceCollection(self):
        self.surfaceCloud()
        self.currentReferenceTip()
        self.collectionContext = self.contextKey()
        self.surfaceCollecting = True
        self.surfaceTimer.start()

    def stopSurfaceCollection(self):
        self.surfaceCollecting = False
        self.surfaceTimer.stop()

    def sampleSurface(self):
        try:
            if self.contextKey() != self.collectionContext:
                self.stopSurfaceCollection()
                raise ValueError('工作区改变，扫面已停止')
            node = self.surfaceCloud()
            point = self.currentReferenceTip()
            count = node.GetNumberOfControlPoints()
            if count >= 2000:
                self.stopSurfaceCollection()
                return
            if count:
                previous = [0., 0., 0.]
                node.GetNthControlPointPosition(count-1, previous)
                if np.linalg.norm(np.asarray(point)-previous) < self.surfaceMinimumDistance.value:
                    return
                if np.linalg.norm(np.asarray(point)-previous) > self.surfaceMaximumJump.value:
                    raise ValueError('相邻采样跳变超过 %.1f mm/100 ms，拒绝本帧'%self.surfaceMaximumJump.value)
            node.AddControlPoint(vtk.vtkVector3d(*point))
            preview = self.logic.node('SurfacePreview_'+self.requireVolume().GetID(), 'vtkMRMLMarkupsFiducialNode')
            preview.CreateDefaultDisplayNodes()
            preview.SetAttribute('Aimooe.ImageOwner', self.requireVolume().GetID())
            preview.SetAndObserveTransformNodeID(self.logic.nodes['ReferenceToImage'].GetID())
            preview.GetDisplayNode().SetSelectedColor(.8, .1, .8)
            preview.GetDisplayNode().SetTextScale(0)
            preview.GetDisplayNode().SetGlyphScale(.7)
            preview.AddControlPoint(vtk.vtkVector3d(*point))
            self.surfaceStatus.text = '正在采集：%d 点（最小间隔 %.1f mm）；保持针尖接触同一实物表面。'%(count+1,self.surfaceMinimumDistance.value)
        except Exception as exc:
            self.surfaceStatus.text = '本帧未采点：'+str(exc)

    def clearSurfacePoints(self):
        self.stopSurfaceCollection()
        volume = self.requireVolume()
        node = slicer.mrmlScene.GetNodeByID(volume.GetAttribute('Aimooe.SurfaceSamples') or '')
        if node:
            node.RemoveAllControlPoints()
            node.SetAttribute('Aimooe.SampleContext', self.contextKey())
        preview = self.logic.nodes.get('SurfacePreview_'+volume.GetID())
        if preview:
            preview.RemoveAllControlPoints()
        self.surfaceCandidate = None
        self.surfaceStatus.text = '表面采点已清空'

    def computeSurface(self):
        self.stopSurfaceCollection()
        if self.surfaceProcess:
            raise ValueError('正在计算，请等待或取消')
        volume, reg = self.ensureRegistration()
        target = self.surfaceSelector.currentNode()
        if not target or not target.GetPolyData() or target.GetAttribute('Aimooe.SourceVolume') != volume.GetID():
            raise ValueError('请选择本影像的重建表面，不能用导航工具模型作为目标')
        self.surfaceJob = ROOT/'Logs'/('surface-'+uuid.uuid4().hex+'.json')
        world = vtk.vtkGeneralTransform()
        slicer.vtkMRMLTransformNode.GetTransformBetweenNodes(target.GetParentTransformNode(), None, world)
        filt = vtk.vtkTransformPolyDataFilter()
        filt.SetInputData(target.GetPolyData())
        filt.SetTransform(world)
        filt.Update()
        writer = vtk.vtkXMLPolyDataWriter()
        writer.SetFileName(str(self.surfaceJob.with_suffix('.vtp')))
        writer.SetInputData(filt.GetOutput())
        if not writer.Write():
            raise IOError('无法写入表面计算文件')
        m = vtk.vtkMatrix4x4()
        reg.GetMatrixTransformToParent(m)
        self.jobContext = (self.contextKey(), reg.GetAttribute('Aimooe.FREmm'), rows(m), target.GetID(), target.GetMTime())
        payload = dict(points=self.points(self.surfaceCloud()), initial=rows(m), surface=str(self.surfaceJob.with_suffix('.vtp')))
        self.surfaceJob.write_text(json.dumps(payload), encoding='utf-8')
        process = qt.QProcess()
        self.surfaceProcess = process
        process.setStandardOutputFile(str(self.surfaceJob.with_suffix('.stdout.txt')))
        process.setStandardErrorFile(str(self.surfaceJob.with_suffix('.stderr.txt')))
        process.connect('finished(int,QProcess::ExitStatus)', self.surfaceFinished)
        process.connect('errorOccurred(QProcess::ProcessError)', self.surfaceFailed)
        process.start(str(Path(slicer.app.slicerHome)/'bin/PythonSlicer.exe'), [str(ROOT/'AimooeNavigation/AimooeLib/AimooeRegistration.py'), str(self.surfaceJob)])
        self.surfaceStatus.text = '后台计算中；不会自动覆盖点配准。'

    def surfaceFailed(self, error):
        self.surfaceStatus.text = '后台计算启动失败：'+str(error)
        self.logic.log.error(self.surfaceStatus.text)
        self.surfaceProcess = None

    def candidateContextValid(self):
        volume, reg = self.ensureRegistration()
        current = vtk.vtkMatrix4x4()
        reg.GetMatrixTransformToParent(current)
        target = slicer.mrmlScene.GetNodeByID(self.jobContext[3])
        return (self.contextKey() == self.jobContext[0] and rows(current) == self.jobContext[2]
                and target is not None and target.GetMTime() == self.jobContext[4])

    def surfaceFinished(self, code, status):
        self.surfaceProcess = None
        try:
            result = json.loads(self.surfaceJob.with_suffix('.result.json').read_text(encoding='utf-8'))
            if code or not result['ok']:
                raise ValueError(result.get('error', '后台计算失败'))
            if not self.candidateContextValid():
                raise ValueError('工作区或目标表面已改变，计算结果已丢弃')
            self.surfaceCandidate = result
            self.surfaceStatus.text = '拟合 RMS %.3f → %.3f mm；最大 %.3f mm，P95 %.3f mm。可应用；仍需独立靶点验证。' % (result['before']['rms'], result['after']['rms'], result['after']['maximum'], result['after']['p95'])
        except Exception as exc:
            self.surfaceCandidate = None
            self.surfaceStatus.text = str(exc)
            self.logic.log.exception('Surface registration failed')

    def cancelSurface(self):
        if self.surfaceProcess:
            self.surfaceProcess.disconnect('finished(int,QProcess::ExitStatus)', self.surfaceFinished)
            self.surfaceProcess.kill()
            self.surfaceProcess = None
        self.surfaceCandidate = None
        if hasattr(self, 'surfaceStatus'):
            self.surfaceStatus.text = '已取消计算，原配准不变'

    def applySurface(self):
        if not self.surfaceCandidate or not self.candidateContextValid():
            raise ValueError('没有可应用的当前工作区结果')
        volume, reg = self.ensureRegistration()
        saved = json.loads(volume.GetAttribute('Aimooe.SavedRegistration'))
        self.clearNavigationPresets(volume)
        saved['matrix'] = self.surfaceCandidate['matrix']
        reg.SetMatrixTransformToParent(matrix(saved['matrix']))
        reg.SetAttribute('Aimooe.RegistrationMethod', 'Landmarks + rigid surface ICP')
        reg.SetAttribute('Aimooe.FinalRegistrationValidated', 'True')
        volume.SetAttribute('Aimooe.SavedRegistration', json.dumps(saved))
        volume.SetAttribute('Aimooe.SurfaceReport', json.dumps(self.surfaceCandidate))
        report=self.precisionReport(volume); report['SURFACE']={'repeatability':[],'absolute':[]}
        volume.SetAttribute('Aimooe.PrecisionReport',json.dumps(report))
        self.clearPrecisionVisualization('SURFACE',volume)
        self.surfaceCandidate = None
        self.logic.publish()
        self.surfaceStatus.text += '\n已应用；可恢复点配准。'

    def restoreLandmarkRegistration(self):
        volume, reg = self.ensureRegistration()
        saved = volume.GetAttribute('Aimooe.LandmarkRegistration')
        if not saved:
            raise ValueError('没有本组点配准备份')
        self.clearNavigationPresets(volume)
        reg.SetMatrixTransformToParent(matrix(json.loads(saved)['matrix']))
        reg.SetAttribute('Aimooe.FinalRegistrationValidated', 'False')
        volume.SetAttribute('Aimooe.SavedRegistration', saved)
        volume.SetAttribute('Aimooe.SurfaceReport', None)
        report=self.precisionReport(volume); report['SURFACE']={'repeatability':[],'absolute':[]}
        volume.SetAttribute('Aimooe.PrecisionReport',json.dumps(report))
        self.clearPrecisionVisualization('SURFACE',volume)
        self.surfaceCandidate = None
        self.logic.publish()
        self.surfaceStatus.text = '已恢复点配准'

    def startGuidance(self):
        volume, reg = self.ensureRegistration()
        self.currentReferenceTip()
        ids = [node.GetID() for node, _, _ in self.logic.assetBindings if node.GetScene() and
               (node.GetAttribute('Aimooe.Pair') or node.GetAttribute('Aimooe.TipTarget'))]
        volume.SetAttribute('Aimooe.SelectedModels', json.dumps(ids))
        self.logic.imageView = True
        self.volumeLayout()
        self.logic.guidanceRequest = (volume.GetID(), reg.GetAttribute('Aimooe.ReferenceName'))
        self.logic.publish()
        self.logic.cameraFitPending = False
        self.guidanceStatus.text = '影像空间导航中；红点为针尖。'+('模拟数据。' if self.logic.simulation else '')

    def fitGuidanceView(self):
        volume = self.requireVolume()
        bounds = [0.]*6
        volume.GetRASBounds(bounds)
        for node, _, _ in self.logic.assetBindings:
            if node.GetScene() and node.GetDisplayNode().GetVisibility():
                current = [0.]*6
                node.GetRASBounds(current)
                for i in range(3):
                    bounds[2*i] = min(bounds[2*i], current[2*i])
                    bounds[2*i+1] = max(bounds[2*i+1], current[2*i+1])
        center = [(bounds[2*i]+bounds[2*i+1])*.5 for i in range(3)]
        radius = max(sum(((bounds[2*i+1]-bounds[2*i])*.5)**2 for i in range(3))**.5, 1.)
        manager = slicer.app.layoutManager()
        # Slicer can keep cached custom-layout views: index 0 need not be the
        # visible Four-Up image view. Configure every current 3D widget.
        for index in range(manager.threeDViewCount):
            view = manager.threeDWidget(index).threeDView()
            view.mrmlViewNode().SetRenderMode(slicer.vtkMRMLViewNode.Orthographic)
            renderer = view.renderWindow().GetRenderers().GetFirstRenderer()
            cameraNode = slicer.modules.cameras.logic().GetViewActiveCameraNode(view.mrmlViewNode())
            camera = cameraNode.GetCamera()
            camera.SetFocalPoint(*center)
            camera.SetPosition(center[0], center[1]-3*radius, center[2]+radius)
            camera.SetViewUp(0, 0, 1)
            camera.ParallelProjectionOn()
            width, height = renderer.GetSize()
            camera.SetParallelScale(radius*1.1/max(min(max(width,1)/max(height,1),1.),.01))
            renderer.ResetCameraClippingRange(bounds)
            cameraNode.Modified()
            view.forceRender()

    def stopImageDisplay(self):
        self.logic.guidanceRequest = None
        super().stopImageDisplay()

    def precisionReport(self, volume):
        report=json.loads(volume.GetAttribute('Aimooe.PrecisionReport') or '{}')
        for stage in ('POINT','SURFACE'):
            report.setdefault(stage, {'repeatability':[], 'absolute':[]})
        return report

    def stageMatrix(self, stage, volume, reg):
        if stage=='POINT':
            saved=volume.GetAttribute('Aimooe.LandmarkRegistration')
            if not saved: raise ValueError('点配准尚未完成')
            return json.loads(saved)['matrix']
        if not volume.GetAttribute('Aimooe.SurfaceReport'):
            raise ValueError('表面精配准尚未应用')
        current=vtk.vtkMatrix4x4(); reg.GetMatrixTransformToParent(current); return rows(current)

    def measuredTipForStage(self, stage):
        volume,reg=self.ensureRegistration(); source=self.currentReferenceTip()
        stageMatrix=self.stageMatrix(stage,volume,reg)
        return volume, transform([source], stageMatrix)[0].tolist(), stageMatrix

    def precisionTruthSelector(self, form):
        """An independently measured image-space fiducial is optional but required for absolute error."""
        selector=slicer.qMRMLNodeComboBox()
        selector.nodeTypes=['vtkMRMLMarkupsFiducialNode']
        selector.noneEnabled=True
        selector.addEnabled=False
        selector.removeEnabled=False
        selector.setMRMLScene(slicer.mrmlScene)
        selector.setCurrentNode(None)
        form.addRow('独立真值点（可选，影像 RAS）',selector)
        return selector

    def togglePrecisionCollection(self, stage):
        if self.precisionStage:
            if self.precisionStage!=stage:
                raise ValueError('请先完成当前阶段的精度采集')
            self.finishPrecisionCollection()
            return
        volume,_=self.ensureRegistration()
        self.stageMatrix(stage,volume,self.logic.nodes['ReferenceToImage'])
        selector=self.pointTruthSelector if stage=='POINT' else self.surfaceTruthSelector
        truth=selector.currentNode()
        if truth:
            if (truth.GetNumberOfControlPoints()!=1 or
                    truth in (self.imagePoints.currentNode(),self.referencePoints.currentNode())):
                raise ValueError('独立真值必须是单独建立的一个影像 RAS 点，不能使用配准点列表')
            owner=truth.GetAttribute('Aimooe.ImageOwner')
            if owner and owner!=volume.GetID():
                raise ValueError('独立真值点属于另一影像，请选择当前病例的点')
        report=self.precisionReport(volume)
        report[stage]={'repeatability':[],'absolute':[]}
        volume.SetAttribute('Aimooe.PrecisionReport',json.dumps(report))
        self.clearPrecisionVisualization(stage,volume)
        self.precisionVolumeID=volume.GetID()
        self.precisionMatrix=self.stageMatrix(stage,volume,self.logic.nodes['ReferenceToImage'])
        self.precisionTruth=None
        if truth:
            point=[0.,0.,0.];truth.GetNthControlPointPositionWorld(0,point)
            self.precisionTruth={'id':truth.GetID(),'xyz':point}
        self.precisionStage=stage
        button=self.pointPrecisionButton if stage=='POINT' else self.surfacePrecisionButton
        button.text='停止并查看'+('点配准' if stage=='POINT' else '面配准')+'精度'
        self.precisionTimer.start()
        self.collectPrecisionFrame()

    def collectPrecisionFrame(self):
        """Collect one accepted SDK frame; never infer absolute accuracy without ground truth."""
        stage=self.precisionStage
        if not stage: return
        status=self.pointPrecisionStatus if stage=='POINT' else self.surfacePrecisionStatus
        try:
            volume,measured,stageMatrix=self.measuredTipForStage(stage)
            if volume.GetID()!=self.precisionVolumeID or stageMatrix!=self.precisionMatrix:
                self.precisionTimer.stop();self.precisionStage=None
                status.text='配准变换或影像已改变，采集终止；请重新开始。'
                return
            report=self.precisionReport(volume)
            frame=getattr(getattr(self.logic,'tracker',None),'last_frame',None)
            samples=report[stage]['repeatability']
            if frame is not None and any(item.get('frame')==frame for item in samples): return
            samples.append({'image':measured,'frame':frame,'matrix':stageMatrix,'time':time.time()})
            if self.precisionTruth:
                report[stage]['absolute'].append({'measured':measured,'truth':self.precisionTruth['xyz'],
                                                   'truthNodeID':self.precisionTruth['id'],
                                                   'matrix':stageMatrix,'time':time.time()})
            volume.SetAttribute('Aimooe.PrecisionReport',json.dumps(report))
            if len(samples)==1 or len(samples)%10==0:
                self.buildPrecisionVisualization(stage,volume,report[stage])
            status.text=self.precisionText(report[stage])+'\n采集中：%d/30 个有效帧。'%len(samples)
            if len(samples)>=30: self.finishPrecisionCollection()
        except ValueError as exc:
            # Rejected optical frames remain rejected; the timer waits for recovery.
            status.text='等待有效跟踪帧：'+str(exc)
        except Exception as exc:
            self.precisionTimer.stop();self.precisionStage=None
            status.text='精度采集中止：'+str(exc)
            raise

    def finishPrecisionCollection(self):
        stage=self.precisionStage
        if not stage:return
        self.precisionTimer.stop();self.precisionStage=None
        button=self.pointPrecisionButton if stage=='POINT' else self.surfacePrecisionButton
        button.text='开始'+('点配准' if stage=='POINT' else '面配准')+'精度采集'
        volume,_=self.ensureRegistration()
        samples=self.precisionReport(volume)[stage]
        if samples['repeatability']:
            self.showPrecisionVisualization(stage)
        else:
            status=self.pointPrecisionStatus if stage=='POINT' else self.surfacePrecisionStatus
            status.text='本次没有有效采样；请检查工具 TRACKING 状态。'

    @staticmethod
    def precisionNodeKey(kind, stage):
        return 'Aimooe.Precision%s_%s' % (kind, stage)

    def clearPrecisionVisualization(self, stage, volume=None):
        volume=volume or (self.volumeSelector.currentNode() if hasattr(self,'volumeSelector') else None)
        if volume is None: return
        for kind in ('Visualization','Landmarks','Color'):
            key=self.precisionNodeKey(kind,stage)
            node=slicer.mrmlScene.GetNodeByID(volume.GetAttribute(key) or '')
            if node: slicer.mrmlScene.RemoveNode(node)
            volume.SetAttribute(key,None)

    @staticmethod
    def errorGlyph(point, error, direction=None):
        """Create a scalar-labelled sphere or arrow in image RAS coordinates."""
        if direction is None or np.linalg.norm(direction)<1e-9:
            source=vtk.vtkSphereSource(); source.SetCenter(*point); source.SetRadius(.55)
            source.SetThetaResolution(12); source.SetPhiResolution(12); source.Update()
            poly=vtk.vtkPolyData(); poly.DeepCopy(source.GetOutput())
        else:
            vector=np.asarray(direction,float); length=float(np.linalg.norm(vector)); x=vector/length
            guide=np.array([0.,0.,1.]) if abs(x[2])<.9 else np.array([0.,1.,0.])
            z=np.cross(x,guide); z/=np.linalg.norm(z); y=np.cross(z,x)
            width=max(.12,min(.65,length*.08))
            m=vtk.vtkMatrix4x4()
            for row in range(3):
                m.SetElement(row,0,float(x[row]*length));m.SetElement(row,1,float(y[row]*width));m.SetElement(row,2,float(z[row]*width));m.SetElement(row,3,float(point[row]))
            arrow=vtk.vtkArrowSource(); arrow.SetTipResolution(16); arrow.SetShaftResolution(12)
            transformFilter=vtk.vtkTransformPolyDataFilter(); transform=vtk.vtkTransform(); transform.SetMatrix(m)
            transformFilter.SetTransform(transform); transformFilter.SetInputConnection(arrow.GetOutputPort()); transformFilter.Update()
            poly=vtk.vtkPolyData(); poly.DeepCopy(transformFilter.GetOutput())
        scalars=vtk.vtkDoubleArray(); scalars.SetName('ErrorMm'); scalars.SetNumberOfTuples(poly.GetNumberOfPoints()); scalars.Fill(float(error))
        poly.GetPointData().SetScalars(scalars)
        return poly

    def buildPrecisionVisualization(self, stage, volume, stageData):
        self.clearPrecisionVisualization(stage,volume)
        append=vtk.vtkAppendPolyData(); truthPoints=[]; errors=[]
        repeat=stageData['repeatability']
        if repeat:
            xyz=np.asarray([item['image'] for item in repeat],float); centroid=xyz.mean(axis=0)
            for point in xyz:
                error=float(np.linalg.norm(point-centroid));errors.append(error)
                append.AddInputData(self.errorGlyph(point,error))
        for item in stageData['absolute']:
            truth=np.asarray(item['truth'],float); measured=np.asarray(item['measured'],float)
            error=float(np.linalg.norm(measured-truth)); errors.append(error); truthPoints.append(truth)
            append.AddInputData(self.errorGlyph(truth,error,measured-truth))
        if not errors: raise ValueError('当前阶段尚无精度样本')
        append.Update(); poly=vtk.vtkPolyData(); poly.DeepCopy(append.GetOutput())
        model=slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode','PrecisionErrors_'+stage+'_'+volume.GetName())
        model.SetAndObservePolyData(poly); model.CreateDefaultDisplayNodes(); model.SetAttribute('Aimooe.ImageOwner',volume.GetID())
        volume.SetAttribute(self.precisionNodeKey('Visualization',stage),model.GetID())
        color=slicer.mrmlScene.AddNewNodeByClass('vtkMRMLProceduralColorNode','PrecisionHeat_'+stage+'_'+volume.GetName())
        color.SetAttribute('Aimooe.ImageOwner',volume.GetID()); maximum=max(max(errors),.001)
        function=color.GetColorTransferFunction(); function.RemoveAllPoints(); function.AddRGBPoint(0.,.05,.35,1.); function.AddRGBPoint(maximum*.5,1.,.9,.05); function.AddRGBPoint(maximum,1.,.05,.02)
        volume.SetAttribute(self.precisionNodeKey('Color',stage),color.GetID())
        display=model.GetDisplayNode(); display.SetActiveScalarName('ErrorMm'); display.SetScalarVisibility(True)
        display.SetScalarRange(0.,maximum); display.SetAndObserveColorNodeID(color.GetID()); display.SetPointSize(8); display.SetLineWidth(4)
        if truthPoints:
            marks=slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode','PrecisionTruth_'+stage+'_'+volume.GetName())
            marks.CreateDefaultDisplayNodes(); marks.SetAttribute('Aimooe.ImageOwner',volume.GetID()); marks.SetLocked(True)
            for index,point in enumerate(truthPoints): marks.AddControlPoint(vtk.vtkVector3d(*point),'Truth '+str(index+1))
            marks.GetDisplayNode().SetSelectedColor(1.,1.,1.); marks.GetDisplayNode().SetColor(1.,1.,1.)
            volume.SetAttribute(self.precisionNodeKey('Landmarks',stage),marks.GetID())
        return model, maximum

    def showPrecisionVisualization(self, stage):
        volume,_=self.ensureRegistration(); stageData=self.precisionReport(volume)[stage]
        model,maximum=self.buildPrecisionVisualization(stage,volume,stageData)
        dialog=qt.QDialog(slicer.util.mainWindow()); dialog.setWindowTitle(('点配准' if stage=='POINT' else '面配准')+'精度统计与三维误差可视化')
        layout=qt.QVBoxLayout(dialog); summary=qt.QLabel(self.precisionText(stageData)+'\n颜色范围：0.000–%.3f mm；白点为独立真值，彩色箭头指向实测位置。'%maximum)
        summary.wordWrap=True; layout.addWidget(summary)
        records=[]
        repeat=stageData['repeatability']
        if repeat:
            centroid=np.asarray([item['image'] for item in repeat],float).mean(axis=0)
            for index,item in enumerate(repeat):
                measured=np.asarray(item['image'],float);records.append(('重复',index+1,centroid,measured,float(np.linalg.norm(measured-centroid))))
        for index,item in enumerate(stageData['absolute']):
            truth=np.asarray(item['truth'],float);measured=np.asarray(item['measured'],float);records.append(('绝对',index+1,truth,measured,float(np.linalg.norm(measured-truth))))
        table=qt.QTableWidget(len(records),5); table.setHorizontalHeaderLabels(['类型','序号','参考/质心 R,A,S','实测 R,A,S','误差 mm']);table.setEditTriggers(qt.QAbstractItemView.NoEditTriggers)
        for row,(kind,index,truth,measured,error) in enumerate(records):
            values=[kind,index,', '.join(f'{v:.3f}' for v in truth),', '.join(f'{v:.3f}' for v in measured),f'{error:.3f}']
            for column,value in enumerate(values): table.setItem(row,column,qt.QTableWidgetItem(str(value)))
        table.resizeColumnsToContents();layout.addWidget(table)
        hide=qt.QPushButton('隐藏三维精度可视化');hide.connect('clicked()',lambda:model.GetDisplayNode().SetVisibility(False));layout.addWidget(hide)
        dialog.resize(900,520);dialog.show();dialog.raise_()
        if not hasattr(self,'precisionDialogs'): self.precisionDialogs=[]
        self.precisionDialogs=[item for item in self.precisionDialogs if item.visible];self.precisionDialogs.append(dialog)

    @staticmethod
    def precisionText(stageData):
        repeat=repeatability_statistics([item['image'] for item in stageData['repeatability']])
        absolute=absolute_accuracy_statistics([item['measured'] for item in stageData['absolute']],
                                               [item['truth'] for item in stageData['absolute']]) if stageData['absolute'] else None
        r=('未采样' if repeat is None else
           'n=1，样本不足（至少 2 帧）' if repeat['count']==1 else
           'n=%d，RMS %.3f mm，P95 %.3f mm%s'%(repeat['count'],repeat['rms'],repeat['p95'],
                                              '（未满 30 帧）' if repeat['count']<30 else ''))
        a='未采样' if absolute is None else 'n=%d，均值 %.3f mm，RMS %.3f mm，P95 %.3f mm'%(absolute['count'],absolute['mean'],absolute['rms'],absolute['p95'])
        return '重复定位精度（定点多帧）：'+r+'\n绝对定位精度（独立真值点）：'+a

    def refreshPrecisionMetrics(self):
        volume=self.volumeSelector.currentNode() if hasattr(self,'volumeSelector') else None
        if volume is None: return
        point=json.loads(volume.GetAttribute('Aimooe.PointReport') or 'null')
        surface=json.loads(volume.GetAttribute('Aimooe.SurfaceReport') or 'null')
        self.pointFitLabel.text=('拟合质量：未完成' if not point else '点配准 FRE：RMS %.3f mm，P95 %.3f mm（非绝对精度）'%(point['rms'],point['p95']))
        self.surfaceFitLabel.text=('拟合质量：未应用' if not surface else '表面 ICP：RMS %.3f → %.3f mm，P95 %.3f mm（非绝对精度）'%(surface['before']['rms'],surface['after']['rms'],surface['after']['p95']))

    def exportRegistrationReport(self):
        volume = self.requireVolume()
        report = dict(volume=volume.GetName(), context=self.contextKey(), time=time.time(),
                      simulated=self.logic.simulation, tipSource=self.logic.nodes['TipToTool'].GetAttribute('Aimooe.TipCalibration'))
        for name in ('PointReport', 'SurfaceReport', 'ValidationReport', 'PrecisionReport',
                     'SavedRegistration'):
            report[name] = json.loads(volume.GetAttribute('Aimooe.'+name) or 'null')
        path = ROOT/'Logs'/('registration-report-'+time.strftime('%Y%m%d-%H%M%S')+'.json')
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        self.guidanceStatus.text = '报告已保存：'+str(path)

    def nativeSegmentation(self):
        volume = self.requireVolume()
        node = slicer.mrmlScene.GetNodeByID(volume.GetAttribute('Aimooe.NativeSegmentation') or '')
        if node is None:
            node = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLSegmentationNode', volume.GetName()+'_Threshold')
            node.CreateDefaultDisplayNodes()
            node.SetReferenceImageGeometryParameterFromVolumeNode(volume)
            node.SetAndObserveTransformNodeID(volume.GetTransformNodeID())
            node.SetAttribute('Aimooe.SourceFile', 'native-threshold')
            volume.SetAttribute('Aimooe.NativeSegmentation', node.GetID())
        return node

    def nativeThreshold(self):
        volume = self.requireVolume()
        array = slicer.util.arrayFromVolume(volume)
        lower, upper, selected = effective_threshold(array, self.lowerThreshold.value, self.upperThreshold.value)
        node = self.nativeSegmentation()
        segmentation = node.GetSegmentation()
        segment = segmentation.GetNthSegmentID(0) if segmentation.GetNumberOfSegments() else segmentation.AddEmptySegment('Target', '配准目标表面')
        editor = slicer.qMRMLSegmentEditorWidget()
        params = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLSegmentEditorNode')
        try:
            editor.setMRMLScene(slicer.mrmlScene)
            editor.setMRMLSegmentEditorNode(params)
            editor.setSegmentationNode(node)
            editor.setSourceVolumeNode(volume)
            editor.setCurrentSegmentID(segment)
            editor.setActiveEffectByName('Threshold')
            effect = editor.activeEffect()
            # Batch parameter changes: the effect UI must not feed intermediate
            # out-of-range slider values back into the parameter node.
            token = params.StartModify()
            effect.setParameter('MinimumThreshold', lower)
            effect.setParameter('MaximumThreshold', upper)
            params.EndModify(token)
            if abs(effect.doubleParameter('MinimumThreshold')-lower)>1e-8 or abs(effect.doubleParameter('MaximumThreshold')-upper)>1e-8:
                raise RuntimeError('分割阈值被编辑器改变，已停止重建')
            node.GetDisplayNode().SetVisibility(True)
            effect.self().onApply()
            actual = slicer.util.arrayFromSegmentBinaryLabelmap(node, segment, volume)
            expected = (array >= lower) & (array <= upper)
            if actual.shape != expected.shape or not np.array_equal(actual != 0, expected):
                raise RuntimeError('原生分割结果与阈值体素不一致，未更新三维模型')
            node.CreateClosedSurfaceRepresentation()
            mesh = vtk.vtkPolyData()
            node.GetClosedSurfaceRepresentation(segment, mesh)
            if not mesh.GetNumberOfPoints():
                raise ValueError('阈值范围没有生成表面')
            model = slicer.mrmlScene.GetNodeByID(volume.GetAttribute('Aimooe.SurfaceID') or '')
            if model is None:
                model = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', volume.GetName()+'_3D')
                model.CreateDefaultDisplayNodes()
                volume.SetAttribute('Aimooe.SurfaceID', model.GetID())
            model.SetAndObservePolyData(mesh)
            model.GetDisplayNode().SetVisibility2D(False)
            model.SetAndObserveTransformNodeID(node.GetTransformNodeID())
            model.SetAttribute('Aimooe.SourceVolume', volume.GetID())
            model.GetDisplayNode().SetColor(.92, .85, .72)
            node.GetDisplayNode().SetVisibility(False)
            volume.SetAttribute('Aimooe.RenderMode', 'surface')
            self.volumeLayout()
            self.surfaceSelector.setCurrentNode(model)
            volume.SetAttribute('Aimooe.EffectiveThreshold', json.dumps([lower, upper]))
            self.reconstructionStatus.text = '实际阈值 %g–%g；选中 %d / %d 体素（%.2f%%），已逐体素核对。' % (lower, upper, selected, array.size, 100.*selected/array.size)
            self.logic.applyWorkspaceVisibility()
            slicer.util.resetThreeDViews()
        finally:
            node.GetDisplayNode().SetVisibility(False)
            editor.setActiveEffectByName('')
            editor.setMRMLScene(None)
            editor.deleteLater()
            slicer.mrmlScene.RemoveNode(params)

    def openNativeEditor(self):
        node = self.nativeSegmentation()
        slicer.util.selectModule('SegmentEditor')
        editor = slicer.modules.segmenteditor.widgetRepresentation().self().editor
        editor.setSegmentationNode(node)
        editor.setSourceVolumeNode(self.requireVolume())
        editor.setActiveEffectByName('Threshold')

    def cleanupStudio(self):
        if hasattr(self, 'surfaceTimer'):
            self.stopSurfaceCollection()
            self.cancelSurface()
            self.endPlacement()
        super().cleanupStudio()

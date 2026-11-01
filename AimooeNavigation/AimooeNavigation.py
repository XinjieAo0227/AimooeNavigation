import json
import os
import logging
import time
from collections import deque
from pathlib import Path
import vtk
import qt
import ctk
import slicer
from slicer.ScriptedLoadableModule import (
    ScriptedLoadableModule, ScriptedLoadableModuleWidget,
    ScriptedLoadableModuleLogic)
from slicer.util import VTKObservationMixin
from AimooeLib.AimooeTransforms import simulation_pose, relative_matrix
from AimooeLib.AimooeCoordinates import coordinate_snapshot, transform_point
from AimooeLib.AimooeAssets import AssetMixin
from AimooeLib.AimooeLibrary import LibraryMixin
from AimooeLib.AimooePairs import PairLogicMixin, PairWidgetMixin
from AimooeLib.AimooeModels import load_profile, matrix_from_rows, read_mesh, sphere

from AimooeLib.AimooeNext import NextLogicMixin, NextWidgetMixin, tr

from AimooeLib.AimooeContinuity import ContinuityWidgetMixin
from AimooeLib.AimooeGuidance import GuidanceLogicMixin, GuidanceWidgetMixin
from AimooeLib.AimooeCalibration import CalibrationWidgetMixin
from AimooeLib.AimooeWorkspace import WorkspaceLogicMixin, WorkspaceWidgetMixin
from AimooeLib.AimooeStudio import StudioLogicMixin, StudioWidgetMixin

from AimooeLib.constants import STALE_SECONDS, LOST_SECONDS, UPDATE_INTERVAL_MS
from AimooeLib.AimooePaths import project_path, stored_path

ROOT = Path(__file__).resolve().parents[1]


def project_logger():
    logger = logging.getLogger('AimooeNavigation')
    if not logger.handlers:
        (ROOT / 'Logs').mkdir(exist_ok=True)
        handler = logging.FileHandler(ROOT / 'Logs' / 'navigation.log', encoding='utf-8')
        handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
    return logger


class AimooeNavigation(ScriptedLoadableModule):
    def __init__(self, parent):
        super().__init__(parent)
        parent.title = 'Aimooe Navigation'
        parent.categories = ['IGT']
        parent.contributors = ['AimooeNavigation project']
        parent.helpText = 'LaunchOptimized082 / v0.1.0 candidate：稳定实时渲染、精度三维可视化、可配置 Live coordinates。请阅读使用说明。'
        parent.acknowledgementText = 'AimPosition SDK'


class AimooeNavigationLogic(GuidanceLogicMixin, WorkspaceLogicMixin, StudioLogicMixin, NextLogicMixin, PairLogicMixin, LibraryMixin, AssetMixin, ScriptedLoadableModuleLogic):
    def __init__(self):
        super().__init__()
        self.log = project_logger()
        self.nodes = {}
        self.tracker = None
        self.simulation = True
        self.running = False
        self.last = {'Tool': None, 'Reference': None, 'CalibrationTool': None}
        self.valid = {'Tool': False, 'Reference': False, 'CalibrationTool': False}
        self.frames = deque(maxlen=120)
        self.error = ''
        self.cameraFitPending = False
        self.referenceView = False
        self.referenceEnabled = True
        self.singleToolModeActive = False
        self.assetBindings = []
        self.previewOnly = False
        self.pairToolProfile = None
        self.cadProfile = None
        self.modelParentName = 'CadToTool'
        self.showMarkers = True
        self.measuredPoints = []
        self.displayModelNames = []
        self.autoKeepInView = False
        self.updateDurations = deque(maxlen=240)
        # Navigation validity remains frame-strict. This short hold is display
        # only and prevents a single rejected SDK frame from blinking geometry.
        self.visualHoldSeconds = 0.25
        self.timer = qt.QTimer()
        self.timer.setInterval(UPDATE_INTERVAL_MS)
        self.timer.connect('timeout()', self.update)

    def node(self, name, class_name='vtkMRMLLinearTransformNode'):
        node = self.nodes.get(name)
        if node is None or node.GetScene() is not slicer.mrmlScene:
            matches = slicer.util.getNodesByClass(class_name)
            node = next((n for n in matches if n.GetName() == name), None)
            if node is None:
                node = slicer.mrmlScene.AddNewNodeByClass(class_name, name)
            self.nodes[name] = node
        return node

    def createTransforms(self):
        for name in ['ToolToTracker', 'ReferenceToTracker', 'CalibrationToolToTracker', 'TipToTool', 'ToolToReference']:
            self.node(name)
        self.nodes['TipToTool'].SetAndObserveTransformNodeID(self.nodes['ToolToTracker'].GetID())
        if not self.nodes['TipToTool'].GetAttribute('Aimooe.TipCalibration'):
            self.nodes['TipToTool'].SetAttribute('Aimooe.TipCalibration', 'UNCALIBRATED_MARKER_ORIGIN')
        display = self.node('TrackerToDisplay')
        self.nodes['ToolToTracker'].SetAndObserveTransformNodeID(display.GetID())
        self.nodes['ReferenceToTracker'].SetAndObserveTransformNodeID(display.GetID())
        self.nodes['CalibrationToolToTracker'].SetAndObserveTransformNodeID(display.GetID())
        self.statusNode = self.node('AimooeTrackingStatus', 'vtkMRMLScriptedModuleNode')
        self.publish()

    def createNeedle(self):
        self.createTransforms()
        folder = ROOT / 'AimooeNavigation' / 'Resources' / 'Models'
        if not (folder / 'LTS-B4E1.json').is_file():
            if not self.simulation:
                raise FileNotFoundError('Licensed CAD profile is not installed; import your own model.')
            self.pairToolProfile = None
            self.cadProfile = None
            self.nodes['TipToTool'].SetMatrixTransformToParent(vtk.vtkMatrix4x4())
            self.nodes['TipToTool'].SetAttribute('Aimooe.TipCalibration', 'SYNTHETIC_UNCALIBRATED')
            self.displayModelNames = ['SyntheticTool']
            self.setModel('SyntheticTool', sphere([0, 0, 0], 3),
                          self.nodes['ToolToTracker'], [0.2, 0.6, 0.9])
            return
        profile = load_profile(folder)
        if self.tracker and self.tracker.selected:
            if self.tracker.selected['Tool'] != profile['toolName']:
                raise ValueError('This CAD profile matches ' + profile['toolName'] +
                                 '; select that tool before displaying this model')
        self.pairToolProfile = None
        self.cadProfile = profile
        cad = self.node('CadToTool')
        cad.SetAndObserveTransformNodeID(self.nodes['ToolToTracker'].GetID())
        cad.SetMatrixTransformToParent(matrix_from_rows(profile['cadToTool']))
        cad.SetAttribute('Aimooe.Alignment', 'CAD_MARKER_GEOMETRY_ONLY')
        tip = self.nodes['TipToTool']
        if tip.GetAttribute('Aimooe.TipCalibration') != 'CALIBRATED':
            tip.SetMatrixTransformToParent(matrix_from_rows(profile['tipToTool']))
            tip.SetAttribute('Aimooe.TipCalibration', profile['tipSource'])
        self.displayModelNames = []
        for part in profile['parts']:
            self.setModel(part['name'], read_mesh(folder / part['file']),
                          cad, part['color'])
        # TipToTool origin is the CAD tip sphere distal pole and -Z is the needle
        # axis, so the ball centre sits 1 mm along -Z: radius 1 mm reproduces the
        # original CAD tip ball in red instead of overhanging the tip by 1.5 mm.
        self.setModel('NeedleTip', sphere([0, 0, -1.0], 1.0), tip, [1.0, 0.0, 0.0])
        tracker = self.nodes['ToolToTracker']
        for i, point in enumerate(profile['markersTool']):
            self.setModel('OpticalMarker' + str(i+1),
                          sphere(point, profile['markerRadiusMm']),
                          tracker, [0.86, 0.88, 0.93])
        labels = self.node('OpticalMarkerLabels', 'vtkMRMLMarkupsFiducialNode')
        labels.RemoveAllControlPoints()
        labels.SetAndObserveTransformNodeID(tracker.GetID())
        labels.CreateDefaultDisplayNodes()
        for i, point in enumerate(profile['markersTool']):
            labels.AddControlPoint(vtk.vtkVector3d(*point), 'M' + str(i+1))
        labels.SetLocked(True)
        display = labels.GetDisplayNode()
        display.SetUseGlyphScale(False)
        display.SetGlyphSize(1.0)
        display.SetTextScale(2.0)
        display.SetPointLabelsVisibility(True)
        display.SetPropertiesLabelVisibility(False)
        display.SetSelectedColor(1.0, 0.85, 0.2)
        measured = self.node('MeasuredOpticalMarkers', 'vtkMRMLMarkupsFiducialNode')
        measured.RemoveAllControlPoints()
        measured.SetAndObserveTransformNodeID(self.nodes['TrackerToDisplay'].GetID())
        measured.CreateDefaultDisplayNodes()
        for i in range(4):
            measured.AddControlPoint(vtk.vtkVector3d(0, 0, 0), 'Measured M' + str(i+1))
        measured.SetLocked(True)
        display = measured.GetDisplayNode()
        display.SetUseGlyphScale(False)
        display.SetGlyphSize(2.0)
        display.SetSelectedColor(0.05, 1.0, 0.9)
        display.SetPointLabelsVisibility(False)
        display.SetPropertiesLabelVisibility(False)
        self.measuredPoints = []
        self.publish()
        if slicer.app.layoutManager():
            slicer.util.resetThreeDViews()

    def fitToolView(self):
        manager = slicer.app.layoutManager()
        if not manager:
            return
        if not self.cadProfile:
            self.fitPairView()
            return
        matrix = vtk.vtkMatrix4x4()
        self.nodes['ToolToTracker'].GetMatrixTransformToWorld(matrix)
        tip = self.cadProfile['tipToTool']
        center = [tip[i][3] * 0.5 for i in range(3)]
        focal = transform_point(matrix, center)
        position = transform_point(matrix, [center[0], center[1]-120, center[2]-800])
        up = [matrix.GetElement(i, 0) for i in range(3)]
        for index in range(manager.threeDViewCount):
            view = manager.threeDWidget(index).threeDView()
            camera = slicer.modules.cameras.logic().GetViewActiveCameraNode(view.mrmlViewNode())
            view.mrmlViewNode().SetBoxVisible(False)
            view.mrmlViewNode().SetAxisLabelsVisible(False)
            camera.SetFocalPoint(*focal)
            camera.SetPosition(*position)
            camera.SetViewUp(*up)
            camera.GetCamera().SetParallelScale(175)
            camera.ResetClippingRange()

    def setModel(self, name, polydata, parent, color):
        model = self.node(name, 'vtkMRMLModelNode')
        model.SetAndObservePolyData(polydata)
        model.CreateDefaultDisplayNodes()
        model.GetDisplayNode().SetColor(*color)
        model.GetDisplayNode().SetSpecular(0.3)
        model.SetAndObserveTransformNodeID(parent.GetID())
        model.SetAttribute('Aimooe.Geometry', 'CAD_NOMINAL_NOT_CALIBRATION')
        self.displayModelNames.append(name)

    def updateMeasuredMarkers(self):
        points = None
        pointProfile = self.cadProfile or self.pairToolProfile
        if pointProfile and self.state('Tool') == 'TRACKING':
            if self.simulation:
                matrix = vtk.vtkMatrix4x4()
                self.nodes['ToolToTracker'].GetMatrixTransformToParent(matrix)
                points = [transform_point(matrix, p) for p in pointProfile['markersTool']]
            elif self.tracker:
                points = list(self.tracker.measured_markers)
        if points is not None and len(points) == 4 and all(p is not None for p in points):
            self.measuredPoints = points
        elif not self.visualTracking('Tool'):
            self.measuredPoints = []
        node = self.nodes.get('MeasuredOpticalMarkers')
        if node and len(self.measuredPoints) == 4:
            token = node.StartModify()
            for i, point in enumerate(self.measuredPoints):
                if point is not None:
                    node.SetNthControlPointPosition(i, *point)
            node.EndModify(token)

    def captureCoordinates(self):
        # A coordinate read must never publish/re-enter updateImageTip.
        required = ('TipToTool', 'ToolToTracker', 'ReferenceToTracker')
        if any(name not in self.nodes or self.nodes[name].GetScene() is None for name in required):
            raise ValueError('请先初始化或连接定位工具')
        matrices = {}
        for name in ['TipToTool', 'ToolToTracker', 'ReferenceToTracker']:
            matrices[name] = vtk.vtkMatrix4x4()
            self.nodes[name].GetMatrixTransformToParent(matrices[name])
        registration = next((n for n in slicer.util.getNodesByClass('vtkMRMLLinearTransformNode')
                             if n.GetName() == 'ReferenceToImage'), None)
        image_matrix = None
        image_valid = bool(registration and
                           registration.GetAttribute('Aimooe.RegistrationValidated') == 'True')
        if image_valid:
            image_matrix = vtk.vtkMatrix4x4()
            registration.GetMatrixTransformToParent(image_matrix)
        result = coordinate_snapshot(
            matrices['TipToTool'], matrices['ToolToTracker'], matrices['ReferenceToTracker'],
            self.state('Tool') == 'TRACKING', self.referenceEnabled and self.state('Reference') == 'TRACKING',
            image_matrix, image_valid,
            self.nodes['TipToTool'].GetAttribute('Aimooe.TipCalibration'),
            self.tracker.last_frame if self.tracker else 'SIMULATION')
        result['experimentalPartialTracking'] = bool(self.tracker and getattr(self.tracker, 'minimumMatchPts', 0) == 3)
        result['simulation'] = self.simulation
        result['markerSource'] = 'SIMULATED' if self.simulation else 'SDK_MEASURED'
        result['markers'] = []
        pointProfile = self.cadProfile or self.pairToolProfile
        if pointProfile:
            result['cadAlignmentRmsMm'] = pointProfile['rmsMm']
            for i, point in enumerate(pointProfile['markersTool']):
                valid = self.state('Tool') == 'TRACKING'
                measured = self.measuredPoints[i] if valid and len(self.measuredPoints) == 4 else None
                result['markers'].append({'name': 'M' + str(i+1), 'tool': point,
                                          'trackerExpected': transform_point(matrices['ToolToTracker'], point) if valid else None,
                                          'trackerMeasured': measured})
        return result

    def startSimulation(self):
        self.disconnect()
        self.simulation = True
        self.referenceEnabled = True
        self.referenceView = False
        self.createNeedle()
        self.running = True
        self.error = ""
        self.cameraFitPending = True
        self.started = time.monotonic()
        self.frames.clear()
        self.timer.start()
        self.log.info('Tracking start: simulation')

    def connectDevice(self, interface, path, localIP='', deviceIP=''):
        self.disconnect()
        from AimooeLib.AimooeStream import AimooeStreamTracker
        self.simulation = False
        self.tracker = AimooeStreamTracker(Path(os.environ.get('AIMOOE_SDK_RUNTIME', str(ROOT / 'AimooeSDK' / 'Runtime'))), self.log, Path(slicer.app.slicerHome)/'bin/PythonSlicer.exe', ROOT/'Logs/Tracking')
        try:
            self.tracker.initializeSdk()
            self.tracker.connectDevice(interface,localIP,deviceIP)
            names = self.tracker.loadToolFiles(path)
            self.error = ""
            self.createTransforms()
            return names
        except Exception:
            self.disconnect()
            raise

    def startHardware(self, tool, reference=None, calibrationTool=None):
        if self.running:
            raise ValueError('请先停止跟踪再更换或重新启动工具')
        if not self.tracker:
            raise RuntimeError('Connect tracker first')
        self.tracker.selectTools(tool, reference, calibrationTool)
        self.singleToolModeActive = reference is None
        self.referenceEnabled = reference is not None
        self.createTransforms()
        tipNode=self.nodes['TipToTool']
        if (tipNode.GetAttribute('Aimooe.TipCalibration')=='CALIBRATED' and
                tipNode.GetAttribute('Aimooe.CalibrationTool')!=tool):
            tipNode.SetAttribute('Aimooe.TipCalibration','UNCALIBRATED_TOOL_CHANGED')
            for key in ('Aimooe.CalibrationMethod','Aimooe.CalibrationFile','Aimooe.CalibrationRMSmm','Aimooe.CalibrationTool'):
                tipNode.SetAttribute(key,None)
        folder = ROOT / 'AimooeNavigation' / 'Resources' / 'Models'
        profile = load_profile(folder) if (folder / 'LTS-B4E1.json').is_file() else None
        if profile and tool == profile['toolName']:
            self.createNeedle()
        else:
            self.createTransforms()
            for name in self.displayModelNames + ['OpticalMarkerLabels', 'MeasuredOpticalMarkers']:
                if name in self.nodes:
                    self.nodes[name].GetDisplayNode().SetVisibility(False)
            self.displayModelNames = []
            self.pairToolProfile = None
            self.cadProfile = None
            self.nodes['TipToTool'].SetMatrixTransformToParent(vtk.vtkMatrix4x4())
            self.nodes['TipToTool'].SetAttribute('Aimooe.TipCalibration', 'UNCALIBRATED_MARKER_ORIGIN')
        if reference:
            self.loadSelectedPairs(tool, reference)
        else:
            from AimooeLib.AimooePairs import profiles, read_pair
            matches=[path for path in profiles() if read_pair(path)['toolName']==tool]
            if len(matches)==1:
                self.loadPair(matches[0],'Tool')
        self.tracker.startTracking()
        self.error = ""
        self.cameraFitPending = True
        self.running = True
        self.frames.clear()
        self.timer.start()

    def stop(self):
        self.timer.stop()
        self.running = False
        self.valid = dict.fromkeys(self.valid, False)
        self.last = dict.fromkeys(self.last, None)
        self.frames.clear()
        if self.tracker:
            try:
                self.tracker.stopTracking()
            except Exception as exc:
                self.error = str(exc)
                self.log.exception('Tracking stop failed')
        self.publish()
        self.log.info('Tracking stop')

    def disconnect(self):
        self.stop()
        if self.tracker:
            try:
                self.tracker.disconnectDevice()
            except Exception as exc:
                self.error = str(exc)
                self.log.exception('Disconnect failed')
            finally:
                self.tracker = None
        self.singleToolModeActive = False
        self.publish()

    def state(self, role):
        if not self.running or self.last[role] is None:
            return 'LOST'
        age = time.monotonic() - self.last[role]
        if age >= LOST_SECONDS or not self.valid[role]:
            return 'LOST'
        return 'TRACKING' if age < STALE_SECONDS else 'STALE'

    def visualTracking(self, role):
        """Display-only continuity; never used for registration or navigation."""
        if self.state(role) == 'TRACKING':
            return True
        updated = self.last.get(role)
        return bool(self.running and updated is not None and
                    0 <= time.monotonic()-updated < self.visualHoldSeconds)

    def ready(self):
        experimental = self.tracker and getattr(self.tracker, 'minimumMatchPts', 0) == 3
        return not experimental and self.referenceEnabled and all(
            self.state(role) == 'TRACKING' for role in ('Tool','Reference'))

    def detailedState(self, role):
        if self.state(role)=='TRACKING':
            return 'VALID'
        tracker=self.tracker
        if (tracker and role in getattr(tracker,'previewUpdated',{}) and
                time.monotonic()-tracker.previewUpdated[role] < STALE_SECONDS):
            return 'DEGRADED'
        updated=self.last.get(role)
        if self.running and updated is not None and time.monotonic()-updated < LOST_SECONDS:
            return 'TEMP_LOST'
        return 'LOST'

    def rate(self):
        if len(self.frames) < 2 or time.monotonic() - self.frames[-1] >= STALE_SECONDS:
            return 0.0
        return (len(self.frames)-1)/(self.frames[-1]-self.frames[0])

    def publish(self):
        node = self.nodes.get('AimooeTrackingStatus')
        if node is None or node.GetScene() is None:
            return
        token = node.StartModify()
        for role in self.last:
            node.SetParameter(role, self.state(role))
            node.SetParameter(role+'Detail',self.detailedState(role))
            reason=getattr(self.tracker,'role_reason',{}).get(role,'') if self.tracker else ''
            node.SetParameter(role+'Reason',reason)
            transform = self.nodes.get(role + 'ToTracker')
            if transform:
                transform.SetAttribute('Aimooe.Valid', str(self.state(role) == 'TRACKING'))
                transform.SetAttribute('Aimooe.LastUpdateTime', str(self.last[role]))
        node.SetParameter('Navigation', 'READY' if self.ready() else 'NOT READY')
        node.SetParameter('Rate', (f'采集 {(self.tracker.acquisitionHz if time.monotonic()-self.tracker.receivedTime < .5 else 0):.1f} / 更新 {self.rate():.1f} Hz' if self.tracker and hasattr(self.tracker, 'acquisitionHz') else f'{self.rate():.1f} Hz'))
        node.SetParameter('Tracker', 'Simulation' if self.simulation else
                          ('Connected' if self.tracker and self.tracker.connected else 'Disconnected'))
        self.updateDisplayFrame()
        self.updateAssetVisibility()
        node.SetParameter('ReferenceFrame', 'REFERENCE' if self.referenceView else 'TRACKER')
        quality = self.tracker.quality_message if self.tracker else ''
        experimental = self.tracker and getattr(self.tracker, 'minimumMatchPts', 0) == 3
        hasPreview = self.tracker and bool(getattr(self.tracker, 'previewPoses', {}))
        prefix = '实时诊断／告警：' if hasPreview else '无有效跟踪（设备仍连接）：'
        node.SetParameter('Quality', (prefix + quality) if quality else ('三球实验预览 / NOT READY' if experimental else '正常 / 等待有效工具'))
        node.SetParameter('DisplayStatus', '实时诊断显示（质量告警）' if hasPreview else ('有效姿态显示' if self.state('Tool')=='TRACKING' else '未取得可显示姿态'))
        node.SetParameter('RejectedFrames', str(self.tracker.rejected_frames) if self.tracker else '0')
        node.SetParameter('Diagnostics',json.dumps(getattr(self.tracker,'diagnostics',{}),ensure_ascii=False) if self.tracker else '{}')
        node.SetParameter('Error', self.error or (self.tracker.quality_message if self.tracker else ''))
        node.SetParameter('ViewMode', 'SINGLE NEEDLE / TRACKER FIXED' if self.singleToolModeActive else ('REFERENCE VIEW - NOT REGISTERED' if self.referenceView else ('TRACKER PREVIEW - NOT REGISTERED' if self.previewOnly else 'DUAL TOOL TRACKING')))
        node.SetParameter('Markers', str(self.tracker.marker_count) if self.tracker else '--')
        node.SetParameter('Frame', str(self.tracker.last_frame) if self.tracker else '--')
        node.SetParameter('Coordinates', self.nodes['TipToTool'].GetAttribute('Aimooe.TipCalibration'))
        node.SetParameter('Workflow', self.workflowState() if hasattr(self,'workflowState') else 'UNKNOWN')
        self.nodes['ToolToReference'].SetAttribute('Aimooe.Valid', str(self.ready()))
        for name in self.displayModelNames:
            if name in self.nodes:
                self.nodes[name].GetDisplayNode().SetVisibility(self.canDisplayTool())
        visible = self.canDisplayTool()
        for i in range(4):
            marker = self.nodes.get('OpticalMarker' + str(i+1))
            if marker:
                marker.GetDisplayNode().SetVisibility(visible and self.showMarkers and self.cadProfile is not None)
        labels = self.nodes.get('OpticalMarkerLabels')
        if labels:
            labels.GetDisplayNode().SetVisibility(visible and self.showMarkers and self.cadProfile is not None)
        measured = self.nodes.get('MeasuredOpticalMarkers')
        if measured:
            measured.GetDisplayNode().SetVisibility(
                self.showMarkers and self.canDisplayTool() and self.visualTracking('Tool')
                and len(self.measuredPoints) == 4 and all(p is not None for p in self.measuredPoints))
        if getattr(self, 'imageView', False) and not getattr(self, 'imageNavigation', False):
            for name in self.displayModelNames + ['OpticalMarkerLabels','MeasuredOpticalMarkers']:
                if name in self.nodes:
                    self.nodes[name].GetDisplayNode().SetVisibility(False)
        node.EndModify(token)

    def update(self):
        if not self.running:
            return
        updateStarted = time.perf_counter()
        slicer.app.pauseRender()
        try:
            now = time.monotonic()
            poses = None
            if self.simulation:
                poses = {'Tool': simulation_pose(now-self.started), 'Reference': vtk.vtkMatrix4x4()}
            else:
                poses = self.tracker.updateTracking()
            if poses is not None:
                self.error = ''
                self.frames.append(now)
                for role in self.last:
                    pose = poses.get(role)
                    self.valid[role] = pose is not None
                    if pose is not None:
                        self.last[role] = now if self.simulation else self.tracker.lastUpdateTime.get(role)
                        node=self.nodes[role+'ToTracker']
                        previous=vtk.vtkMatrix4x4()
                        node.GetMatrixTransformToParent(previous)
                        if any(previous.GetElement(i,j)!=pose.GetElement(i,j) for i in range(4) for j in range(4)):
                            node.SetMatrixTransformToParent(pose)
            if poses is not None:
                self.applySdkTip()
                self.updateMeasuredMarkers()
            if self.ready():
                tool, ref = vtk.vtkMatrix4x4(), vtk.vtkMatrix4x4()
                self.nodes['ToolToTracker'].GetMatrixTransformToParent(tool)
                self.nodes['ReferenceToTracker'].GetMatrixTransformToParent(ref)
                self.nodes['ToolToReference'].SetMatrixTransformToParent(relative_matrix(tool, ref))
            self.publish()
            if (self.cameraFitPending and slicer.app.layoutManager()
                    and (self.ready() or (self.previewOnly and self.state('Tool') == 'TRACKING')
                         or (self.tracker and getattr(self.tracker, 'previewPoses', {})))):
                self.cameraFitPending = False
                self.fitToolView()
            if poses is not None:
                self.updateCameraClipping()
        except ValueError as exc:
            self.error = str(exc)
            self.valid = dict.fromkeys(self.valid, False)
            self.measuredPoints = []
            self.publish()
            self.log.error('Invalid pose frame rejected, awaiting next frame: %s', exc)
        except Exception as exc:
            self.error = str(exc)
            self.log.exception('Tracking update failed')
            self.disconnect()
        finally:
            slicer.app.resumeRender()
            self.updateDurations.append(time.perf_counter() - updateStarted)


class AimooeNavigationWidget(CalibrationWidgetMixin, ContinuityWidgetMixin, GuidanceWidgetMixin, WorkspaceWidgetMixin, StudioWidgetMixin, NextWidgetMixin, PairWidgetMixin, ScriptedLoadableModuleWidget, VTKObservationMixin):
    def __init__(self, parent=None):
        VTKObservationMixin.__init__(self)
        ScriptedLoadableModuleWidget.__init__(self, parent)

    def setup(self):
        super().setup()
        self.logic = AimooeNavigationLogic()
        slicer.app.connect("aboutToQuit()", self.logic.disconnect)
        self.layout.addWidget(qt.QLabel('AIMOOE NAVIGATION'))
        self.labels = {}
        self.coordinatesDialog = None
        self.uiRefreshCount = 0
        self.uiTimer = qt.QTimer()
        self.uiTimer.setSingleShot(True)
        self.uiTimer.setInterval(100)
        self.uiTimer.connect('timeout()', self.refresh)
        self.settings = {}
        settingsPath = ROOT / 'Config' / 'tracking.json'
        if settingsPath.exists():
            try:
                self.settings = json.loads(settingsPath.read_text(encoding='utf-8-sig'))
            except (ValueError, OSError):
                self.logic.log.exception('Could not read tracking configuration')
        optical = self.section('Optical Tracking')
        self.sim = qt.QCheckBox('Simulation Mode')
        self.sim.checked = True
        self.sim.connect("toggled(bool)", self.modeChanged)
        optical.addRow(self.sim)
        self.interface = qt.QComboBox()
        self.interface.addItems(['Ethernet', 'USB'])
        self.interface.setCurrentText(self.settings.get('interface', 'Ethernet'))
        optical.addRow('Tracking Interface', self.interface)
        self.localIP = qt.QLineEdit(self.settings.get('localIP',''))
        self.deviceIP = qt.QLineEdit(self.settings.get('deviceIP',''))
        self.localIP.setToolTip('Wi-Fi 下用于核对本机网络配置；SDK 不提供绑定本机 IP 的接口。')
        self.deviceIP.setToolTip('以太网连接前传给 Aim_SetEthernetConnectIP；Wi-Fi SDK 使用定位仪热点。')
        optical.addRow('本机 IP（网络核对）',self.localIP)
        optical.addRow('定位仪 IP',self.deviceIP)
        self.button(optical, '连接设备（暂不采集）', self.connectHardware)
        self.button(optical, '开始实时跟踪', self.startHardware)
        self.button(optical, '所有工具实时坐标 / Live coordinates', self.showCoordinates)
        self.button(optical, 'Disconnect', self.logic.disconnect)
        self.configuration = self.section('Tool Configuration')
        toolPath = project_path(self.settings.get('toolPath', 'AimooeSDK/AimTools'))
        self.path = qt.QLineEdit(str(toolPath))
        self.configuration.addRow('Tool Path', self.path)
        self.reference = qt.QComboBox()
        self.tool = qt.QComboBox()
        # Distinct synthetic roles keep SDK-free registration usable without vendor definitions.
        self.tool.addItem('SyntheticTool')
        self.reference.addItem('SyntheticReference')
        self.calibrationTool = qt.QComboBox()
        self.configuration.addRow('Reference Tool', self.reference)
        self.configuration.addRow('Navigation Tool', self.tool)
        self.configuration.addRow('Calibration Tool（可选）', self.calibrationTool)
        self.button(self.configuration, '设定参考坐标系（使用所选 Reference Tool）', self.setReferenceFrame)
        self.button(self.configuration, '取消参考坐标系（恢复 Tracker）', self.clearReferenceFrame)
        status = self.section('Tracking Status')
        self.readyLabel = qt.QLabel('NOT READY')
        status.addRow('Navigation Status', self.readyLabel)
        self.rawRecord = qt.QCheckBox('诊断时记录每帧原始 JSONL（默认关闭）')
        status.addRow(self.rawRecord)
        for name in ['Tracker', 'Reference', 'Tool', 'CalibrationTool', 'Rate', 'Markers', 'Frame', 'ViewMode', 'ReferenceFrame', 'Quality', 'DisplayStatus', 'RejectedFrames', 'X', 'Y', 'Z', 'Error']:
            self.labels[name] = qt.QLabel('--')
            self.labels[name].wordWrap = name in ('Quality', 'Error')
            self.labels[name].setFixedHeight(42 if name in ('Quality', 'Error') else 20)
            self.labels[name].setSizePolicy(qt.QSizePolicy.Ignored, qt.QSizePolicy.Fixed)
            status.addRow(tr(name), self.labels[name])
        modelForm = self.section('长针模型与针尖坐标')
        self.button(modelForm, '显示长针模型及 4 个标记球', self.logic.createNeedle)
        self.markerCheck = qt.QCheckBox('显示标记球 M1–M4（青色点为实测球心）')
        self.markerCheck.checked = True
        self.markerCheck.connect('toggled(bool)', self.setMarkersVisible)
        modelForm.addRow(self.markerCheck)
        tipNote = qt.QLabel('CAD 名义针尖：未实物标定。球心匹配 RMS 不等于针尖标定误差。')
        tipNote.wordWrap = True
        modelForm.addRow(tipNote)
        self.button(modelForm, '居中查看器械', self.logic.fitToolView)
        self.keepInView = qt.QCheckBox('自动保持跟踪模型在视野内（可关闭）')
        self.keepInView.connect('toggled(bool)', lambda value: setattr(self.logic, 'autoKeepInView', bool(value)))
        modelForm.addRow(self.keepInView)
        self.setupPairs()
        assets = self.section('其他工具模型与 CT / MRI')
        self.assetRole = qt.QComboBox()
        self.assetRole.addItems(['Tool', 'Reference', 'CalibrationTool'])
        assets.addRow('模型绑定对象', self.assetRole)
        self.button(assets, '导入工具网格（STL / OBJ / PLY / VTK）', self.importToolMesh)
        self.assetSelector = slicer.qMRMLNodeComboBox()
        self.assetSelector.nodeTypes = ['vtkMRMLModelNode']
        self.assetSelector.setMRMLScene(slicer.mrmlScene)
        assets.addRow('待绑定模型', self.assetSelector)
        self.button(assets, '加载 ModelToTool / ModelToReference 矩阵并绑定', self.bindToolMesh)
        self.button(assets, '导入 CT / MRI（NRRD / NIfTI / MHA）', self.importVolume)
        self.button(assets, '导入分割（SEG.NRRD）', self.importSegmentation)
        self.button(assets, '打开 DICOM 导入（停止当前跟踪）', self.importDicomFolder)
        note = qt.QLabel('导入不等于标定或配准。网格初始为独立预览；绑定需要以 mm 为单位的已知刚体矩阵。影像中的工具不会自动成为光学参考。')
        note.wordWrap = True
        assets.addRow(note)
        library = self.section('工具文件与多组影像')
        self.definitionFiles = sorted((ROOT/'AimooeSDK/AimTools/ToolBox').glob('*.aimtool'))
        self.definitionCombo = qt.QComboBox()
        self.definitionCombo.addItems([path.stem for path in self.definitionFiles])
        library.addRow('本组关联工具文件', self.definitionCombo)
        self.button(library, '显示该工具球心（非完整 CAD；使用上方绑定对象）', self.createDefinitionModel)
        self.groupName = qt.QLineEdit('新影像组')
        library.addRow('影像组名称', self.groupName)
        self.button(library, '批量选择影像 / 网格并建立独立组', self.importAssetGroup)
        self.groupCombo = qt.QComboBox()
        library.addRow('已保存影像组', self.groupCombo)
        self.button(library, '加载 / 切换选中影像组（未配准）', self.loadAssetGroup)
        self.reloadGroupList()
        test = self.section('Navigation Test')
        self.button(test, 'Create Test Transforms', self.createTransforms)
        self.button(test, 'Create Needle Model', self.logic.createNeedle)
        self.button(test, 'Start Tracking Simulation / Start Test Motion', self.startSimulation)
        self.button(test, 'Stop Tracking', self.logic.stop)
        self.preview = qt.QCheckBox('Single-tool Tracker preview (NOT navigation)')
        self.preview.connect('toggled(bool)', self.setPreview)
        test.addRow(self.preview)
        for title, text in [('Tool Calibration', '物理针尖标定结果为 CALIBRATED 时才允许长针单工具配准'),
                            ('Registration', '请使用上方导航工作流中的点配准与表面精配准')]:
            form = self.section(title)
            label = qt.QLabel(text)
            label.enabled = False
            form.addRow(label)
        navigation = self.section('Navigation')
        self.labels['Navigation'] = qt.QLabel('NOT READY')
        navigation.addRow('Tracking readiness', self.labels['Navigation'])
        navigation.addRow(qt.QLabel('针尖来源见坐标快照；CAD 名义值尚未实物标定。READY 仅代表双工具跟踪有效。'))
        self.button(navigation, 'Start Tracking (Hardware)', self.startHardware)
        todo = qt.QPushButton('请使用导航工作流页开始导航')
        todo.enabled = False
        navigation.addRow(todo)
        self.layout.addStretch(1)
        self.addObserver(slicer.mrmlScene, slicer.vtkMRMLScene.StartCloseEvent, self.sceneClose)
        self.addObserver(slicer.mrmlScene, slicer.vtkMRMLScene.EndCloseEvent, self.sceneClosed)
        self.createTransforms()
        self.setupNext()
        self.setupStudio()
        self.setupWorkspace()
        self.setupGuidance()
        self.setupCalibration()
        self.setupContinuity()

    def section(self, title):
        box = ctk.ctkCollapsibleButton()
        box.text = tr(title)
        self.layout.addWidget(box)
        return qt.QFormLayout(box)

    def button(self, form, title, callback):
        button = qt.QPushButton(tr(title))
        button.connect('clicked()', lambda: self.perform(callback))
        form.addRow(button)
        return button

    def perform(self, callback):
        try:
            callback()
            self.observeNodes()
            self.refresh()
        except Exception as exc:
            self.logic.error = str(exc)
            self.logic.log.exception('UI action failed')
            self.logic.publish()
            self.labels['Error'].text = tr(str(exc))
            slicer.util.errorDisplay(str(exc))

    def observeNodes(self):
        node = self.logic.nodes.get('AimooeTrackingStatus')
        event = vtk.vtkCommand.ModifiedEvent
        if node and not self.hasObserver(node, event, self.requestRefresh):
            self.addObserver(node, event, self.requestRefresh)

    def requestRefresh(self, caller=None, event=None):
        # Invalid readiness/positions are cleared immediately; numerical text is coalesced.
        status = self.logic.nodes.get('AimooeTrackingStatus')
        if status and status.GetParameter('Navigation') != 'READY':
            self.setLabelText(self.readyLabel, 'NOT READY')
            self.setLabelText(self.labels['Navigation'], 'NOT READY')
        if status and status.GetParameter('Tool') != 'TRACKING':
            for axis in 'XYZ':
                self.setLabelText(self.labels[axis], '--')
        # Event-driven coalescing: no GUI polling, at most ten text updates/sec.
        if not self.uiTimer.isActive():
            self.uiTimer.start()

    @staticmethod
    def setLabelText(label, value):
        value = tr(value)
        if label.text != value:
            label.text = value
            label.toolTip = value

    def createTransforms(self):
        self.logic.createTransforms()
        self.observeNodes()

    def startSimulation(self):
        if not self.sim.checked:
            raise ValueError('Enable Simulation Mode first')
        self.logic.startSimulation()

    def modeChanged(self, checked):
        self.logic.disconnect()
        self.logic.simulation = bool(checked)
        self.logic.publish()

    def connectHardware(self):
        if self.sim.checked:
            raise ValueError('Disable Simulation Mode to connect hardware')
        names = self.logic.connectDevice(self.interfaceName(), self.path.text,
                                         self.localIP.text.strip(),self.deviceIP.text.strip())
        for combo in [self.reference, self.tool]:
            combo.clear()
            combo.addItems(names)
        self.calibrationTool.clear()
        self.calibrationTool.addItem('')
        self.calibrationTool.addItems(names)
        normal = [entry['name'] for entry in self.logic.tracker.toolInfo
                  if entry.get('isBoard') in (False, 0, '\x00')]
        defaults = normal if len(normal) >= 2 else names
        for combo, key, default in [(self.reference, 'reference', defaults[0]),
                                     (self.tool, 'tool', defaults[min(1, len(defaults)-1)])]:
            selected = self.settings.get(key, default)
            combo.setCurrentText(selected if selected in names else default)
        calibration=self.settings.get('calibrationTool','')
        self.calibrationTool.setCurrentText(calibration if calibration in names else '')
        self.saveSettings()

    def startHardware(self):
        if self.sim.checked:
            raise ValueError('Disable Simulation Mode first')
        if self.logic.tracker:
            self.logic.tracker.minimumMatchPts = 3 if self.partialCheck.checked else 0
            self.logic.tracker.recordRaw = bool(self.rawRecord.checked)
        single = hasattr(self,'singleToolMode') and self.singleToolMode()
        self.logic.startHardware(self.tool.currentText, None if single else self.reference.currentText,
                                 self.calibrationTool.currentText or None)
        self.ensureToolModels()
        if single and self.calibrationPath().is_file():
            self.loadTipCalibration()
        self.saveSettings()

    def createDefinitionModel(self):
        index = self.definitionCombo.currentIndex
        if index < 0:
            raise ValueError('No tool definitions loaded')
        self.logic.createDefinitionModel(self.definitionFiles[index], self.assetRoleName())
        slicer.util.resetThreeDViews()

    def reloadGroupList(self):
        self.groupFiles = sorted((ROOT/'Config/Groups').glob('*.json'))
        self.groupCombo.clear()
        for path in self.groupFiles:
            data = json.loads(path.read_text(encoding='utf-8-sig'))
            self.groupCombo.addItem(data['name'] + ' / ' + data['toolName'])

    def importAssetGroup(self):
        paths = qt.QFileDialog.getOpenFileNames(slicer.util.mainWindow(), '选择本组影像和模型', '', 'Assets (*.nrrd *.nii *.nii.gz *.mha *.mhd *.stl *.obj *.ply *.vtk *.vtp)')
        if paths:
            index = self.definitionCombo.currentIndex
            self.logic.importAssetGroup(list(paths), self.groupName.text, self.definitionFiles[index] if index>=0 else None)
            self.reloadGroupList()

    def loadAssetGroup(self):
        index = self.groupCombo.currentIndex
        if index < 0:
            raise ValueError('先建立影像组')
        data = json.loads(self.groupFiles[index].read_text(encoding='utf-8-sig'))
        self.logic.importAssetGroup(data['files'], data['name'], data['toolFile'], data['id'])

    def setReferenceFrame(self):
        self.logic.setReferenceFrame(self.reference.currentText)

    def clearReferenceFrame(self):
        self.logic.clearReferenceFrame()
        self.preview.checked = True

    def importToolMesh(self):
        path = qt.QFileDialog.getOpenFileName(slicer.util.mainWindow(), '选择模型', '', '模型 (*.stl *.obj *.ply *.vtk *.vtp)')
        if path:
            role=self.assetRoleName()
            name=self.roleToolName(role)
            model=self.logic.importConfiguredModel(path,self.attachModelCheck.checked,role,self.attachMatrix.text,name)
            self.assetSelector.setCurrentNode(model)

    def bindToolMesh(self):
        node = self.assetSelector.currentNode()
        if node is None:
            raise ValueError('先选择已导入模型')
        path = qt.QFileDialog.getOpenFileName(slicer.util.mainWindow(), '选择模型到工具的刚体矩阵 JSON', '', 'JSON (*.json)')
        if path:
            role = self.assetRoleName()
            tool = self.roleToolName(role)
            self.logic.bindAsset(node, role, path, tool)

    def importVolume(self):
        self.importMedicalImage()

    def importSegmentation(self):
        path = qt.QFileDialog.getOpenFileName(slicer.util.mainWindow(), '选择分割', '', 'Segmentation (*.seg.nrrd)')
        if path:
            self.logic.importAsset(path, 'Segmentation')

    def setMarkersVisible(self, visible):
        self.logic.showMarkers = bool(visible)
        self.logic.publish()

    def legacyShowCoordinates(self):
        result = self.logic.captureCoordinates()
        self.lastCoordinateSnapshot = result
        (ROOT / 'Logs' / 'latest-coordinate-snapshot.json').write_text(
            json.dumps(result, indent=2), encoding='utf-8')
        if self.coordinatesDialog is None:
            self.coordinatesDialog = qt.QDialog(slicer.util.mainWindow())
            self.coordinatesDialog.setWindowTitle('针尖与标记球坐标快照')
            layout = qt.QVBoxLayout(self.coordinatesDialog)
            self.snapshotTitle = qt.QLabel()
            self.snapshotTitle.wordWrap = True
            layout.addWidget(self.snapshotTitle)
            self.coordinateTable = qt.QTableWidget(5, 5)
            self.coordinateTable.setHorizontalHeaderLabels(['坐标系', 'X', 'Y', 'Z', '状态 / 来源'])
            self.coordinateTable.setEditTriggers(qt.QAbstractItemView.NoEditTriggers)
            layout.addWidget(self.coordinateTable)
            layout.addWidget(qt.QLabel('四个球心（mm）：工具文件位置 / Tracker 预测位置 / 同帧实测位置'))
            self.markerTable = qt.QTableWidget(4, 4)
            self.markerTable.setHorizontalHeaderLabels(['标记点', 'Tool XYZ', 'Tracker 预测 XYZ', 'Tracker 实测 XYZ'])
            self.markerTable.setEditTriggers(qt.QAbstractItemView.NoEditTriggers)
            layout.addWidget(self.markerTable)
            button = qt.QPushButton('重新采集当前坐标')
            button.connect('clicked()', lambda: self.perform(self.showCoordinates))
            layout.addWidget(button)
            self.coordinatesDialog.resize(980, 600)
        source = ('CAD 名义针尖，未实物标定' if result['source'] == 'CAD_NOMINAL_UNCALIBRATED'
                  else result['source'])
        self.snapshotTitle.text = (f"{'模拟' if result['simulation'] else '真实 USB/设备'}快照 · 帧 {result['frame']} · {source}\n"
                                   '这是点击时的快照，不是实时读数。Reference 丢失或 Image 未配准时不显示对应坐标。')
        for i, row in enumerate(result['rows']):
            xyz = ['—'] * 3 if row['xyz'] is None else [f'{v:.3f}' for v in row['xyz']]
            status = source if row['xyz'] is not None else ('未配准 / 无有效参考' if row['space']=='Image' else '跟踪无效 / 参考缺失')
            for j, value in enumerate([row['space']] + xyz + [status]):
                self.coordinateTable.setItem(i, j, qt.QTableWidgetItem(value))
        for i in range(4):
            marker = result['markers'][i] if len(result['markers']) == 4 else {}
            values = [marker.get('name', 'M'+str(i+1))]
            for key in ['tool', 'trackerExpected', 'trackerMeasured']:
                point = marker.get(key)
                values.append('—' if point is None else ', '.join(f'{v:.3f}' for v in point))
            for j, value in enumerate(values):
                self.markerTable.setItem(i, j, qt.QTableWidgetItem(value))
        self.coordinateTable.resizeColumnsToContents()
        self.markerTable.resizeColumnsToContents()
        self.coordinatesDialog.show()
        self.coordinatesDialog.raise_()

    def setPreview(self, enabled):
        self.logic.previewOnly = bool(enabled)
        self.logic.cameraFitPending = True
        self.logic.publish()

    def saveSettings(self):
        self.settings = {'interface': self.interfaceName(),
                         'toolPath': stored_path(self.path.text),
                         'reference': self.reference.currentText,
                         'tool': self.tool.currentText,
                         'calibrationTool': self.calibrationTool.currentText,
                         'localIP':self.localIP.text.strip(),
                         'deviceIP':self.deviceIP.text.strip()}
        (ROOT / 'Config' / 'tracking.json').write_text(
            json.dumps(self.settings, indent=2), encoding='utf-8')

    def refresh(self, caller=None, event=None):
        if hasattr(self, 'studioReady'):
            self.refreshStudio()
        if hasattr(self, 'pointFitLabel'):
            self.refreshPrecisionMetrics()
        self.uiRefreshCount += 1
        node = self.logic.nodes.get('AimooeTrackingStatus')
        if node:
            for name in ['Tracker', 'Reference', 'Tool', 'CalibrationTool', 'Rate', 'Markers', 'Frame', 'ViewMode', 'ReferenceFrame', 'Quality', 'DisplayStatus', 'RejectedFrames', 'Navigation', 'Error']:
                self.setLabelText(self.labels[name], node.GetParameter(name))
            for role in ('Reference','Tool','CalibrationTool'):
                detail=node.GetParameter(role+'Detail')
                reason=node.GetParameter(role+'Reason')
                value=node.GetParameter(role)+' / '+detail
                if reason:value+=' · '+reason
                self.setLabelText(self.labels[role],value)
            self.setLabelText(self.readyLabel, node.GetParameter('Navigation'))
        else:
            for name in ['Tracker', 'Reference', 'Tool', 'CalibrationTool', 'Rate', 'Markers', 'Frame', 'ViewMode', 'ReferenceFrame', 'Quality', 'DisplayStatus', 'RejectedFrames', 'Error']:
                self.labels[name].text = '--'
            self.labels['Navigation'].text = 'NOT READY'
            self.readyLabel.text = 'NOT READY'
        matrix = vtk.vtkMatrix4x4()
        tool = self.logic.nodes.get('ToolToTracker')
        if tool:
            tool.GetMatrixTransformToParent(matrix)
        positionValid = self.logic.state('Tool') == 'TRACKING'
        if self.logic.referenceView:
            positionValid = positionValid and self.logic.state('Reference') == 'TRACKING'
            if positionValid:
                referenceMatrix = vtk.vtkMatrix4x4()
                self.logic.nodes['ReferenceToTracker'].GetMatrixTransformToParent(referenceMatrix)
                matrix = relative_matrix(matrix, referenceMatrix)
        for i, axis in enumerate('XYZ'):
            self.setLabelText(self.labels[axis], f'{matrix.GetElement(i,3):.2f}' if positionValid else '--')

    def sceneClose(self, caller=None, event=None):
        self.cleanupNavigationPresets()
        self.cleanupCalibration()
        if hasattr(self.logic,'tipOverlays'):self.logic.tipOverlays.close()
        self.cleanupStudio()
        self.logic.disconnect()
        self.uiTimer.stop()
        self.removeObservers(self.requestRefresh)
        self.logic.nodes.clear()
        self.logic.assetBindings.clear()
        self.logic.referenceView = False

    def sceneClosed(self, caller=None, event=None):
        self.refresh()

    def exit(self):
        self.cleanupNavigationPresets()
        self.cleanupCalibration()
        self.cleanupStudio()
        self.logic.disconnect()
        self.uiTimer.stop()

    def cleanup(self):
        self.cleanupNavigationPresets()
        self.cleanupCalibration()
        if hasattr(self.logic,'tipOverlays'):self.logic.tipOverlays.close()
        self.cleanupStudio()
        self.cleanupPairs()
        self.removeObservers()
        self.uiTimer.stop()
        self.uiTimer.disconnect('timeout()', self.refresh)
        if self.coordinatesDialog:
            self.coordinatesDialog.close()
        slicer.app.disconnect("aboutToQuit()", self.logic.disconnect)
        self.logic.disconnect()
        self.logic.timer.disconnect('timeout()', self.logic.update)
        self.removeObservers()

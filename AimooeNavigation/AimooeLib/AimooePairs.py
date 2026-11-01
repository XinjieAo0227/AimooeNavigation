"""Explicit STEP/definition pairing, cached conversion and role-specific display."""
import hashlib
import itertools
import json
import shutil
import time
import uuid
from pathlib import Path
import numpy as np
import qt
import vtk
import slicer
from .AimooeModels import matrix_from_rows, read_mesh, sphere
from .AimooePaths import project_path, stored_path, cad_python

ROOT = Path(__file__).resolve().parents[2]


def profiles():
    return sorted((ROOT/'AimooeNavigation/Resources/Models/Pairs').glob('*/profile.json'))


def read_pair(path):
    path = Path(path).resolve()
    data = json.loads(path.read_text(encoding='utf-8'))
    matrix_from_rows(data['cadToTool'])
    if data['units'] != 'mm' or not 0 <= data['rmsMm'] <= 0.5 or data['nextBestRmsMm'] < 2:
        raise ValueError('Unverified CAD geometry match')
    definition = path.parent/'definition.aimtool'
    if hashlib.sha256(definition.read_bytes()).hexdigest() != data['toolSHA256']:
        raise ValueError('Definition changed: rebuild the pair')
    if data['mesh'] != 'body.stl':
        raise ValueError('Invalid mesh resource')
    return data


def activate_pairs(paths):
    data = [read_pair(path) for path in paths]
    names = [item['toolName'] for item in data]
    if len(set(names)) != len(names) or len(names) < 2:
        raise ValueError('Select at least two different tool definitions')
    for first, second in itertools.combinations(data, 2):
        a, b = np.array(first['markersTool']), np.array(second['markersTool'])
        da = sorted(np.linalg.norm(a[i]-a[j]) for i,j in itertools.combinations(range(4),2))
        db = sorted(np.linalg.norm(b[i]-b[j]) for i,j in itertools.combinations(range(4),2))
        if max(abs(np.array(da)-db)) < 1.0:
            raise ValueError('Ambiguous duplicate marker geometry: '+first['toolName']+' / '+second['toolName'])
    # New immutable set, never delete or overwrite SDK source files.
    folder = ROOT/'AimooeSDK/AimTools/Sets'/uuid.uuid4().hex
    folder.mkdir(parents=True)
    for index, path in enumerate(paths):
        shutil.copy2(Path(path).parent/'definition.aimtool', folder/(str(index)+'.aimtool'))
    (ROOT/'Config/active-pairs.json').write_text(json.dumps([stored_path(p) for p in paths]), encoding='utf-8')
    return folder, names


class PairLogicMixin:
    def fitPairView(self):
        if slicer.app.layoutManager().layout == 912:
            self.fitFourViews()
            return
        models = [item[0] for item in self.assetBindings if item[0].GetScene() and item[0].GetDisplayNode().GetVisibility()]
        if not models:
            return
        bounds = [float('inf'), float('-inf')]*3
        for model in models:
            current=[0.]*6
            model.GetRASBounds(current)
            for axis in range(3):
                bounds[2*axis]=min(bounds[2*axis],current[2*axis])
                bounds[2*axis+1]=max(bounds[2*axis+1],current[2*axis+1])
        matrix=vtk.vtkMatrix4x4()
        self.nodes['ToolToTracker'].GetMatrixTransformToWorld(matrix)
        center=[(bounds[2*i]+bounds[2*i+1])/2 for i in range(3)]
        manager=slicer.app.layoutManager()
        for index in range(manager.threeDViewCount):
            view=manager.threeDWidget(index).threeDView()
            view.mrmlViewNode().SetBoxVisible(False)
            view.mrmlViewNode().SetAxisLabelsVisible(False)
            renderer=view.renderWindow().GetRenderers().GetFirstRenderer()
            camera=renderer.GetActiveCamera()
            camera.SetFocalPoint(*center)
            camera.SetPosition(*[center[i]-700*matrix.GetElement(i,2) for i in range(3)])
            camera.SetViewUp(*[matrix.GetElement(i,1) for i in range(3)])
            renderer.ResetCamera(bounds)
            renderer.ResetCameraClippingRange()

    def loadPair(self, path, role):
        if role not in ('Tool', 'Reference', 'CalibrationTool'):
            raise ValueError('Invalid role')
        data = read_pair(path)
        self.createTransforms()
        key = 'Pair_'+data['id']+'_'+role
        parent = self.node(key+'_CadTo'+role)
        parent.SetMatrixTransformToParent(matrix_from_rows(data['cadToTool']))
        parent.SetAndObserveTransformNodeID(self.nodes[role+'ToTracker'].GetID())
        model = self.node(key, 'vtkMRMLModelNode')
        model.SetAndObservePolyData(self.displayMesh(Path(path).parent/'body.stl'))
        model.CreateDefaultDisplayNodes()
        model.GetDisplayNode().SetColor(0.12, 0.17, 0.27)
        model.SetAndObserveTransformNodeID(parent.GetID())
        append = vtk.vtkAppendPolyData()
        for point in data['markersTool']:
            append.AddInputData(sphere(point, data['markerRadiusMm']))
        append.Update()
        markers = self.node(key+'_Markers', 'vtkMRMLModelNode')
        mesh = vtk.vtkPolyData()
        mesh.DeepCopy(append.GetOutput())
        markers.SetAndObservePolyData(mesh)
        markers.CreateDefaultDisplayNodes()
        markers.GetDisplayNode().SetColor(.88,.9,.94)
        markers.SetAndObserveTransformNodeID(self.nodes[role+'ToTracker'].GetID())
        for node in (model, markers):
            node.GetDisplayNode().SetVisibility2D(False)  # Slices use the exact red tip, not CAD intersections.
            node.SetAttribute('Aimooe.Pair', str(path))
            node.SetAttribute('Aimooe.LiveParent', node.GetTransformNodeID())
            node.SetAttribute('Aimooe.LiveColor', json.dumps(node.GetDisplayNode().GetColor()))
            node.SetAttribute('Aimooe.BoundTool', data['toolName'])
            self.assetBindings = [entry for entry in self.assetBindings if entry[0] != node]
            self.assetBindings.append((node, role, data['toolName']))
        if role == 'Tool':
            self.pairToolProfile = data
            # Old physical calibrations cannot be transferred to a different tool.
            tip = vtk.vtkMatrix4x4()
            for i, value in enumerate(data.get('nominalTipTool', [0,0,0])):
                tip.SetElement(i,3,value)
            self.nodes['TipToTool'].SetMatrixTransformToParent(tip)
            self.nodes['TipToTool'].SetAttribute('Aimooe.TipCalibration', data['tipSource'])
            for name in self.displayModelNames + ['OpticalMarkerLabels','MeasuredOpticalMarkers']:
                if name in self.nodes:
                    self.nodes[name].GetDisplayNode().SetVisibility(False)
                    self.nodes[name].SetAndObserveTransformNodeID(None)
            self.displayModelNames = []
            self.cadProfile = None
        self.updateAssetVisibility()
        return model

    def loadSelectedPairs(self, tool, reference):
        found = {}
        active = ROOT/'Config/active-pairs.json'
        candidates = [project_path(p) for p in json.loads(active.read_text(encoding='utf-8'))] if active.is_file() else profiles()
        for path in candidates:
            data = read_pair(path)
            for role, name in [('Tool',tool),('Reference',reference)]:
                if name == data['toolName']:
                    if role in found:
                        raise ValueError('Multiple CAD bindings for '+name+'; select a unique cached pair')
                    found[role] = path
        # An old active set must not silently hide a newly selected, verified CAD.
        for role, name in [('Tool', tool), ('Reference', reference)]:
            if role not in found:
                matches = [path for path in profiles() if read_pair(path)['toolName'] == name]
                if len(matches) == 1:
                    found[role] = matches[0]
                elif len(matches) > 1:
                    raise ValueError('Multiple verified CAD bindings for '+name+'; select an explicit tool set')
        for role, path in found.items():
            self.loadPair(path, role)
        return found


class PairWidgetMixin:
    def setupPairs(self):
        self.pairProcess = None
        self.logic.historyPreviewSeconds = 0.25
        form = self.section('STP 与 aimtool 配对（四球几何验证）')
        self.button(form, '多选 STP 并逐件选择配套 aimtool', self.importPairs)
        self.pairList = qt.QListWidget()
        self.pairList.setSelectionMode(qt.QAbstractItemView.ExtendedSelection)
        self.pairList.setMaximumHeight(100)
        form.addRow(self.pairList)
        self.pairStatus = qt.QLabel('全匹配为默认；三球仅实验预览。两球不解算有效位姿。')
        self.pairStatus.wordWrap = True
        form.addRow(self.pairStatus)
        self.button(form, '用选中配对建立独立工具集（连接前）', self.activatePairs)
        self.button(form, '按当前 Tool / Reference 同时显示模型', self.showPairs)
        self.ghostCheck = qt.QCheckBox('短暂丢失时保留 0.25 秒灰色历史模型（不是实时位姿）')
        self.ghostCheck.checked = True
        self.ghostCheck.connect('toggled(bool)', lambda value: setattr(self.logic, 'historyPreviewSeconds', 0.25 if value else 0.0))
        form.addRow(self.ghostCheck)
        self.partialCheck = qt.QCheckBox('实验：允许三球识别（仅预览，NOT READY）')
        form.addRow(self.partialCheck)
        self.reloadPairs()

    def reloadPairs(self):
        self.pairPaths = profiles()
        self.pairList.clear()
        for path in self.pairPaths:
            data = read_pair(path)
            self.pairList.addItem(data['toolName']+' | CAD 球心 RMS %.3f mm' % data['rmsMm'])

    def activatePairs(self):
        if self.logic.tracker or self.logic.running:
            raise ValueError('请先断开设备，再更换工具集')
        paths = [self.pairPaths[self.pairList.row(item)] for item in self.pairList.selectedItems()]
        folder, names = activate_pairs(paths)
        self.path.text = folder.as_posix()+'/'
        for combo in (self.tool, self.reference):
            combo.clear()
            combo.addItems(names)
        self.tool.setCurrentIndex(0)
        self.reference.setCurrentIndex(1)
        self.saveSettings()

    def showPairs(self):
        if self.logic.running:
            raise ValueError('请先停止跟踪再加载网格')
        found = self.logic.loadSelectedPairs(self.tool.currentText, self.reference.currentText)
        if not found:
            raise ValueError('当前工具名称未找到配对；先建立独立工具集')
        self.logic.fitPairView()

    def importPairs(self):
        if self.logic.running or self.pairProcess:
            raise ValueError('请先停止跟踪或等待转换完成')
        files = qt.QFileDialog.getOpenFileNames(slicer.util.mainWindow(), '多选 STEP 模型', '', 'STEP (*.stp *.step)')
        if not files:
            return
        pairs = []
        for file in files:
            definition = qt.QFileDialog.getOpenFileName(slicer.util.mainWindow(), '选择 '+Path(file).name+' 配套的 aimtool', '', 'AimTools (*.aimtool)')
            if not definition:
                return
            pairs.append({'step':file, 'definition':definition})
        self.pairJob = ROOT/'Logs'/('pair-'+uuid.uuid4().hex+'.json')
        self.pairJob.write_text(json.dumps(pairs,ensure_ascii=False),encoding='utf-8')
        python = cad_python()
        if not python.is_file():
            raise FileNotFoundError('Offline CAD Python runtime missing: '+str(python))
        process = qt.QProcess()
        self.pairProcess = process
        process.setStandardOutputFile(str(self.pairJob.with_suffix('.stdout.txt')))
        process.setStandardErrorFile(str(self.pairJob.with_suffix('.stderr.txt')))
        process.connect('finished(int,QProcess::ExitStatus)', self.pairsFinished)
        process.connect('errorOccurred(QProcess::ProcessError)', self.pairsFailed)
        self.pairStatus.text = '正在独立进程中转换并验证球心配对…'
        process.start(str(python), [str(ROOT/'scripts/build_pairs.py'),str(self.pairJob)])

    def pairsFailed(self, *args):
        if self.pairProcess:
            self.pairStatus.text = 'CAD 转换进程错误：'+self.pairProcess.errorString()
            self.logic.log.error(self.pairStatus.text)
            if self.pairProcess.state() == qt.QProcess.NotRunning:
                self.pairProcess.deleteLater()
                self.pairProcess = None

    def pairsFinished(self, *args):
        try:
            result = json.loads(self.pairJob.with_suffix('.result.json').read_text(encoding='utf-8'))
            if 'error' in result:
                raise ValueError(result['error'])
            self.reloadPairs()
            self.pairStatus.text = '转换完成。选择配对建立工具集，再选择 Tool / Reference。'
        except Exception as exc:
            self.pairStatus.text = str(exc)
            self.logic.log.exception('CAD conversion failed')
        finally:
            if self.pairProcess:
                self.pairProcess.deleteLater()
            self.pairProcess = None

    def cleanupPairs(self):
        if self.pairProcess:
            self.pairProcess.disconnect('finished(int,QProcess::ExitStatus)', self.pairsFinished)
            self.pairProcess.disconnect('errorOccurred(QProcess::ProcessError)', self.pairsFailed)
            self.pairProcess.kill()
            self.pairProcess.waitForFinished(1000)
            self.pairProcess.deleteLater()
            self.pairProcess = None

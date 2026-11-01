"""V0.3.2: diagnostic display, four views, DICOM and explicit attachment."""
import json
import time
import uuid
from pathlib import Path
import qt
import ctk
import vtk
import slicer
from .AimooePaths import project_path

ROOT = Path(__file__).resolve().parents[2]
FOUR_TRACKING_VIEWS = 912

TEXT = {
 'DisplayStatus':'模型显示状态','Help && Acknowledgement':'帮助与致谢','Help & Acknowledgement':'帮助与致谢','Data Probe':'数据探查','Show Zoomed Slice':'显示放大切片',
 'Optical Tracking':'光学定位', 'Simulation Mode':'模拟模式', 'Tracking Interface':'连接方式',
 'Connect Aimooe':'连接定位仪', 'Disconnect':'断开设备', 'Tool Configuration':'工具配置',
 'Tool Path':'工具文件目录','Reference Tool':'参考工具','Navigation Tool':'导航工具',
 'Tracking Status':'跟踪状态','Navigation Status':'导航状态','Tracker':'定位仪','Reference':'参考工具',
 'Tool':'导航工具','Rate':'更新频率','Markers':'标记球数量','Frame':'帧编号','ViewMode':'显示模式',
 'ReferenceFrame':'参考坐标系','Quality':'数据质量','RejectedFrames':'质量不合格帧数','Error':'错误信息',
 'Navigation Test':'模拟测试','Create Test Transforms':'创建测试变换','Create Needle Model':'创建针模型',
 'Start Tracking Simulation / Start Test Motion':'开始模拟运动','Stop Tracking':'停止跟踪',
 'Single-tool Tracker preview (NOT navigation)':'单工具预览（不可导航）','Tool Calibration':'工具标定',
 'Registration':'影像配准','Navigation':'导航','Tracking readiness':'双工具有效状态',
 'Start Tracking (Hardware)':'开始实时跟踪','Start Navigation — TODO: calibration and registration':'开始导航（待实现标定与配准）',
 'Pivot Calibration / Calibration RMSE: TODO':'轴心标定／标定均方根误差：待实现',
 'Image / Physical Fiducials / FRE: TODO':'影像点／实物点／配准误差：待实现',
 'READY':'双工具有效','NOT READY':'不可导航','TRACKING':'有效跟踪','STALE':'数据过期','LOST':'无有效位姿',
 'Connected':'已连接','Disconnected':'未连接','Simulation':'模拟','None':'暂无',
 'DUAL TOOL TRACKING':'双工具跟踪','TRACKER':'定位仪坐标系','REFERENCE':'参考工具坐标系',
 'TRACKER PREVIEW - NOT REGISTERED':'定位仪空间预览（未配准）','REFERENCE VIEW - NOT REGISTERED':'参考空间预览（未配准）',
 'AIMOOE NAVIGATION':'艾目易导航 · 优化版 0.3.2',
 'CAD_NOMINAL_UNCALIBRATED':'未标定的模型名义针尖',
 'CAD_SPHERE_CENTER_NOMINAL_UNCALIBRATED':'未标定的模型末端球心',
 'UNCALIBRATED_MARKER_ORIGIN':'未标定，仅使用工具原点',
}


LANGUAGE = 'zh'

def tr(text):
    text = str(text)
    if LANGUAGE == 'en':
        reverse = {value:key for key,value in TEXT.items()}
        mapping = globals().get('ENGLISH', {})
        if text in mapping:
            return mapping[text]
        for chinese, english in sorted(mapping.items(), key=lambda item: -len(item[0])):
            if len(chinese)>3:
                text=text.replace(chinese,english)
        return reverse.get(text, text)
    if text in TEXT:
        return TEXT[text]
    for old, new in {
        'Hardware status: E_HardwareStatus.HW_IR_LEFT_VOLTAGE_TOO_LOW':'左侧红外电压过低',
        'Background light: E_BackgroundLightStatus.BG_LIGHT_ABNORMAL':'背景光异常',
        'Waiting for fresh hardware status':'等待新的硬件状态',
        'NOT READY':'不可导航','TODO':'待实现',
        'Reference Tool':'参考工具','Navigation Tool':'导航工具',
        'Tool / Reference':'导航工具／参考工具','Tracker':'定位仪',
        'Reference':'参考工具','ModelToTool / ModelToReference':'模型到工具／参考工具',
        'Tool XYZ':'工具坐标','Image':'影像','Tip':'针尖',
    }.items():
        text = text.replace(old,new)
    return text


class NextLogicMixin:
    def displayMesh(self, path):
        from .AimooeModels import read_mesh
        path=Path(path)
        cache=getattr(self,'displayMeshCache',{})
        key=(str(path),path.stat().st_mtime_ns)
        if key not in cache:
            mesh=read_mesh(path)
            simplify=vtk.vtkDecimatePro()
            simplify.SetInputData(mesh)
            simplify.SetTargetReduction(.6)
            simplify.PreserveTopologyOn()
            simplify.SplittingOff()
            simplify.BoundaryVertexDeletionOff()
            simplify.SetErrorIsAbsolute(True)
            simplify.SetMaximumError(.01)
            simplify.Update()
            output=vtk.vtkPolyData()
            output.DeepCopy(simplify.GetOutput())
            cache[key]=output
            self.displayMeshCache=cache
            self.log.info('Display mesh cells %d -> %d; original CAD/calibration unchanged',mesh.GetNumberOfCells(),output.GetNumberOfCells())
        return cache[key]

    @staticmethod
    def configureVolumeDisplay(volume):
        low,high=volume.GetImageData().GetScalarRange()
        display=volume.GetDisplayNode()
        if 0 <= low <= high <= 1:
            # Sparse binary/normalized data can give a zero-width automatic
            # histogram window. Change only display, never the image samples.
            display.AutoWindowLevelOff()
            display.SetWindowLevel(max(high-low,1.0),(high+low)/2)
        else:
            display.AutoWindowLevelOn()

    def updateDiagnosticModel(self, model, role, name):
        tracker = self.tracker
        fresh = bool(getattr(self, 'diagnosticPreview', True) and self.running and not self.simulation
                     and not self.referenceView and tracker and tracker.selected.get(role)==name
                     and role in getattr(tracker, 'previewPoses', {})
                     and time.monotonic()-tracker.previewUpdated.get(role, 0)<.5)
        if not model.GetAttribute('Aimooe.LiveParent'):
            model.SetAttribute('Aimooe.LiveParent', model.GetTransformNodeID())
        liveParent = slicer.mrmlScene.GetNodeByID(model.GetAttribute('Aimooe.LiveParent'))
        if liveParent is None:
            return False
        parentID = liveParent.GetID()
        if fresh:
            model.SetAttribute('Aimooe.LastDisplayTime',str(tracker.previewUpdated[role]))
            # Diagnostic transforms NEVER feed ToolToTracker/ReferenceToTracker.
            diagnostic = self.node('Diagnostic'+role+'ToTracker')
            diagnostic.SetAndObserveTransformNodeID(self.nodes['TrackerToDisplay'].GetID())
            diagnostic.SetMatrixTransformToParent(tracker.previewPoses[role])
            diagnostic.SetAttribute('Aimooe.Valid','False')
            if liveParent.GetID() != self.nodes[role+'ToTracker'].GetID():
                offset = self.node('DiagnosticOffset_'+model.GetID())
                matrix=vtk.vtkMatrix4x4()
                liveParent.GetMatrixTransformToParent(matrix)
                offset.SetMatrixTransformToParent(matrix)
                offset.SetAndObserveTransformNodeID(diagnostic.GetID())
                parentID=offset.GetID()
            else:
                parentID=diagnostic.GetID()
        if (not fresh and self.running and not self.referenceView
                and tracker and tracker.selected.get(role)==name
                and role not in getattr(tracker,'previewPoses',{})
                and self.state(role)!='TRACKING'
                and time.monotonic()-float(model.GetAttribute('Aimooe.LastDisplayTime') or 0)
                    <getattr(self,'historyPreviewSeconds',0)):
            parentID=model.GetTransformNodeID()  # 250 ms hold, never a current pose.
        if model.GetTransformNodeID()!=parentID:
            model.SetAndObserveTransformNodeID(parentID)
        model.SetAttribute('Aimooe.DiagnosticPreview',str(fresh))
        return fresh

    def fourTrackingViews(self):
        manager=slicer.app.layoutManager()
        names=['1','AimSide','AimTop','AimIso']
        rows=[]
        for i in range(2):
            cells=[]
            for j in range(2):
                k=2*i+j
                cells.append('<item><view class="vtkMRMLViewNode" singletontag="'+names[k]+'"><property name="viewlabel" action="default">'+['正视','侧视','俯视','立体'][k]+'</property></view></item>')
            rows.append('<item><layout type="horizontal">'+''.join(cells)+'</layout></item>')
        node=manager.layoutLogic().GetLayoutNode()
        if not node.IsLayoutDescription(FOUR_TRACKING_VIEWS):
            node.AddLayoutDescription(FOUR_TRACKING_VIEWS,'<layout type="vertical">'+''.join(rows)+'</layout>')
        manager.setLayout(FOUR_TRACKING_VIEWS)
        self.fitFourViews()

    def fitFourViews(self):
        manager=slicer.app.layoutManager()
        directions=[(0,0,-1),(1,0,0),(0,1,0),(1,1,-1)]
        for index in range(manager.threeDViewCount):
            view=manager.threeDWidget(index).threeDView()
            renderer=view.renderWindow().GetRenderers().GetFirstRenderer()
            view.renderWindow().SetMultiSamples(0)
            camera=renderer.GetActiveCamera()
            direction=directions[index%4]
            camera.SetFocalPoint(0,0,0)
            camera.SetPosition(*[700*v for v in direction])
            camera.SetViewUp(*( (0,0,1) if index in (1,2) else (0,1,0)))
            camera.SetParallelProjection(index!=3)
            renderer.ResetCamera()
            renderer.ResetCameraClippingRange()
            view.mrmlViewNode().SetBoxVisible(False)
            view.mrmlViewNode().SetAxisLabelsVisible(False)

    def importDicomDirectory(self, folder, selectedSeries=None):
        if self.running:
            raise ValueError('请先停止跟踪，再导入影像')
        folder=Path(folder)
        if not folder.is_dir():
            raise FileNotFoundError('影像目录不存在：'+str(folder))
        from DICOMLib import DICOMUtils
        database=ROOT/'Logs/DICOM'/uuid.uuid4().hex
        database.mkdir(parents=True)
        with DICOMUtils.TemporaryDICOMDatabase(str(database)) as db:
            from pydicom.misc import is_dicom
            files=[p for p in folder.rglob('*') if p.is_file() and (p.suffix.lower() in ('.dcm','.ima') or is_dicom(str(p)))]
            indexer=ctk.ctkDICOMIndexer()
            for file in files:
                indexer.addFile(db,str(file),False)
            indexer.waitForImportFinished()
            series=[series for patient in db.patients() for study in db.studiesForPatient(patient) for series in db.seriesForStudy(study)]
            if not series:
                raise ValueError('目录内没有可导入的 DICOM 序列')
            selected=list(selectedSeries or series)
            if not set(selected).issubset(series):
                raise ValueError('所选影像序列不存在')
            nodes=DICOMUtils.loadSeriesByUID(selected,enabledPluginClassNames=['DICOMScalarVolumePlugin'])
        volumes=[slicer.mrmlScene.GetNodeByID(identity) for identity in nodes]
        volumes=[node for node in volumes if node and node.IsA('vtkMRMLVolumeNode')]
        if not volumes:
            raise ValueError('未加载标量影像；其他 DICOM 类型请使用标准影像导入器')
        for volume in volumes:
            volume.SetAttribute('Aimooe.Registration','UNREGISTERED')
            volume.SetAttribute('Aimooe.Attached','False')
            volume.SetName(slicer.mrmlScene.GenerateUniqueName('影像序列'))
            self.configureVolumeDisplay(volume)
        slicer.util.setSliceViewerLayers(background=volumes[0])
        slicer.util.resetSliceViews()
        return volumes

    def importConfiguredModel(self,path,attach=False,role='Tool',matrixPath=None,toolName=None):
        if self.running:
            raise ValueError('请先停止跟踪，再导入模型')
        if attach:
            if not matrixPath or not Path(matrixPath).is_file():
                raise ValueError('挂载需要已知的模型到工具矩阵，不能仅凭文件名自动对齐')
            data=json.loads(Path(matrixPath).read_text(encoding='utf-8-sig'))
            from .AimooeModels import matrix_from_rows
            matrix_from_rows(data['matrix'])
            if data.get('units')!='mm' or data.get('to')!=role or data.get('toolName')!=toolName:
                raise ValueError('挂载矩阵的单位、工具名称或绑定对象不一致')
        node=self.importAsset(path,'Model',reuse=False)
        node.SetAttribute('Aimooe.Attached','False')
        if attach:
            self.bindAsset(node,role,matrixPath,toolName)
            node.SetAttribute('Aimooe.Attached','True')
        return node


class NextWidgetMixin:
    def enter(self):
        for child in slicer.util.findChildren(slicer.util.mainWindow()):
            value=getattr(child,'text',None)
            if isinstance(value,str) and value in ('Help && Acknowledgement','Help & Acknowledgement','Data Probe','Show Zoomed Slice'):
                child.text=tr(value)

    def autoWindowLevel(self):
        node=self.volumeSelector.currentNode()
        if node:
            self.logic.configureVolumeDisplay(node)

    def interfaceName(self):
        return ['Ethernet','USB','WiFi'][self.interface.currentIndex]

    def assetRoleName(self):
        return ['Tool','Reference','CalibrationTool'][self.assetRole.currentIndex]

    def roleToolName(self, role):
        return {'Tool':self.tool.currentText,
                'Reference':self.reference.currentText,
                'CalibrationTool':self.calibrationTool.currentText}.get(role,'')

    def setupNext(self):
        self.parent.setFont(qt.QFont('Microsoft YaHei UI',9))
        active=ROOT/'Config/active-pairs.json'
        if active.is_file():
            names=[json.loads(project_path(path).read_text(encoding='utf-8'))['toolName'] for path in json.loads(active.read_text(encoding='utf-8'))]
            for combo,key in [(self.tool,'tool'),(self.reference,'reference')]:
                combo.clear()
                combo.addItems(names)
                combo.setCurrentText(self.settings.get(key,names[0] if names else ''))
        if hasattr(self,'reloadCollapsibleButton'):
            self.reloadCollapsibleButton.hide()
        self.interface.setItemText(0,'以太网')
        self.assetRole.setItemText(0,'导航工具')
        self.assetRole.setItemText(1,'参考工具')
        self.assetRole.setItemText(2,'标定工具')
        form=self.section('多视角与影像导入')
        self.layout.insertWidget(2,form.parentWidget())
        self.button(form,'四窗口器械视图（同步显示）',self.logic.fourTrackingViews)
        self.button(form,'影像四窗口（横断、冠状、矢状及三维）',self.volumeLayout)
        self.button(form,'导入影像文件或医学影像序列',self.importMedicalImage)
        self.volumeSelector=slicer.qMRMLNodeComboBox()
        self.volumeSelector.nodeTypes=['vtkMRMLScalarVolumeNode']
        self.volumeSelector.noneEnabled=True
        self.volumeSelector.noneDisplay="未选择影像"
        self.volumeSelector.addEnabled=False
        self.volumeSelector.removeEnabled=False
        self.volumeSelector.selectNodeUponCreation=False
        self.volumeSelector.setMRMLScene(slicer.mrmlScene)
        self.volumeSelector.connect('currentNodeChanged(vtkMRMLNode*)',lambda node:slicer.util.setSliceViewerLayers(background=node))
        form.addRow('当前影像',self.volumeSelector)
        self.button(form,'自动调整影像窗宽窗位',self.autoWindowLevel)
        self.button(form,'导入医学影像文件夹',self.importDicomFolder)
        self.diagnosticCheck=qt.QCheckBox('告警时保持模型与实时诊断坐标')
        self.diagnosticCheck.checked=True
        self.diagnosticCheck.connect('toggled(bool)',lambda value:setattr(self.logic,'diagnosticPreview',bool(value)))
        form.addRow(self.diagnosticCheck)
        self.diagnosticLabel=qt.QLabel('模型保持原色；数据质量单独标注。未配准的影像不会随器械移动。')
        self.diagnosticLabel.wordWrap=True
        form.addRow(self.diagnosticLabel)
        form=self.section('模型挂载设置')
        self.attachModelCheck=qt.QCheckBox('将导入模型挂载到光学工具（默认不挂载）')
        form.addRow(self.attachModelCheck)
        self.attachMatrix=qt.QLineEdit()
        form.addRow('模型到工具矩阵文件',self.attachMatrix)
        self.button(form,'选择挂载矩阵',self.chooseAttachMatrix)
        form.addRow(qt.QLabel('绑定对象使用上方“模型绑定对象”；挂载需要实际标定或配准矩阵。'))
        self.button(form,'按配置导入模型',self.importToolMesh)
        for child in slicer.util.findChildren(self.parent):
            value=getattr(child,'text',None)
            if isinstance(value,str) and value:
                translated=tr(value)
                if translated!=value:
                    child.text=translated
        for child in slicer.util.findChildren(slicer.util.mainWindow()):
            value=getattr(child,'text',None)
            if isinstance(value,str) and value in ('Help && Acknowledgement','Help & Acknowledgement','Data Probe','Show Zoomed Slice'):
                child.text=tr(value)
        self.logic.fourTrackingViews()

    def volumeLayout(self):
        slicer.app.layoutManager().setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutFourUpView)
        slicer.util.resetSliceViews()

        qt.QTimer.singleShot(0, slicer.util.resetSliceViews)

    def chooseAttachMatrix(self):
        path=qt.QFileDialog.getOpenFileName(slicer.util.mainWindow(),'选择模型到工具矩阵','','矩阵文件 (*.json)')
        if path:
            self.attachMatrix.text=path

    def importMedicalImage(self):
        if self.logic.running:
            raise ValueError('请先停止跟踪，再导入影像')
        path=qt.QFileDialog.getOpenFileName(slicer.util.mainWindow(),'选择影像文件',str(ROOT/'ScalarVolume_11'),'影像 (*.dcm *.nrrd *.nhdr *.nii *.nii.gz *.mha *.mhd *.vti *.hdr *.img);;所有文件 (*)')
        if not path:
            return
        if Path(path).suffix.lower()=='.dcm':
            self.loadDicomFolder(str(Path(path).parent))
        else:
            volume=self.logic.importAsset(path,'Volume')
            self.logic.configureVolumeDisplay(volume)
            self.volumeSelector.setCurrentNode(volume)
            self.volumeLayout()

    def importDicomFolder(self):
        path=qt.QFileDialog.getExistingDirectory(slicer.util.mainWindow(),'选择医学影像目录',str(ROOT))
        if path:
            self.loadDicomFolder(path)

    def loadDicomFolder(self,path):
        volumes=self.logic.importDicomDirectory(path)
        self.volumeSelector.setCurrentNode(volumes[0])
        self.volumeLayout()
        self.diagnosticLabel.text='已加载 %d 组影像，均未配准。影像切片不跟随器械；请选择需要查看的影像。'%len(volumes)

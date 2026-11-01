"""0.8.2 workflow: live diagnostics, registration, capture, bilingual controls."""
import json
import math
import time
from pathlib import Path
import numpy as np
import qt
import vtk
import slicer
from . import AimooeNext as language
from .AimooeCoordinates import transform_point
from .AimooeModels import matrix_from_rows
from .AimooeCapture import CaptureSession, RegionOverlay

ROOT = Path(__file__).resolve().parents[2]


def rigid_fit(source, target):
    """Corresponding Reference-mm to Image-RAS-mm points, proper rotation only."""
    source, target = np.asarray(source,float), np.asarray(target,float)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3 or len(source)<3:
        raise ValueError('至少三对一一对应的点 / At least three corresponding points required')
    if not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError('Non-finite registration points')
    a, b = source-source.mean(0), target-target.mean(0)
    for points in (a,b):
        singular = np.linalg.svd(points, compute_uv=False)
        if singular[0]<1e-6 or singular[1]/singular[0]<.01:
            raise ValueError('配准点接近共线 / Registration points are nearly collinear')
    u, _, vt = np.linalg.svd(a.T@b)
    correction = np.eye(3)
    correction[2,2] = np.linalg.det(vt.T@u.T)
    rotation = vt.T@correction@u.T
    translation = target.mean(0)-rotation@source.mean(0)
    matrix = np.eye(4)
    matrix[:3,:3], matrix[:3,3] = rotation, translation
    residual = np.linalg.norm(source@rotation.T+translation-target,axis=1)
    return matrix_from_rows(matrix.tolist()), float(np.sqrt(np.mean(residual**2))), residual.tolist()


class StudioLogicMixin:
    def fourTrackingViews(self):
        self.imageView = False
        self.imageNavigation = False
        super().fourTrackingViews()
        self.publish()

    def updateAssetVisibility(self):
        super().updateAssetVisibility()
        if getattr(self, 'imageView', False) and not getattr(self, 'imageNavigation', False):
            for model, _, _ in self.assetBindings:
                if model.GetScene() is not None:
                    model.GetDisplayNode().SetVisibility(False)
            for name in self.displayModelNames:
                if name in self.nodes:
                    self.nodes[name].GetDisplayNode().SetVisibility(False)

    def applySdkTip(self):
        sample = getattr(self.tracker,'samples',{}).get('Tool') if self.tracker else None
        if self.simulation or not sample or sample['tipTool'] is None:
            return
        tip = vtk.vtkMatrix4x4()
        for i,value in enumerate(sample['tipTool']):
            tip.SetElement(i,3,value)
        node = self.nodes['TipToTool']
        old = vtk.vtkMatrix4x4()
        node.GetMatrixTransformToParent(old)
        if any(abs(old.GetElement(i,3)-tip.GetElement(i,3))>1e-9 for i in range(3)):
            node.SetMatrixTransformToParent(tip)
        node.SetAttribute('Aimooe.TipCalibration','SDK_TOOL_FILE_TIP_NOT_INDEPENDENTLY_VALIDATED')

    def liveCoordinates(self):
        result = self.captureCoordinates()
        result['samples'] = {}
        result['trackedTools'] = []
        tracker = self.tracker
        if self.simulation or tracker is None:
            return result
        samples = {role:s for role,s in getattr(tracker,'samples',{}).items()
                   if 0<=time.monotonic()-s['monotonic']<.5}
        result['samples'] = samples
        # One SDK matching call already recognized all loaded tool files on this
        # frame; this panel only reads its copied snapshot.
        selected=getattr(tracker,'selected',{})
        allSamples={name:s for name,s in getattr(tracker,'allToolSamples',{}).items()
                    if 0<=time.monotonic()-s['monotonic']<.5}
        result['allToolSamples']=allSamples
        roleByName={name:role for role,name in selected.items()}
        for name in getattr(tracker,'names',[]):
            role=roleByName.get(name,'未分配 / Unassigned')
            sample=allSamples.get(name) or samples.get(role)
            position=[sample['matrix'][i][3] for i in range(3)] if sample else None
            result['trackedTools'].append({'role':role,'name':name,'trackerXYZ':position,
                                           'frame':sample['frame'] if sample else None,
                                           'status':('AMBIGUOUS_SHARED_MARKERS' if sample and sample['ambiguous']
                                                     else 'SDK_DIAGNOSTIC' if sample and sample['quality']
                                                     else 'TRACKING' if sample else 'NOT_VISIBLE')})
        if not self.running:
            return result
        tool = samples.get('Tool')
        ref = samples.get('Reference')
        if tool is None:
            return result
        matrix = matrix_from_rows(tool['matrix'])
        tip = tool['tipTool']
        if tip is not None and getattr(self,'tipMode','SDK')=='SDK':
            result['source'] = 'SDK toolCstip / 工具文件针尖（未独立验证）'
            tipTracker = transform_point(matrix,tip)
            result['rows'][1]['xyz'] = tip
        else:
            tip = result['rows'][1]['xyz']
            tipTracker = transform_point(matrix,tip) if tip is not None else None
        result['rows'][2]['xyz'] = tipTracker
        result['rows'][2]['diagnostic'] = bool(tool['quality'] or tool['ambiguous'])
        result['rows'][3]['xyz'] = None
        result['rows'][4]['xyz'] = None
        if ref and self.referenceEnabled and ref['frame']==tool['frame'] and not tool['ambiguous'] and not ref['ambiguous'] and tipTracker is not None:
            inv = vtk.vtkMatrix4x4()
            vtk.vtkMatrix4x4.Invert(matrix_from_rows(ref['matrix']),inv)
            referenceTip = transform_point(inv,tipTracker)
            result['rows'][3]['xyz'] = referenceTip
            registration = self.nodes.get('ReferenceToImage')
            if registration and registration.GetAttribute('Aimooe.RegistrationValidated')=='True':
                m = vtk.vtkMatrix4x4()
                registration.GetMatrixTransformToParent(m)
                result['rows'][4]['xyz'] = transform_point(m,referenceTip)
        residuals = []
        # Only the primary Tool has local marker geometry for the residual.
        # Other recognized tools have measured positions but no CAD prediction.
        for i, marker in enumerate(result['markers']):
            marker['trackerExpected'] = transform_point(matrix,marker['tool'])
            marker['trackerMeasured'] = tool['markers'][i] if i<len(tool['markers']) else None
            if marker['trackerMeasured'] is not None:
                residuals.append(np.linalg.norm(np.array(marker['trackerExpected'])-marker['trackerMeasured']))
            marker['name']='Tool '+marker['name']
        result['slicerMarkerResidualRmsMm'] = float(np.sqrt(np.mean(np.square(residuals)))) if residuals else None
        for name,sample in allSamples.items():
            if name==selected.get('Tool'):continue
            for index,position in enumerate(sample['markers']):
                result['markers'].append({'name':name+' M'+str(index+1),
                                          'trackerExpected':None,'trackerMeasured':position})
        result['sdkTipConsistencyMm'] = (float(np.linalg.norm(np.array(transform_point(matrix,tool['tipTool']))-tool['tipTracker']))
                                       if tool['tipTool'] is not None and tool['tipTracker'] is not None else None)
        for row in result['rows']:
            row['status'] = ('UNAVAILABLE' if row['xyz'] is None else
                             'SDK_DIAGNOSTIC' if tool['quality'] or tool['ambiguous'] else result['source'])
        result['frame'] = tool['frame']
        result['quality'] = tracker.quality_message
        return result

    def reconstructVolume(self, volume, threshold=None):
        if volume is None or volume.GetImageData() is None:
            raise ValueError('请先选择影像 / Select an image first')
        low,high = volume.GetImageData().GetScalarRange()
        if low==high:
            raise ValueError('影像没有灰度变化 / Constant image')
        if threshold is None and not (0<=low<high<=1):
            # General CT/MRI: use Slicer's transfer-function volume renderer,
            # never assume that MRI intensity is a CT HU bone threshold.
            logic = slicer.modules.volumerendering.logic()
            display = logic.CreateDefaultVolumeRenderingNodes(volume)
            display.SetVisibility(True)
            return display
        iso = (low+high)/2 if threshold is None else float(threshold)
        if not low<iso<high:
            raise ValueError('重建阈值必须在影像灰度范围内 / Threshold outside scalar range')
        geometry = vtk.vtkMatrix4x4()
        volume.GetIJKToRASMatrix(geometry)
        key = str((volume.GetImageData().GetMTime(),iso,
                   tuple(geometry.GetElement(i,j) for i in range(4) for j in range(4))))
        existing = slicer.mrmlScene.GetNodeByID(volume.GetAttribute('Aimooe.SurfaceID') or '')
        if existing and existing.GetAttribute('Aimooe.SurfaceKey')==key:
            existing.SetAndObserveTransformNodeID(volume.GetTransformNodeID())
            existing.GetDisplayNode().SetVisibility(True)
            return existing
        image = vtk.vtkImageData()
        image.ShallowCopy(volume.GetImageData())
        image.SetOrigin(0,0,0)
        image.SetSpacing(1,1,1)
        image.SetDirectionMatrix(1,0,0,0,1,0,0,0,1)
        surface = vtk.vtkFlyingEdges3D()
        surface.SetInputData(image)
        surface.SetValue(0,iso)
        surface.ComputeNormalsOn()
        matrix = vtk.vtkMatrix4x4()
        volume.GetIJKToRASMatrix(matrix)
        transform = vtk.vtkTransform()
        transform.SetMatrix(matrix)
        converted = vtk.vtkTransformPolyDataFilter()
        converted.SetTransform(transform)
        converted.SetInputConnection(surface.GetOutputPort())
        converted.Update()
        if converted.GetOutput().GetNumberOfPoints()==0:
            raise ValueError('阈值没有生成表面 / No surface at threshold')
        model = slicer.mrmlScene.GetNodeByID(volume.GetAttribute('Aimooe.SurfaceID') or '')
        if model is None:
            model = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode',volume.GetName()+'_3D')
            model.CreateDefaultDisplayNodes()
            volume.SetAttribute('Aimooe.SurfaceID',model.GetID())
        data = vtk.vtkPolyData()
        data.DeepCopy(converted.GetOutput())
        model.SetAndObservePolyData(data)
        model.SetAndObserveTransformNodeID(volume.GetTransformNodeID())
        model.SetAttribute('Aimooe.SourceVolume',volume.GetID())
        model.SetAttribute('Aimooe.SurfaceKey',key)
        model.SetAttribute('Aimooe.Alignment','IMAGE_RAS')
        model.GetDisplayNode().SetColor(.92,.85,.72)
        model.GetDisplayNode().SetVisibility(True)
        return model

    def registerPoints(self, source, target, volume, referenceName):
        if volume is None or not referenceName:
            raise ValueError('选择影像和参考工具 / Select image and reference tool')
        matrix, rms, residual = rigid_fit(source,target)
        self.createTransforms()
        node = self.node('ReferenceToImage')
        node.SetMatrixTransformToParent(matrix)
        node.SetAttribute('Aimooe.RegistrationValidated','True')
        node.SetAttribute('Aimooe.ReferenceName',referenceName)
        node.SetAttribute('Aimooe.VolumeID',volume.GetID())
        node.SetAttribute('Aimooe.FREmm',str(rms))
        node.SetAttribute('Aimooe.RegistrationMethod','Corresponding landmarks; FRE is not target accuracy')
        self.imageNavigation = False
        return rms, residual

    def updateDisplayFrame(self):
        if not getattr(self,'imageNavigation',False):
            return super().updateDisplayFrame()
        registration = self.nodes.get('ReferenceToImage')
        if registration and registration.GetAttribute('Aimooe.SourceFrame')=='Tracker':
            if (registration.GetAttribute('Aimooe.RegistrationValidated')!='True' or
                    self.state('Tool')!='TRACKING'):
                self.nodes['TrackerToDisplay'].SetAttribute('Aimooe.Valid','False')
                self.imageNavigation=False
                return super().updateDisplayFrame()
            result=vtk.vtkMatrix4x4()
            registration.GetMatrixTransformToParent(result)
            self.nodes['TrackerToDisplay'].SetMatrixTransformToParent(result)
            self.nodes['TrackerToDisplay'].SetAttribute('Aimooe.Valid','True')
            self.nodes['TrackerToDisplay'].SetAttribute('Aimooe.DisplayFrame','Image_SingleToolTrackerFixed')
            return
        if (not registration or registration.GetAttribute('Aimooe.RegistrationValidated')!='True'
                or not self.ready()):
            self.nodes['TrackerToDisplay'].SetAttribute('Aimooe.Valid','False')
            # A frozen image-space instrument must not appear to be live.
            self.imageNavigation = False
            return super().updateDisplayFrame()
        ref, reg, inv, combined = [vtk.vtkMatrix4x4() for _ in range(4)]
        self.nodes['ReferenceToTracker'].GetMatrixTransformToParent(ref)
        registration.GetMatrixTransformToParent(reg)
        vtk.vtkMatrix4x4.Invert(ref,inv)
        vtk.vtkMatrix4x4.Multiply4x4(reg,inv,combined)
        previous=vtk.vtkMatrix4x4()
        self.nodes['TrackerToDisplay'].GetMatrixTransformToParent(previous)
        if any(previous.GetElement(i,j)!=combined.GetElement(i,j) for i in range(4) for j in range(4)):
            self.nodes['TrackerToDisplay'].SetMatrixTransformToParent(combined)
        self.nodes['TrackerToDisplay'].SetAttribute('Aimooe.Valid','True')
        self.nodes['TrackerToDisplay'].SetAttribute('Aimooe.DisplayFrame','Image')


class StudioWidgetMixin:
    def setupStudio(self):
        self.studioReady = True
        self.capture = CaptureSession(ROOT,self.logic.log)
        slicer.app.connect('aboutToQuit()',self.capture.shutdown)
        self.liveDialog = None
        self.liveRefreshCount = 0
        self.liveTimer = qt.QTimer()
        self.liveTimer.connect('timeout()', self.refreshCoordinatesIfVisible)
        self.baseline = None
        self.languageWidgets = []
        self.interface.addItem('Wi-Fi（SDK 无线接口）')
        if self.settings.get('interface')=='WiFi':
            self.interface.setCurrentIndex(2)
        self.ghostCheck.text = '短暂丢失保留 0.25 秒原色模型（历史位置）'
        # Keep every advanced function available, initially collapse it.
        for child in slicer.util.findChildren(self.parent):
            if child.inherits('ctkCollapsibleButton'):
                child.collapsed = child.text not in ('光学定位','工具配置','跟踪状态')
                if child.text in ('影像配准','导航'):
                    child.hide()  # Replaced by the functional registration workflow below.
        form = self.section('工作流程 · 0.8.2')
        self.layout.insertWidget(0,form.parentWidget())
        self.languageButton = qt.QPushButton('English')
        self.languageButton.connect('clicked()',self.toggleLanguage)
        form.addRow(self.languageButton)
        form.addRow(qt.QLabel('连接 → 选择工具 → 跟踪 → 导入影像 → 点配准 → 影像空间显示'))
        self.button(form,'使用说明与更新日志',self.showHelp)
        self.button(form,'正向观察（仅调整相机）',self.frontCamera)
        self.button(form,'反向观察（仅调整相机）',self.reverseCamera)
        self.button(form,'重建当前影像',self.reconstructSelected)
        form = self.section('点配准与影像空间显示')
        self.referencePoints = self.pointSelector()
        self.imagePoints = self.pointSelector()
        form.addRow('实物点（参考工具坐标，毫米）',self.referencePoints)
        form.addRow('影像点（RAS 坐标，毫米）',self.imagePoints)
        self.button(form,'创建配准点列表',self.createPointLists)
        self.button(form,'采集当前选定针尖为实物点',self.collectPoint)
        self.button(form,'计算点配准',self.registerSelected)
        self.registrationLabel = qt.QLabel('未配准；两组点按列表顺序一一对应')
        self.registrationLabel.wordWrap = True
        form.addRow(self.registrationLabel)
        self.button(form,'开始影像空间显示',self.startImageDisplay)
        self.button(form,'停止影像空间显示',self.stopImageDisplay)
        form = self.section('截图与录屏')
        self.capturePath = qt.QLineEdit(str(ROOT/'Captures'))
        form.addRow('保存目录',self.capturePath)
        self.button(form,'选择保存目录',self.chooseCaptureDirectory)
        self.button(form,'框选区域',self.selectCaptureRegion)
        self.button(form,'恢复整窗',self.clearCaptureRegion)
        self.imageFormat = qt.QComboBox()
        self.imageFormat.addItems(['PNG','JPG','BMP'])
        form.addRow('截图格式',self.imageFormat)
        self.videoFormat = qt.QComboBox()
        self.videoFormat.addItems(['MP4','MKV','AVI'])
        form.addRow('录像格式',self.videoFormat)
        self.button(form,'截图',self.takeScreenshot)
        self.button(form,'开始录屏',self.startRecording)
        self.button(form,'停止录屏',self.capture.stop)
        self.captureLabel = qt.QLabel('未录制；10 帧/秒，编码积压时丢弃录制帧')
        self.captureLabel.wordWrap = True
        form.addRow(self.captureLabel)
        # The table is a project resource so translations can be extended.
        path = ROOT/'Config/translations-en.json'
        self.english = json.loads(path.read_text(encoding='utf-8')) if path.is_file() else {}
        self.english.update({v:k for k,v in language.TEXT.items()})
        language.ENGLISH = self.english
        self.english['艾目易导航 · 优化版 0.3.2']='Aimooe Navigation 0.3.4'
        for child in slicer.util.findChildren(self.parent):
            value = getattr(child,'text',None)
            if isinstance(value,str) and value and child not in (self.languageButton,self.capturePath):
                if child.inherits('QLineEdit'):
                    continue
                if value=='艾目易导航 · 优化版 0.3.2':
                    child.text='艾目易导航 · 0.8.2'
                    value=child.text
                self.languageWidgets.append((child,value,self.english.get(value,value)))

    def toggleLanguage(self):
        language.LANGUAGE = 'en' if language.LANGUAGE=='zh' else 'zh'
        for widget, chinese, english in self.languageWidgets:
            widget.text = english if language.LANGUAGE=='en' else chinese
        self.languageButton.text = '简体中文' if language.LANGUAGE=='en' else 'English'
        self.interface.setItemText(0,'Ethernet' if language.LANGUAGE=='en' else '以太网')
        self.interface.setItemText(2,'Wi-Fi (SDK)' if language.LANGUAGE=='en' else 'Wi-Fi（SDK 无线接口）')
        self.assetRole.setItemText(0,'Navigation tool' if language.LANGUAGE=='en' else '导航工具')
        self.assetRole.setItemText(1,'Reference tool' if language.LANGUAGE=='en' else '参考工具')
        self.refresh()

    def showCoordinates(self):
        if self.liveDialog is None:
            self.liveDialog = qt.QDialog(slicer.util.mainWindow())
            self.liveDialog.setWindowTitle('实时坐标 / Live coordinates · mm')
            layout = qt.QVBoxLayout(self.liveDialog)
            self.liveTitle = qt.QLabel()
            self.liveTitle.wordWrap = True
            layout.addWidget(self.liveTitle)
            refreshRow=qt.QHBoxLayout()
            self.liveCheck = qt.QCheckBox('实时刷新 / Live refresh')
            self.liveCheck.checked = True
            self.liveRate=qt.QSpinBox();self.liveRate.setRange(1,20);self.liveRate.setValue(10);self.liveRate.setSuffix(' Hz')
            self.liveRate.setToolTip('面板刷新独立于 SDK 采集；限制为 20 Hz，避免表格绘制抢占三维跟踪刷新。')
            self.liveCheck.connect('toggled(bool)',self.setLiveRefresh)
            self.liveRate.connect('valueChanged(int)',self.liveRateChanged)
            refreshRow.addWidget(self.liveCheck);refreshRow.addWidget(qt.QLabel('刷新频率'));refreshRow.addWidget(self.liveRate);refreshRow.addStretch(1)
            layout.addLayout(refreshRow)
            self.liveTable = qt.QTableWidget(5,4)
            self.liveTable.setHorizontalHeaderLabels(['坐标系 / Frame','X','Y','Z'])
            self.liveTable.setEditTriggers(qt.QAbstractItemView.NoEditTriggers)
            layout.addWidget(self.liveTable)
            self.allToolTable=qt.QTableWidget(0,5)
            self.allToolTable.setHorizontalHeaderLabels(['角色 / Role','工具 / Tool','Tracker X,Y,Z mm','帧 / Frame','状态 / Status'])
            self.allToolTable.setEditTriggers(qt.QAbstractItemView.NoEditTriggers)
            layout.addWidget(self.allToolTable)
            self.liveMarkers = qt.QTableWidget(4,3)
            self.liveMarkers.setHorizontalHeaderLabels(['标记球 / Marker','变换预测 / Predicted XYZ','SDK 实测 / Measured XYZ'])
            self.liveMarkers.setEditTriggers(qt.QAbstractItemView.NoEditTriggers)
            self.liveMarkers.setColumnWidth(1,300)
            self.liveMarkers.setColumnWidth(2,300)
            layout.addWidget(self.liveMarkers)
            self.metricsLabel = qt.QLabel()
            self.metricsLabel.wordWrap = True
            layout.addWidget(self.metricsLabel)
            self.comparisonLabel = qt.QLabel('Toolbox：尚未导入外部记录 / No external Toolbox record')
            self.comparisonLabel.wordWrap = True
            layout.addWidget(self.comparisonLabel)
            for label, callback in [('导出快照 / Export snapshot',self.exportCoordinates),
                                    ('导入 Toolbox 对比记录 / Import Toolbox record',self.importComparison)]:
                button=qt.QPushButton(label)
                button.connect('clicked()',lambda callback=callback:self.perform(callback))
                layout.addWidget(button)
            self.liveDialog.resize(980,680)
        self.liveDialog.show()
        self.liveDialog.raise_()
        self.refreshCoordinates()
        self.setLiveRefresh(self.liveCheck.checked)

    def liveRateChanged(self, rate):
        self.liveTimer.setInterval(max(33,int(round(1000/max(int(rate),1)))))
        if self.liveTimer.isActive(): self.liveTimer.start()

    def setLiveRefresh(self, enabled):
        self.liveRateChanged(self.liveRate.value if hasattr(self,'liveRate') else 10)
        if enabled and self.liveDialog and self.liveDialog.visible: self.liveTimer.start()
        else: self.liveTimer.stop()

    def refreshCoordinatesIfVisible(self):
        if not self.liveDialog or not self.liveDialog.visible or not self.liveCheck.checked:
            self.liveTimer.stop(); return
        self.refreshCoordinates()

    @staticmethod
    def tableCell(table,row,column,text):
        item=table.item(row,column)
        if item is None:
            table.setItem(row,column,qt.QTableWidgetItem(str(text)))
        elif item.text()!=str(text):
            item.setText(str(text))

    def refreshCoordinates(self):
        self.liveRefreshCount += 1
        result=self.logic.liveCoordinates()
        self.lastCoordinateSnapshot=result
        diagnostic=bool(result.get('quality'))
        self.liveTitle.text=('帧 / Frame: '+str(result['frame'])+' · '+str(result['source'])+
                             '\n'+('告警诊断坐标 / Diagnostic coordinates: '+result.get('quality','') if diagnostic
                                     else '当前测量 / Current measurement')+
                             '\nToolbox 同时独占 USB 时无法双端直播；导入同工具、同坐标系记录用于离线比较。')
        for i,row in enumerate(result['rows']):
            values=['—']*3 if row['xyz'] is None else [f'{v:.3f}' for v in row['xyz']]
            for j,value in enumerate([row['space']]+values):
                self.tableCell(self.liveTable,i,j,value)
        tracked=result.get('trackedTools',[])
        self.allToolTable.setRowCount(len(tracked))
        for i,item in enumerate(tracked):
            position=item['trackerXYZ']
            xyz='—' if position is None else ', '.join(f'{v:.3f}' for v in position)
            for j,value in enumerate((item['role'],item['name'],xyz,item['frame'] or '—',item['status'])):
                self.tableCell(self.allToolTable,i,j,value)
        self.liveMarkers.setRowCount(max(4,len(result['markers'])))
        for i in range(self.liveMarkers.rowCount):
            marker=result['markers'][i] if i<len(result['markers']) else {}
            values=[marker.get('name','M'+str(i+1))]
            for key in ('trackerExpected','trackerMeasured'):
                p=marker.get(key)
                values.append('—' if p is None else ', '.join(f'{v:.3f}' for v in p))
            for j,value in enumerate(values):
                self.tableCell(self.liveMarkers,i,j,value)
        samples=result.get('samples',{})
        def number(v):
            return '—' if v is None else f'{v:.4f}'
        labels={'Tool':'Tool 平均误差 / MeanError','Reference':'Reference 平均误差 / MeanError'}
        lines=[]
        for role in ('Tool','Reference'):
            sample=samples.get(role)
            if sample:
                lines.append(labels[role]+'='+number(sample['meanErrorMm'])+' mm; '
                             +role+' RMS='+number(sample['rmsMm'])+' mm')
        lines+=['Slicer 球心变换残差 / marker residual RMS: '+number(result.get('slicerMarkerResidualRmsMm'))+' mm',
                'SDK 针尖变换一致性 / tip consistency: '+number(result.get('sdkTipConsistencyMm'))+' mm',
                '拟合误差不是绝对定位精度 / Fit residual is not absolute positioning accuracy.']
        self.metricsLabel.text='\n'.join(lines)
        if self.baseline:
            s=next((s for s in samples.values() if s['name']==self.baseline['toolName']),None)
            self.comparisonLabel.text=('Toolbox [外部记录 / external]: MeanError='+number(self.baseline.get('meanErrorMm'))+
                                       ' mm; Rms='+number(self.baseline.get('rmsMm'))+' mm; '+
                                       ('当前同名工具已匹配 / same name; 非同帧精度验证 / not synchronized accuracy validation'
                                        if s else '当前没有同名工具 / no matching tool'))

    def refreshStudio(self):
        if self.capture.process:
            self.captureLabel.text=f'录制 / Recording: {self.capture.frames} · 丢帧 / Dropped: {self.capture.dropped}'
        elif self.capture.error:
            self.captureLabel.text=self.capture.error
        elif hasattr(self.capture,'path'):
            self.captureLabel.text='已保存 / Saved: '+self.capture.path

    def exportCoordinates(self):
        path=qt.QFileDialog.getSaveFileName(slicer.util.mainWindow(),'导出 / Export',str(ROOT/'Logs/coordinates.json'),'JSON (*.json)')
        if path:
            exported=dict(self.lastCoordinateSnapshot)
            exported['panel']={'refreshHz':self.liveRate.value,'liveRefresh':self.liveCheck.checked,
                               'toolboxComparison':self.baseline}
            Path(path).write_text(json.dumps(exported,ensure_ascii=False,indent=2),encoding='utf-8')

    def importComparison(self):
        path=qt.QFileDialog.getOpenFileName(slicer.util.mainWindow(),'Toolbox 记录 / Record',str(ROOT/'Config'),'JSON (*.json)')
        if path:
            data=json.loads(Path(path).read_text(encoding='utf-8-sig'))
            if not isinstance(data.get('toolName'),str) or data.get('units')!='mm':
                raise ValueError('需要 toolName 和 units=mm / toolName and units=mm required')
            for key in ('meanErrorMm','rmsMm'):
                if not isinstance(data.get(key),(int,float)) or not math.isfinite(data[key]) or data[key]<0:
                    raise ValueError('Invalid '+key)
            self.baseline=data
            self.refreshCoordinates()

    def showHelp(self):
        dialog=qt.QDialog(slicer.util.mainWindow())
        dialog.setWindowTitle('使用说明 / Help · 0.8.2')
        layout=qt.QVBoxLayout(dialog)
        tabs=qt.QTabWidget()
        layout.addWidget(tabs)
        for title,name in [('操作说明 / Guide','USER_GUIDE.md'),('更新日志 / Changes','CHANGELOG.md')]:
            text=qt.QTextBrowser()
            path=ROOT/name
            content=path.read_text(encoding='utf-8') if path.is_file() else 'File missing: '+str(path)
            text.setSearchPaths([str(ROOT)])
            if hasattr(text,'setMarkdown'):
                text.setMarkdown(content)
            else:
                text.setPlainText(content)
            tabs.addTab(text,title)
        dialog.resize(900,650)
        self.helpDialog=dialog
        dialog.show()

    def orientCamera(self,reverse=False):
        manager=slicer.app.layoutManager()
        for index in range(manager.threeDViewCount):
            renderer=manager.threeDWidget(index).threeDView().renderWindow().GetRenderers().GetFirstRenderer()
            camera=renderer.GetActiveCamera()
            center=camera.GetFocalPoint()
            distance=max(camera.GetDistance(),1)
            camera.SetPosition(center[0],center[1],center[2]+(distance if reverse else -distance))
            # Display convention only; do not alter physical SDK coordinate axes.
            camera.SetViewUp(0,-1,0)
            renderer.ResetCamera()
            renderer.ResetCameraClippingRange()
        self.logic.log.info('Camera orientation changed; SDK and CAD transforms unchanged')

    def frontCamera(self):
        self.orientCamera(False)

    def reverseCamera(self):
        self.orientCamera(True)

    def reconstructSelected(self):
        node=self.volumeSelector.currentNode()
        self.logic.reconstructVolume(node)
        self.volumeLayout()
        qt.QTimer.singleShot(0,slicer.util.resetThreeDViews)

    def volumeLayout(self):
        self.logic.imageView=True
        self.logic.publish()
        super().volumeLayout()
        volume=self.volumeSelector.currentNode() if hasattr(self,'volumeSelector') else None
        if volume and volume.GetImageData():
            self.logic.reconstructVolume(volume)
            qt.QTimer.singleShot(0,slicer.util.resetThreeDViews)

    @staticmethod
    def pointSelector():
        selector=slicer.qMRMLNodeComboBox()
        selector.nodeTypes=['vtkMRMLMarkupsFiducialNode']
        selector.noneEnabled=True
        selector.addEnabled=False
        selector.removeEnabled=False
        selector.selectNodeUponCreation=False
        selector.setMRMLScene(slicer.mrmlScene)
        return selector

    def createPointLists(self):
        for selector,name in [(self.referencePoints,'PhysicalPoints_Reference'),(self.imagePoints,'ImagePoints_RAS')]:
            if selector.currentNode() is None:
                node=slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode',name)
                selector.setCurrentNode(node)

    def collectPoint(self):
        if not self.logic.ready() or self.logic.simulation:
            raise ValueError('需要有效的真实双工具跟踪 / Valid real dual-tool tracking required')
        snapshot=self.logic.liveCoordinates()
        sample=snapshot.get('samples',{}).get('Tool')
        if not sample or sample['tipTool'] is None or sample['ambiguous']:
            raise ValueError('SDK 针尖不可用 / SDK tip unavailable')
        self.createPointLists()
        point=snapshot['rows'][3]['xyz']
        if point is None:
            raise ValueError('参考坐标不可用 / Reference coordinate unavailable')
        if self.referencePoints.currentNode().GetTransformNodeID():
            raise ValueError('实物点列表必须在参考局部坐标中，不要挂载变换 / Physical points must have no parent transform')
        self.referencePoints.currentNode().AddControlPoint(vtk.vtkVector3d(*point))

    @staticmethod
    def points(node):
        if node is None:
            raise ValueError('选择点列表 / Select point list')
        result=[]
        for i in range(node.GetNumberOfControlPoints()):
            if node.GetNthControlPointPositionStatus(i)!=slicer.vtkMRMLMarkupsNode.PositionDefined:
                raise ValueError('存在未定义配准点 / Undefined landmark')
            point=[0.,0.,0.]
            node.GetNthControlPointPositionWorld(i,point)
            result.append(point)
        return result

    def registerSelected(self):
        rms,residual=self.logic.registerPoints(self.points(self.referencePoints.currentNode()),
            self.points(self.imagePoints.currentNode()),self.volumeSelector.currentNode(),
            self.registrationReferenceName() if hasattr(self,'registrationReferenceName') else self.reference.currentText)
        self.registrationLabel.text=f'FRE {rms:.3f} mm · 点误差 / residuals: '+', '.join(f'{v:.3f}' for v in residual)+'\nFRE 不代表靶点精度 / FRE is not target accuracy'

    def startImageDisplay(self):
        reg=self.logic.nodes.get('ReferenceToImage')
        if not reg or reg.GetAttribute('Aimooe.RegistrationValidated')!='True':
            raise ValueError('请先完成点配准 / Register points first')
        if not self.logic.ready():
            raise ValueError('没有有效双工具位姿 / No valid dual-tool poses')
        name=self.logic.tracker.selected.get('Reference') if self.logic.tracker else self.reference.currentText
        if reg.GetAttribute('Aimooe.ReferenceName')!=name:
            raise ValueError('参考工具已改变，请重新配准 / Reference changed; re-register')
        if reg.GetAttribute('Aimooe.VolumeID')!=self.volumeSelector.currentNodeID:
            raise ValueError('影像已改变，请重新配准 / Image changed; re-register')
        self.logic.imageNavigation=True
        self.logic.publish()
        self.volumeLayout()

    def stopImageDisplay(self):
        self.logic.imageNavigation=False
        self.logic.publish()

    def chooseCaptureDirectory(self):
        path=qt.QFileDialog.getExistingDirectory(slicer.util.mainWindow(),'保存目录 / Save directory',self.capturePath.text)
        if path:
            self.capturePath.text=path

    def selectCaptureRegion(self):
        if self.capture.process:
            raise ValueError('请先停止录制 / Stop recording first')
        self.regionOverlay=RegionOverlay(slicer.util.mainWindow(),lambda rect:setattr(self.capture,'region',rect))

    def clearCaptureRegion(self):
        if self.capture.process:
            raise ValueError('请先停止录制 / Stop recording first')
        self.capture.region=None

    def captureFile(self,suffix):
        return Path(self.capturePath.text)/(time.strftime('%Y%m%d-%H%M%S')+f'-{time.time_ns()%1000000:06d}.'+suffix.lower())

    def takeScreenshot(self):
        path=self.captureFile(self.imageFormat.currentText)
        self.capture.screenshot(path)
        self.captureLabel.text=str(path)

    def startRecording(self):
        self.capture.start(self.captureFile(self.videoFormat.currentText))

    def cleanupStudio(self):
        if hasattr(self,'liveTimer'):
            self.liveTimer.stop()
        if hasattr(self,'capture'):
            self.capture.stop()
        if getattr(self,'liveDialog',None):
            self.liveDialog.hide()

"""Import images without disconnecting tracking or switching active registration."""
import json
import time
import uuid
from pathlib import Path
import qt
import slicer
from .AimooeCoordinates import read_tool_definition
from .AimooePairs import profiles, read_pair

ROOT=Path(__file__).resolve().parents[2]


class ContinuityWidgetMixin:
    def setupContinuity(self):
        self.imageProcess=None
        self.importGeneration=0
        self.importedQueue=[]
        self.toolStatus=qt.QLabel('工具模型：请先选择导航工具与参考工具。')
        self.toolStatus.wordWrap=True
        self.importStatus=qt.QLabel('影像可在跟踪期间后台导入；当前导航影像不会自动切换。')
        self.importStatus.wordWrap=True
        panel=self.guidanceTabs.widget(0).layout()
        panel.addRow(self.importStatus)
        self.button(panel,'取消后台影像导入',self.cancelImageImport)
        self.button(panel,'清除当前影像 / 病例',self.clearCurrentCase)
        points=self.guidanceTabs.widget(1).layout()
        points.insertRow(0,self.toolStatus)
        self.button(points,'载入当前导航 / 参考工具模型',self.ensureToolModels)
        self.button(points,'查看定位工具（三维定位仪空间）',self.previewTrackingTools)
        self.button(points,'返回影像配准视图',self.returnToImage)
        self.importTimer=qt.QTimer()
        self.importTimer.setInterval(100)
        self.importTimer.connect('timeout()',self.attachNextImage)

    def ensureToolModels(self):
        single=hasattr(self,'singleToolMode') and self.singleToolMode()
        selected={'Tool':self.tool.currentText}
        if not single:
            selected['Reference']=self.reference.currentText
        if self.calibrationTool.currentText:
            selected['CalibrationTool']=self.calibrationTool.currentText
        found={role:Path(node.GetAttribute('Aimooe.Pair')) for node,role,name in self.logic.assetBindings
               if node.GetScene() and node.GetAttribute('Aimooe.Pair') and name==selected.get(role)}
        required={'Tool'} if single else {'Tool','Reference'}
        if not required.issubset(found):
            if single:
                matches=[path for path in profiles() if read_pair(path)['toolName']==self.tool.currentText]
                if len(matches)==1:
                    self.logic.loadPair(matches[0],'Tool');found['Tool']=matches[0]
            else:
                found.update(self.logic.loadSelectedPairs(self.tool.currentText,self.reference.currentText))
        calibration=selected.get('CalibrationTool')
        if calibration and 'CalibrationTool' not in found:
            matches=[path for path in profiles() if read_pair(path)['toolName']==calibration]
            if len(matches)==1:
                self.logic.loadPair(matches[0],'CalibrationTool')
                found['CalibrationTool']=matches[0]
            elif len(matches)>1:
                raise ValueError('标定工具存在多个 CAD 配对，请先建立唯一配对')
            else:
                definitions=[]
                for path in Path(self.path.text).glob('*.aimtool'):
                    try:
                        if read_tool_definition(path)['name']==calibration:
                            definitions.append(path)
                    except Exception:
                        continue
                if len(definitions)==1:
                    self.logic.createDefinitionModel(definitions[0],'CalibrationTool')
                    found['CalibrationTool']=definitions[0]
        missing=[role for role in selected if role not in found]
        source=self.logic.nodes['TipToTool'].GetAttribute('Aimooe.TipCalibration')
        self.toolStatus.text=('已加载：'+', '.join(found)+'；针尖来源：'+str(source))
        if missing:
            self.toolStatus.text+='；缺少模型绑定：'+', '.join(missing)+'。请提供与实际 aimtool 对应的 STP；仅有 aimtool 时只能显示标记球心。'
        else:
            self.toolStatus.text+='。未配准时在定位仪空间预览，点配准后自动显示到影像。'
        self.refreshSceneModels()
        return found

    def previewTrackingTools(self):
        self.ensureToolModels()
        self.logic.guidanceRequest=None
        self.logic.fourTrackingViews()
        self.logic.publish()
        self.logic.fitFourViews()
        self.toolStatus.text+=' 当前为定位仪空间，不代表影像已配准。'

    def returnToImage(self):
        self.volumeLayout()
        self.fitGuidanceView()

    def registerSelected(self):
        self.ensureToolModels()
        super().registerSelected()
        if self.logic.running:
            volume,reg=self.ensureRegistration()
            ids=[node.GetID() for node,_,_ in self.logic.assetBindings if node.GetScene() and
                 (node.GetAttribute('Aimooe.Pair') or node.GetAttribute('Aimooe.TipTarget'))]
            volume.SetAttribute('Aimooe.SelectedModels',json.dumps(ids))
            self.logic.imageView=True
            self.volumeLayout()
            self.logic.guidanceRequest=(volume.GetID(),reg.GetAttribute('Aimooe.ReferenceName'))
            self.logic.publish()
            qt.QTimer.singleShot(200,self.safeFitGuidanceView)
            self.toolStatus.text+=' 点配准完成；下一有效跟踪帧自动显示影像空间工具与针尖。'

    def startGuidance(self):
        super().startGuidance()
        self.fitGuidanceView()

    def safeFitGuidanceView(self):
        """Ignore a queued camera fit after the module widget has been destroyed."""
        try:
            if self.parent and self.volumeSelector:
                self.fitGuidanceView()
        except (RuntimeError, ValueError):
            return

    def importMedicalImage(self):
        path=qt.QFileDialog.getOpenFileName(slicer.util.mainWindow(),'后台导入医学影像',str(ROOT/'ScalarVolume_11'),
            '医学影像 (*.dcm *.nrrd *.nhdr *.nii *.nii.gz *.mha *.mhd *.hdr *.img);;所有文件 (*)')
        if path:
            self.beginImageImport(str(Path(path).parent) if Path(path).suffix.lower()=='.dcm' else path)

    def importDicomFolder(self):
        path=qt.QFileDialog.getExistingDirectory(slicer.util.mainWindow(),'后台导入 DICOM 目录',str(ROOT/'ScalarVolume_11'))
        if path:
            self.beginImageImport(path)

    def loadDicomFolder(self,path):
        self.beginImageImport(path)

    def beginImageImport(self,path):
        if self.imageProcess or self.importedQueue:
            raise ValueError('已有影像导入任务，请等待或取消')
        path=Path(path)
        if not path.exists():
            raise FileNotFoundError(str(path))
        folder=ROOT/'Logs/ImageImports'/uuid.uuid4().hex
        folder.mkdir(parents=True)
        self.imageJob=folder/'job.json'
        self.imageJob.write_text(json.dumps(dict(path=str(path))),encoding='utf-8')
        self.importOriginalID=self.volumeSelector.currentNodeID
        self.importAdded=[]
        process=qt.QProcess()
        self.imageProcess=process
        process.setStandardOutputFile(str(folder/'stdout.txt'))
        process.setStandardErrorFile(str(folder/'stderr.txt'))
        process.connect('finished(int,QProcess::ExitStatus)',self.imageImportFinished)
        process.connect('errorOccurred(QProcess::ProcessError)',self.imageImportFailed)
        process.start(str(Path(slicer.app.slicerHome)/'bin/PythonSlicer.exe'),
                      [str(ROOT/'AimooeNavigation/AimooeLib/AimooeImageImport.py'),str(self.imageJob)])
        self.importStatus.text='后台读取影像中；设备采集与当前导航继续。'

    def imageImportFailed(self,error):
        self.imageProcess=None
        self.importStatus.text='影像读取进程失败：'+str(error)
        self.logic.log.error(self.importStatus.text)

    def imageImportFinished(self,code,status):
        self.imageProcess=None
        try:
            result=json.loads(self.imageJob.with_suffix('.result.json').read_text(encoding='utf-8'))
            if code or not result['ok']:
                raise RuntimeError(result.get('error','后台读取失败'))
            self.importedQueue=list(result['volumes'])
            self.importTimer.start()
            self.importStatus.text='后台读取完成，正在加入影像列表；不切换当前导航。'
        except Exception as exc:
            self.importStatus.text=str(exc)
            self.logic.log.exception('Background image import failed')

    def attachNextImage(self):
        if not self.importedQueue:
            self.importTimer.stop()
            return
        item=self.importedQueue.pop(0)
        current=self.volumeSelector.currentNode()
        blocked=self.volumeSelector.blockSignals(True)
        oldAuto=self.volumeSelector.selectNodeUponCreation
        self.volumeSelector.selectNodeUponCreation=False
        start=time.monotonic()
        try:
            # No slice-layer changes, no rendering preset, no registration reset.
            volume=slicer.util.loadVolume(item['path'],{'name':item['name'],'show':False})
            if not volume:
                raise RuntimeError('Slicer 未能载入转换后的影像')
            volume.SetAttribute('Aimooe.Registration','UNREGISTERED')
            volume.SetAttribute('Aimooe.Attached','False')
            self.logic.configureVolumeDisplay(volume)
            self.importAdded.append(volume.GetID())
            self.volumeSelector.setCurrentNode(current)
        except Exception as exc:
            self.importedQueue=[]
            self.importTimer.stop()
            self.importStatus.text='导入失败：'+str(exc)
            self.logic.log.exception('Attach image failed')
            return
        finally:
            self.volumeSelector.selectNodeUponCreation=oldAuto
            self.volumeSelector.blockSignals(blocked)
            self.lastImageAttachSeconds=time.monotonic()-start
        if not self.importedQueue:
            self.importTimer.stop()
            self.importStatus.text='已加入 %d 组影像；当前跟踪与导航影像保持不变。选择新影像后需使用该影像自己的配准。'%len(self.importAdded)
            if not self.logic.running and current is None:
                self.volumeSelector.setCurrentNodeID(self.importAdded[0])

    def cancelImageImport(self):
        self.importGeneration+=1
        if hasattr(self,'importTimer'):
            self.importTimer.stop()
        self.importedQueue=[]
        if self.imageProcess:
            self.imageProcess.disconnect('finished(int,QProcess::ExitStatus)',self.imageImportFinished)
            self.imageProcess.disconnect('errorOccurred(QProcess::ProcessError)',self.imageImportFailed)
            self.imageProcess.kill()
            self.imageProcess=None
        if hasattr(self,'importStatus'):
            self.importStatus.text='后台导入已取消；已加入场景的影像保留。'

    @staticmethod
    def _caseReferencedNodeIDs(volume):
        """Return explicit per-volume node references; never infer ownership by name."""
        keys=('Aimooe.NativeSegmentation','Aimooe.SurfaceID','Aimooe.PhysicalPoints',
              'Aimooe.ImagePoints','Aimooe.SurfaceSamples','Aimooe.NavigationTarget',
              'Aimooe.NavigationPath','Aimooe.NavigationRoute','Aimooe.AccuracyTarget_POINT',
              'Aimooe.AccuracyTarget_SURFACE','Aimooe.PrecisionVisualization_POINT',
              'Aimooe.PrecisionLandmarks_POINT','Aimooe.PrecisionColor_POINT',
              'Aimooe.PrecisionVisualization_SURFACE','Aimooe.PrecisionLandmarks_SURFACE',
              'Aimooe.PrecisionColor_SURFACE')
        return {volume.GetAttribute(key) for key in keys if volume.GetAttribute(key)}

    def clearCurrentCase(self):
        """Remove the selected image workspace while preserving live hardware state."""
        volume=self.volumeSelector.currentNode()
        if volume is None:
            raise ValueError('当前没有可清除的影像 / No current image case')
        volumeID=volume.GetID()

        # Stop only workflows that hold references to this case. Tracking and the
        # SDK process deliberately remain untouched.
        if hasattr(self,'cleanupNavigationPresets'):
            self.cleanupNavigationPresets()
        self.endPlacement()
        self.stopSurfaceCollection()
        self.cancelSurface()
        self.stopImageDisplay()
        self.logic.guidanceRequest=None

        owned={volumeID}|self._caseReferencedNodeIDs(volume)
        for index in range(slicer.mrmlScene.GetNumberOfNodes()):
            node=slicer.mrmlScene.GetNthNode(index)
            if (node.GetAttribute('Aimooe.ImageOwner')==volumeID or
                    node.GetAttribute('Aimooe.SourceVolume')==volumeID or
                    node.GetAttribute('Aimooe.VolumeID')==volumeID):
                owned.add(node.GetID())
            if node.IsA('vtkMRMLVolumeRenderingDisplayNode'):
                source=node.GetVolumeNode()
                if source and source.GetID()==volumeID:
                    owned.add(node.GetID())

        # Display/storage nodes are implementation children of owned case nodes.
        # Add them by exact MRML references before deleting their parents.
        for identity in list(owned):
            node=slicer.mrmlScene.GetNodeByID(identity)
            if node is None:
                continue
            if hasattr(node,'GetNumberOfDisplayNodes'):
                for index in range(node.GetNumberOfDisplayNodes()):
                    child=node.GetNthDisplayNode(index)
                    if child:
                        owned.add(child.GetID())
            if hasattr(node,'GetStorageNode'):
                child=node.GetStorageNode()
                if child:
                    owned.add(child.GetID())

        for selectorName in ('imagePoints','referencePoints','surfaceSelector','volumeSelector'):
            selector=getattr(self,selectorName,None)
            if selector and selector.currentNodeID in owned:
                selector.setCurrentNode(None)
        slicer.util.setSliceViewerLayers(background=None,foreground=None,label=None)

        # Remove children before data nodes. MRML also clears incoming references.
        removed=[]
        for identity in list(owned):
            node=slicer.mrmlScene.GetNodeByID(identity)
            if node and node is not volume:
                removed.append(node.GetName())
                slicer.mrmlScene.RemoveNode(node)
        if volume.GetScene():
            removed.append(volume.GetName())
            slicer.mrmlScene.RemoveNode(volume)

        for key,node in list(self.logic.nodes.items()):
            if node is None or node.GetScene() is None or node.GetID() in owned:
                self.logic.nodes.pop(key,None)
        self.logic.activeVolumeID=None
        self.logic.imageView=False
        self.logic.imageNavigation=False
        self.logic.applyWorkspaceVisibility()
        self.refreshSceneModels()
        self.importStatus.text='已清除当前病例及其 %d 个关联节点；设备连接与工具模型保留。'%max(len(removed)-1,0)
        self.logic.log.info('Cleared image case %s; removed nodes=%s',volumeID,removed)
        return removed

    def cleanupStudio(self):
        if hasattr(self,'cleanupNavigationPresets'):
            self.cleanupNavigationPresets()
        if hasattr(self,'cleanupCalibration'):
            self.cleanupCalibration()
        if hasattr(self,'imageProcess'):
            self.cancelImageImport()
        super().cleanupStudio()

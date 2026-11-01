"""0.3.4 independent image workspaces and explicit tool / tip selection."""
import hashlib
import json
import re
import time
import uuid
from pathlib import Path
import qt
import vtk
import slicer
from .AimooeModels import sphere, matrix_from_rows
from .AimooePairs import profiles, read_pair
from .AimooeCoordinates import transform_point, read_tool_definition
from .AimooePaths import SDK_SOURCE, cad_python

ROOT=Path(__file__).resolve().parents[2]
SOURCE=SDK_SOURCE


def discover_pairs():
    """Names only shortlist candidates; cached hashes and four-sphere fit verify them."""
    definitions=list((SOURCE/'Aimtoolbox_release-v3.1.2e/Config/AimTools').glob('*.aimtool'))
    output=[]
    for step in sorted((SOURCE/'tools').glob('*.st*p')):
        match=re.search(r'B4[A-Z][0-9]',step.name.upper())
        if not match:
            continue
        for definition in definitions:
            data=read_tool_definition(definition)
            if match[0] not in data['name'].upper():
                continue
            verified=None
            sh=hashlib.sha256(step.read_bytes()).hexdigest()
            dh=hashlib.sha256(definition.read_bytes()).hexdigest()
            for path in profiles():
                cached=read_pair(path)
                if cached.get('stepSHA256')==sh and cached.get('toolSHA256')==dh:
                    verified=str(path)
                    break
            output.append({'step':str(step),'definition':str(definition),
                           'name':data['name'],'verified':verified})
    return output


class WorkspaceLogicMixin:
    def showAssetGroup(self, groupId):
        super().showAssetGroup(groupId)
        volumes=[node for node in slicer.util.getNodesByClass('vtkMRMLScalarVolumeNode') if node.GetAttribute('Aimooe.GroupId')==groupId]
        self.independentSelection=set()
        self.catalogPreview=False
        self.activeStandaloneGroupID=groupId if not volumes else None
        self.imageView=bool(volumes)
        self.setActiveVolume(volumes[0] if volumes else None)

    def invalidateRegistration(self):
        super().invalidateRegistration()
        for volume in slicer.util.getNodesByClass('vtkMRMLScalarVolumeNode'):
            volume.SetAttribute('Aimooe.SavedRegistration',None)

    def diagnosticPair(self):
        if not self.running or not self.tracker:
            return None
        samples=getattr(self.tracker,'samples',{})
        pair=[samples.get(role) for role in ('Tool','Reference')]
        if any(not sample or sample['ambiguous'] or time.monotonic()-sample['monotonic']>=.5 for sample in pair):
            return None
        if pair[0]['frame']!=pair[1]['frame']:
            return None
        indices=[set(sample['indices'])-{-1} for sample in pair]
        if any(len(index)<4 for index in indices) or indices[0]&indices[1]:
            return None
        return pair

    def updateDisplayFrame(self):
        if getattr(self,'imageNavigation',False) and not self.ready():
            pair=self.diagnosticPair()
            registration=self.nodes.get('ReferenceToImage')
            if pair and registration and registration.GetAttribute('Aimooe.RegistrationValidated')=='True':
                reference=matrix_from_rows(pair[1]['matrix'])
                inverse=vtk.vtkMatrix4x4()
                vtk.vtkMatrix4x4.Invert(reference,inverse)
                reg=vtk.vtkMatrix4x4()
                registration.GetMatrixTransformToParent(reg)
                result=vtk.vtkMatrix4x4()
                vtk.vtkMatrix4x4.Multiply4x4(reg,inverse,result)
                self.nodes['TrackerToDisplay'].SetMatrixTransformToParent(result)
                self.nodes['TrackerToDisplay'].SetAttribute('Aimooe.Valid','False')
                self.nodes['TrackerToDisplay'].SetAttribute('Aimooe.DisplayFrame','Image_Diagnostic')
                return
        super().updateDisplayFrame()

    def setActiveVolume(self, volume):
        newID=volume.GetID() if volume else None
        if newID!=getattr(self,'activeVolumeID',None):
            self.imageNavigation=False
            super().invalidateRegistration()
            self.activeVolumeID=newID
            saved=volume.GetAttribute('Aimooe.SavedRegistration') if volume else None
            if saved:
                saved=json.loads(saved)
                node=self.node('ReferenceToImage')
                node.SetMatrixTransformToParent(matrix_from_rows(saved['matrix']))
                for key,value in saved['attributes'].items():
                    node.SetAttribute(key,value)
        slicer.util.setSliceViewerLayers(background=volume,foreground=None,label=None)
        if volume:
            slicer.util.resetSliceViews()
        self.applyWorkspaceVisibility()

    def applyWorkspaceVisibility(self):
        imageView=getattr(self,'imageView',False)
        identity=getattr(self,'activeVolumeID',None)
        volume=slicer.mrmlScene.GetNodeByID(identity or '')
        group=volume.GetAttribute('Aimooe.GroupId') if volume else None
        allowed=set(json.loads(volume.GetAttribute('Aimooe.SelectedModels') or '[]')) if volume else set()
        for display in slicer.util.getNodesByClass('vtkMRMLVolumeRenderingDisplayNode'):
            display.SetVisibility(bool(imageView and volume and volume.GetAttribute('Aimooe.RenderMode')=='volume' and display.GetVolumeNode() and display.GetVolumeNode().GetID()==identity))
        for model in slicer.util.getNodesByClass('vtkMRMLModelNode'):
            source=model.GetAttribute('Aimooe.SourceVolume')
            display=model.GetDisplayNode()
            if not display:
                continue
            if source:
                display.SetVisibility(bool(imageView and source==identity and volume and volume.GetAttribute('Aimooe.RenderMode')!='volume'))
            elif model.GetAttribute('Aimooe.Pair') or model.GetAttribute('Aimooe.TipTarget'):
                if not imageView and getattr(self,'activeStandaloneGroupID',None):
                    display.SetVisibility(False)
                if imageView and (not getattr(self,'imageNavigation',False) or model.GetID() not in allowed):
                    display.SetVisibility(False)
            elif model.GetAttribute('Aimooe.SourceFile') or model.GetAttribute('Aimooe.GroupId'):
                visible=(model.GetID() in allowed or bool(group and model.GetAttribute('Aimooe.GroupId')==group)) if imageView else (model.GetID() in getattr(self,'independentSelection',set()) or bool(getattr(self,'activeStandaloneGroupID',None) and model.GetAttribute('Aimooe.GroupId')==self.activeStandaloneGroupID))
                # A bound asset requires a valid role pose as well as selection.
                if model.GetAttribute('Aimooe.BoundTool'):
                    visible=visible and bool(getattr(self,'imageNavigation',False)) and bool(display.GetVisibility())
                display.SetVisibility(visible)
        for node in slicer.util.getNodesByClass('vtkMRMLMarkupsFiducialNode'):
            owner=node.GetAttribute('Aimooe.ImageOwner')
            if owner and node.GetDisplayNode() and node is not self.nodes.get('ImageTip'):
                node.GetDisplayNode().SetVisibility(bool(imageView and owner==identity and node.GetAttribute('Aimooe.PhysicalInput')!='True'))
        for node in slicer.util.getNodesByClass('vtkMRMLSegmentationNode'):
            if node.GetAttribute('Aimooe.SourceFile') and node.GetDisplayNode():
                node.GetDisplayNode().SetVisibility(bool(imageView and
                    (node.GetID() in allowed or (group and node.GetAttribute('Aimooe.GroupId')==group))))

    def updateAssetVisibility(self):
        super().updateAssetVisibility()
        if getattr(self,'catalogPreview',False) and not self.running and not getattr(self,'imageView',False):
            for model,_,_ in self.assetBindings:
                if model.GetScene() and model.GetAttribute('Aimooe.Pair'):
                    model.GetDisplayNode().SetVisibility(True)
        self.applyWorkspaceVisibility()

    def loadSelectedPairs(self, tool, reference):
        if tool==reference:
            raise ValueError('导航工具和参考工具必须不同 / Select two different tools')
        for node,_,_ in self.assetBindings:
            if node.GetAttribute('Aimooe.Pair') or node.GetAttribute('Aimooe.TipTarget'):
                node.GetDisplayNode().SetVisibility(False)
        self.assetBindings=[entry for entry in self.assetBindings if not
                            (entry[0].GetAttribute('Aimooe.Pair') or entry[0].GetAttribute('Aimooe.TipTarget'))]
        found=super().loadSelectedPairs(tool,reference)
        if self.pairToolProfile and 'LTS' in Path(self.pairToolProfile.get('sourceStep','')).name.upper():
            tip=self.node('LTS_TipTarget','vtkMRMLModelNode')
            # This profile defines TipToTool as the CAD tip sphere centre
            # (tipSource CAD_SPHERE_CENTER_*), so radius 1 mm is the original
            # CAD tip ball size; the oversized 2.5 mm red ball is removed.
            tip.SetAndObservePolyData(sphere([0,0,0],1.0))
            tip.CreateDefaultDisplayNodes()
            tip.GetDisplayNode().SetColor(1,0,0)
            tip.SetAndObserveTransformNodeID(self.nodes['TipToTool'].GetID())
            tip.SetAttribute('Aimooe.TipTarget','True')
            tip.SetAttribute('Aimooe.Pair',str(found['Tool']))
            tip.SetAttribute('Aimooe.LiveParent',tip.GetTransformNodeID())
            tip.SetAttribute('Aimooe.LiveColor','[1,0,0]')
            tip.SetAttribute('Aimooe.BoundTool',tool)
            self.assetBindings.append((tip,'Tool',tool))
            self.applyChosenTip()
        self.updateAssetVisibility()
        return found

    def applyChosenTip(self):
        if getattr(self,'tipMode','CAD')=='SDK':
            super().applySdkTip()
            return
        profile=self.pairToolProfile
        if not profile or 'nominalTipTool' not in profile:
            return
        node=self.nodes['TipToTool']
        if node.GetAttribute('Aimooe.TipCalibration')=='CALIBRATED':
            return
        matrix=vtk.vtkMatrix4x4()
        for i,value in enumerate(profile['nominalTipTool']):
            matrix.SetElement(i,3,value)
        old=vtk.vtkMatrix4x4()
        node.GetMatrixTransformToParent(old)
        if any(old.GetElement(i,j)!=matrix.GetElement(i,j) for i in range(4) for j in range(4)):
            node.SetMatrixTransformToParent(matrix)
        self.nodes['TipToTool'].SetAttribute('Aimooe.TipCalibration','CAD_SPHERE_CENTER_NOMINAL_UNCALIBRATED')

    def applySdkTip(self):
        self.applyChosenTip()

    def liveCoordinates(self):
        result=super().liveCoordinates()
        result['tipMode']=getattr(self,'tipMode','CAD')
        result['definitions']={'Tip':'Needle target local frame; origin (0,0,0)',
                               'Tool':'Navigation tool local frame (LTS)',
                               'Tracker':'Optical system frame','Reference':'Reference tool local frame (RFS)'}
        return result

    def fourTrackingViews(self):
        super().fourTrackingViews()
        qt.QTimer.singleShot(0,self.fitFourViews)

    def fitFourViews(self):
        super().fitFourViews()
        manager=slicer.app.layoutManager()
        if manager.threeDViewCount:
            renderer=manager.threeDWidget(0).threeDView().renderWindow().GetRenderers().GetFirstRenderer()
            camera=renderer.GetActiveCamera()
            center=camera.GetFocalPoint()
            distance=max(camera.GetDistance(),100.)
            camera.SetPosition(center[0],center[1],center[2]-distance)
            camera.SetViewUp(0,-1,0)
            camera.ParallelProjectionOn()
            renderer.ResetCameraClippingRange()
        bounds=[float('inf'),float('-inf')]*3
        for model,_,_ in self.assetBindings:
            if not model.GetScene() or not model.GetDisplayNode().GetVisibility():
                continue
            current=[0.]*6
            model.GetRASBounds(current)
            for axis in range(3):
                bounds[2*axis]=min(bounds[2*axis],current[2*axis])
                bounds[2*axis+1]=max(bounds[2*axis+1],current[2*axis+1])
        if bounds[0]<=bounds[1]:
            for index in range(manager.threeDViewCount):
                renderer=manager.threeDWidget(index).threeDView().renderWindow().GetRenderers().GetFirstRenderer()
                renderer.ResetCamera(bounds)
                camera=renderer.GetActiveCamera()
                if camera.GetParallelProjection():
                    # Conservative sphere fit also covers edge-on tools and tall viewports.
                    half=[(bounds[2*i+1]-bounds[2*i])*.5 for i in range(3)]
                    width,height=renderer.GetSize()
                    radius=sum(value*value for value in half)**.5
                    camera.SetParallelScale(max(radius,1.)*1.15/max(min(max(width,1)/max(height,1),1.),.01))
                renderer.ResetCameraClippingRange(bounds)

    def reconstructVolume(self, volume, threshold=None):
        # Distinguish binary masks from normalized floating-point MRI.
        if volume and volume.GetImageData() and threshold is None:
            image=volume.GetImageData()
            if image.GetScalarType() in (vtk.VTK_FLOAT,vtk.VTK_DOUBLE):
                display=slicer.modules.volumerendering.logic().CreateDefaultVolumeRenderingNodes(volume)
                if not volume.GetAttribute('Aimooe.RenderPreset'):
                    slicer.modules.volumerendering.logic().UpdateDisplayNodeFromVolumeNode(display,volume)
                display.SetVisibility(True)
                volume.SetAttribute('Aimooe.RenderMode','volume')
                self.applyWorkspaceVisibility()
                return display
        result=super().reconstructVolume(volume,threshold)
        volume.SetAttribute('Aimooe.RenderMode','surface' if result.IsA('vtkMRMLModelNode') else 'volume')
        if result.IsA('vtkMRMLModelNode'):
            result.GetDisplayNode().SetBackfaceCulling(False)
            result.GetDisplayNode().SetInterpolation(1)
            # Smooth normals, not vertex coordinates: retain measured image geometry.
            if result.GetAttribute('Aimooe.NormalsKey')!=result.GetAttribute('Aimooe.SurfaceKey'):
                normals=vtk.vtkPolyDataNormals()
                normals.SetInputData(result.GetPolyData())
                normals.SplittingOff()
                normals.ConsistencyOn()
                normals.Update()
                mesh=vtk.vtkPolyData()
                mesh.DeepCopy(normals.GetOutput())
                result.SetAndObservePolyData(mesh)
                result.SetAttribute('Aimooe.NormalsKey',result.GetAttribute('Aimooe.SurfaceKey'))
        self.applyWorkspaceVisibility()
        return result

    def registerPoints(self, source, target, volume, referenceName):
        result=super().registerPoints(source,target,volume,referenceName)
        node=self.nodes['ReferenceToImage']
        matrix=vtk.vtkMatrix4x4()
        node.GetMatrixTransformToParent(matrix)
        attrs={key:node.GetAttribute(key) for key in ['Aimooe.RegistrationValidated','Aimooe.ReferenceName','Aimooe.VolumeID','Aimooe.FREmm']}
        volume.SetAttribute('Aimooe.SavedRegistration',json.dumps({'matrix':[[matrix.GetElement(i,j) for j in range(4)] for i in range(4)],'attributes':attrs}))
        return result


class WorkspaceWidgetMixin:
    def setupWorkspace(self):
        self.workspaceBusy=False
        self.recordFeedbackTimer=qt.QTimer()
        self.recordFeedbackTimer.setInterval(250)
        self.recordFeedbackTimer.connect('timeout()',self.updateRecordingFeedback)
        self.logic.tipMode='CAD'
        self.volumeSelector.disconnect('currentNodeChanged(vtkMRMLNode*)')
        self.volumeSelector.connect('currentNodeChanged(vtkMRMLNode*)',self.selectWorkspace)
        form=self.section('影像工作区 · 0.3.4')
        self.layout.insertWidget(1,form.parentWidget())
        form.addRow(qt.QLabel('每次显示一组影像；器械必须明确选择并配准后叠加。'))
        self.sceneModels=qt.QListWidget()
        self.sceneModels.setMaximumHeight(140)
        form.addRow('已导入模型（可多选）',self.sceneModels)
        self.button(form,'刷新模型列表',self.refreshSceneModels)
        self.button(form,'应用本影像模型选择',self.selectModels)
        self.button(form,'独立查看选中模型',self.previewSelectedModels)
        form.addRow('工具配对（可多选）',self.pairList)
        form.addRow(self.pairStatus)
        self.button(form,'扫描默认 STP / aimtool',self.scanDefaultPairs)
        self.button(form,'转换并验证扫描配对',self.convertScannedPairs)
        self.button(form,'创建选中独立工具集',self.activateSelectedSet)
        self.button(form,'同时展示导航 / 参考工具',self.showPairWorkspace)
        self.tipModeCombo=qt.QComboBox()
        self.tipModeCombo.addItems(['LTS 模型针尖靶点（CAD 名义值）','工具文件针尖（SDK，需实物标定）'])
        self.tipModeCombo.connect('currentIndexChanged(int)',self.changeTipMode)
        form.addRow('Tip 来源',self.tipModeCombo)
        self.workspaceStatus=qt.QLabel('准备就绪。Tip 的 (0,0,0) 与 Tool 中的针尖偏移是同一点在不同坐标系中的表示。')
        self.workspaceStatus.wordWrap=True
        form.addRow(self.workspaceStatus)
        self.threshold=qt.QDoubleSpinBox()
        self.threshold.setRange(-100000,100000)
        self.threshold.setValue(.5)
        form.addRow('显式表面阈值',self.threshold)
        self.button(form,'按阈值提取表面',self.thresholdSurface)
        self.renderPreset=qt.QComboBox()
        presets=slicer.modules.volumerendering.logic().GetPresetsScene()
        self.renderPreset.addItems([presets.GetNthNodeByClass(i,'vtkMRMLVolumePropertyNode').GetName() for i in range(presets.GetNumberOfNodesByClass('vtkMRMLVolumePropertyNode'))])
        form.addRow('CT / MRI 体绘制预设',self.renderPreset)
        self.button(form,'应用体绘制预设',self.applyRenderPreset)
        self.button(form,'打开标准体绘制设置',lambda:slicer.util.selectModule('VolumeRendering'))
        self.operationStatus=qt.QLabel('截图 / 录屏：待操作')
        self.operationStatus.wordWrap=True
        form.addRow(self.operationStatus)
        self.refreshSceneModels()
        self.scanDefaultPairs()
        self.registerWorkspaceTranslations()

    def selectWorkspace(self, volume):
        if self.workspaceBusy:
            return
        self.workspaceBusy=True
        try:
            self.logic.setActiveVolume(volume)
            if volume:
                self.logic.imageView=True
                self.volumeLayout()
            self.refreshSceneModels()
            if volume:
                self.referencePoints.setCurrentNodeID(volume.GetAttribute('Aimooe.PhysicalPoints') or '')
                self.imagePoints.setCurrentNodeID(volume.GetAttribute('Aimooe.ImagePoints') or '')
        except Exception as exc:
            self.logic.log.exception('Image workspace switch failed')
            self.workspaceStatus.text=str(exc)
        finally:
            self.workspaceBusy=False

    def volumeLayout(self):
        volume=self.volumeSelector.currentNode()
        self.logic.setActiveVolume(volume)
        super().volumeLayout()
        self.logic.applyWorkspaceVisibility()

    def refreshSceneModels(self):
        self.sceneModelIDs=[]
        self.sceneModels.clear()
        volume=self.volumeSelector.currentNode()
        selected=set(json.loads(volume.GetAttribute('Aimooe.SelectedModels') or '[]')) if volume else set()
        for node in slicer.util.getNodesByClass('vtkMRMLModelNode'):
            if not (node.GetAttribute('Aimooe.Pair') or node.GetAttribute('Aimooe.SourceFile') or node.GetAttribute('Aimooe.TipTarget')):
                continue
            self.sceneModelIDs.append(node.GetID())
            item=qt.QListWidgetItem(node.GetName()+' · '+(node.GetAttribute('Aimooe.BoundTool') or '独立模型'))
            item.setFlags(item.flags() | qt.Qt.ItemIsUserCheckable)
            item.setCheckState(qt.Qt.Checked if node.GetID() in selected else qt.Qt.Unchecked)
            self.sceneModels.addItem(item)

    def checkedModelIDs(self):
        return [identity for i,identity in enumerate(self.sceneModelIDs) if self.sceneModels.item(i).checkState()==qt.Qt.Checked]

    def selectModels(self):
        volume=self.volumeSelector.currentNode()
        if not volume:
            raise ValueError('先选择影像；独立模型可用“独立查看”')
        volume.SetAttribute('Aimooe.SelectedModels',json.dumps(self.checkedModelIDs()))
        self.logic.publish()
        self.logic.applyWorkspaceVisibility()
        self.workspaceStatus.text='已保存本影像的模型选择；工具叠加需要该影像的配准。'

    def previewSelectedModels(self):
        identities=set(self.checkedModelIDs())
        self.logic.independentSelection=identities
        self.logic.fourTrackingViews()
        for node in slicer.util.getNodesByClass('vtkMRMLModelNode'):
            if node.GetDisplayNode() and (node.GetAttribute('Aimooe.SourceFile') or node.GetAttribute('Aimooe.Pair')):
                node.GetDisplayNode().SetVisibility(node.GetID() in identities)
        self.logic.fitFourViews()

    def changeTipMode(self, index):
        self.logic.tipMode='CAD' if index==0 else 'SDK'
        self.logic.applyChosenTip()
        self.logic.invalidateRegistration()
        self.workspaceStatus.text='已切换 Tip 来源；旧配准已失效，请核对针尖后重新配准。'

    def scanDefaultPairs(self):
        self.scannedPairs=discover_pairs()
        self.reloadPairs()
        self.workspaceStatus.text='找到 %d 个候选配对，%d 个已通过几何验证。文件名只用于筛选。'%(len(self.scannedPairs),sum(bool(p['verified']) for p in self.scannedPairs))
        selected={p['verified'] for p in self.scannedPairs if p['verified']}
        for i,path in enumerate(self.pairPaths):
            self.pairList.item(i).setSelected(str(path) in selected)

    def convertScannedPairs(self):
        if self.logic.running or self.pairProcess:
            raise ValueError('请先停止跟踪或等待转换完成')
        pairs=[p for p in self.scannedPairs if not p['verified']]
        if not pairs:
            self.workspaceStatus.text='当前候选已验证，可直接创建选中工具集。'
            return
        self.pairJob=ROOT/'Logs'/('auto-pair-'+uuid.uuid4().hex+'.json')
        self.pairJob.write_text(json.dumps(pairs,ensure_ascii=False),encoding='utf-8')
        process=qt.QProcess()
        self.pairProcess=process
        process.setStandardOutputFile(str(self.pairJob.with_suffix('.stdout.txt')))
        process.setStandardErrorFile(str(self.pairJob.with_suffix('.stderr.txt')))
        process.connect('finished(int,QProcess::ExitStatus)',self.pairsFinished)
        process.connect('errorOccurred(QProcess::ProcessError)',self.pairsFailed)
        process.start(str(cad_python()),
                      [str(ROOT/'scripts/build_pairs.py'),str(self.pairJob)])
        self.workspaceStatus.text='正在后台验证四球几何，请查看配对结果。'

    def activateSelectedSet(self):
        self.activatePairs()
        # Prefer the LTS probe as Tool, never choose roles from list order alone.
        for i in range(self.tool.count):
            if 'LTS' in self.tool.itemText(i).upper():
                self.tool.setCurrentIndex(i)
        for i in range(self.reference.count):
            if 'B4C0' in self.reference.itemText(i).upper():
                self.reference.setCurrentIndex(i)
        self.partialCheck.checked=False
        self.saveSettings()
        self.workspaceStatus.text='独立工具集已创建，全匹配模式；请核对导航 / 参考角色。'

    def showPairWorkspace(self):
        self.logic.activeStandaloneGroupID=None
        self.logic.independentSelection=set()
        self.logic.fourTrackingViews()
        self.showPairs()
        self.logic.catalogPreview=True
        if not self.logic.running:
            for role,x in [('Tool',-100.),('Reference',180.)]:
                matrix=vtk.vtkMatrix4x4()
                matrix.SetElement(0,3,x)
                self.logic.nodes[role+'ToTracker'].SetMatrixTransformToParent(matrix)
            self.logic.publish()
            self.logic.fitFourViews()
            self.workspaceStatus.text='离线模型排列预览，不是定位仪实测位置。开始跟踪后使用实际位姿。'
        self.refreshSceneModels()

    def thresholdSurface(self):
        self.logic.reconstructVolume(self.volumeSelector.currentNode(),self.threshold.value)
        self.logic.applyWorkspaceVisibility()
        slicer.util.resetThreeDViews()

    def createPointLists(self):
        super().createPointLists()
        volume=self.volumeSelector.currentNode()
        if volume:
            for selector,key in [(self.referencePoints,'Aimooe.PhysicalPoints'),(self.imagePoints,'Aimooe.ImagePoints')]:
                node=selector.currentNode()
                volume.SetAttribute(key,node.GetID())
                node.SetAttribute('Aimooe.ImageOwner',volume.GetID())
            self.referencePoints.currentNode().SetAttribute('Aimooe.PhysicalInput','True')
            self.logic.applyWorkspaceVisibility()

    def collectPoint(self):
        if not self.logic.ready() and self.logic.diagnosticPair() is None:
            raise ValueError('需要当前帧可解算的双工具位姿；不能从历史模型采点')
        point=self.logic.liveCoordinates()['rows'][3]['xyz']
        if point is None:
            raise ValueError('参考坐标不可用，请设置参考工具')
        self.createPointLists()
        self.referencePoints.currentNode().AddControlPoint(vtk.vtkVector3d(*point))
        self.workspaceStatus.text='已采集选定针尖；告警数据与 CAD 名义针尖需独立精度验证。'

    def registerSelected(self):
        super().registerSelected()
        volume=self.volumeSelector.currentNode()
        for selector,key in [(self.referencePoints,'Aimooe.PhysicalPoints'),(self.imagePoints,'Aimooe.ImagePoints')]:
            volume.SetAttribute(key,selector.currentNode().GetID())
            selector.currentNode().SetAttribute('Aimooe.ImageOwner',volume.GetID())
        self.referencePoints.currentNode().SetAttribute('Aimooe.PhysicalInput','True')
        preview=self.logic.node('RegisteredPoints_'+volume.GetID(),'vtkMRMLMarkupsFiducialNode')
        preview.RemoveAllControlPoints()
        for point in self.points(self.referencePoints.currentNode()):
            preview.AddControlPoint(vtk.vtkVector3d(*point))
        for index in range(preview.GetNumberOfControlPoints()):
            preview.SetNthControlPointLabel(index,'P'+str(index+1))
        preview.GetDisplayNode().SetTextScale(.8)
        preview.SetAndObserveTransformNodeID(self.logic.nodes['ReferenceToImage'].GetID())
        preview.SetAttribute('Aimooe.ImageOwner',volume.GetID())
        self.logic.applyWorkspaceVisibility()
        self.workspaceStatus.text=self.registrationLabel.text

    def startImageDisplay(self):
        if self.logic.ready():
            return super().startImageDisplay()
        pair=self.logic.diagnosticPair()
        registration=self.logic.nodes.get('ReferenceToImage')
        volume=self.volumeSelector.currentNode()
        if not pair or not volume or not registration or registration.GetAttribute('Aimooe.RegistrationValidated')!='True':
            raise ValueError('需要当前双工具可解算位姿和本影像的配准；模型可见不等于位姿有效')
        if registration.GetAttribute('Aimooe.VolumeID')!=volume.GetID() or registration.GetAttribute('Aimooe.ReferenceName')!=pair[1]['name']:
            raise ValueError('配准对应的影像或参考工具不一致')
        self.logic.imageNavigation=True
        self.logic.publish()
        self.volumeLayout()
        self.workspaceStatus.text='工程诊断显示：允许告警帧；仍记录告警，不代表精度已验证。'

    def importAssetGroup(self):
        super().importAssetGroup()
        volume=slicer.mrmlScene.GetNodeByID(getattr(self.logic,'activeVolumeID',None) or '')
        if volume:
            self.volumeSelector.setCurrentNode(volume)
            self.selectWorkspace(volume)
        self.refreshSceneModels()

    def loadAssetGroup(self):
        super().loadAssetGroup()
        data=json.loads(self.groupFiles[self.groupCombo.currentIndex].read_text(encoding='utf-8-sig'))
        volumes=[node for node in slicer.util.getNodesByClass('vtkMRMLScalarVolumeNode') if node.GetAttribute('Aimooe.GroupId')==data['id']]
        if volumes:
            self.volumeSelector.setCurrentNode(volumes[0])
            self.selectWorkspace(volumes[0])

    def takeScreenshot(self):
        super().takeScreenshot()
        self.operationStatus.text='截图已保存：'+self.captureLabel.text
        slicer.util.showStatusMessage(self.operationStatus.text,5000)

    def startRecording(self):
        super().startRecording()
        self.recordFeedbackTimer.start()
        self.operationStatus.text='正在启动录制：'+self.capture.path
        self.captureLabel.text=self.operationStatus.text
        slicer.util.showStatusMessage(self.captureLabel.text,5000)


    def updateRecordingFeedback(self):
        if self.capture.process:
            state='正在保存' if self.capture.stopping else '正在录制'
            self.operationStatus.text=f'{state} · {self.capture.frames} 帧 · 丢弃 {self.capture.dropped} 帧\n{self.capture.path}'
        else:
            self.operationStatus.text=('录制失败：'+self.capture.error if self.capture.error else '录制已保存：'+getattr(self.capture,'path',''))
            self.recordFeedbackTimer.stop()

    def cleanupStudio(self):
        if hasattr(self,'recordFeedbackTimer'):
            self.recordFeedbackTimer.stop()
        super().cleanupStudio()

    def applyRenderPreset(self):
        volume=self.volumeSelector.currentNode()
        if not volume:
            raise ValueError('请先选择影像')
        logic=slicer.modules.volumerendering.logic()
        preset=logic.GetPresetByName(self.renderPreset.currentText)
        if preset is None:
            raise ValueError('体绘制预设不存在')
        display=logic.CreateDefaultVolumeRenderingNodes(volume)
        display.GetVolumePropertyNode().Copy(preset)
        volume.SetAttribute('Aimooe.RenderMode','volume')
        volume.SetAttribute('Aimooe.RenderPreset',self.renderPreset.currentText)
        self.logic.applyWorkspaceVisibility()
        self.workspaceStatus.text='已应用 '+self.renderPreset.currentText+'；预设只改变显示，不改变影像数据。'

    def registerWorkspaceTranslations(self):
        translations={
            '影像工作区 · 0.3.4':'Image workspace · 0.3.4',
            '每次显示一组影像；器械必须明确选择并配准后叠加。':'One image workspace at a time; select and register tools before overlay.',
            '已导入模型（可多选）':'Imported models (multiple selection)',
            '刷新模型列表':'Refresh model list','应用本影像模型选择':'Apply model selection to this image',
            '独立查看选中模型':'Preview selected models independently',
            '工具配对（可多选）':'Tool pairs (multiple selection)',
            '扫描默认 STP / aimtool':'Scan default STP / aimtool folders',
            '转换并验证扫描配对':'Convert and verify detected pairs',
            '创建选中独立工具集':'Create selected independent tool set',
            '同时展示导航 / 参考工具':'Show navigation and reference tools',
            'Tip 来源':'Tip source','显式表面阈值':'Surface threshold',
            '按阈值提取表面':'Extract threshold surface',
            'CT / MRI 体绘制预设':'CT / MRI rendering preset',
            '应用体绘制预设':'Apply rendering preset','打开标准体绘制设置':'Open volume rendering settings',
            '截图 / 录屏：待操作':'Capture / recording: idle',
            '工作流程 · 0.3.4':'Workflow · 0.3.4','艾目易导航 · 优化版 0.3.4':'Aimooe Navigation 0.3.4',
            '采集当前选定针尖为实物点':'Collect selected tip as a physical landmark'}
        self.english.update(translations)
        known={id(item[0]) for item in self.languageWidgets}
        for child in slicer.util.findChildren(self.parent):
            text=getattr(child,'text',None)
            if isinstance(text,str) and text in translations and id(child) not in known:
                self.languageWidgets.append((child,text,translations[text]))
        self.languageWidgets=[(widget,cn,translations.get(cn,en)) for widget,cn,en in self.languageWidgets]

    def toggleLanguage(self):
        super().toggleLanguage()
        if hasattr(self,'tipModeCombo'):
            from .AimooeNext import LANGUAGE
            names=(['LTS CAD tip target (nominal)','SDK file tip (physical calibration required)'] if LANGUAGE=='en'
                   else ['LTS 模型针尖靶点（CAD 名义值）','工具文件针尖（SDK，需实物标定）'])
            for index,name in enumerate(names):
                self.tipModeCombo.setItemText(index,name)

"""Reference display and explicitly aligned model/medical image import."""
import json
import time
from pathlib import Path
import vtk
import slicer
from .AimooeModels import matrix_from_rows


class AssetMixin:
    def canDisplayTool(self):
        if not self.running:
            return self.simulation
        if getattr(self, 'singleToolModeActive', False):
            return self.visualTracking('Tool')
        if self.referenceView and not self.visualTracking('Reference'):
            return False
        return ((self.visualTracking('Tool') and self.visualTracking('Reference')) or
                (self.previewOnly and self.visualTracking('Tool')))

    def setReferenceFrame(self, reference):
        self.createTransforms()
        if not self.simulation:
            if not self.tracker or reference not in self.tracker.names:
                raise ValueError('先连接设备并选择已加载的 Reference Tool')
            if reference == self.tracker.selected.get('Tool'):
                raise ValueError('Reference 必须与 Navigation Tool 不同')
            if self.tracker.selected.get('Reference') != reference:
                self.tracker.selected['Reference'] = reference
                self.tracker.poses['Reference'] = None
                self.tracker.lastUpdateTime['Reference'] = None
                self.valid['Reference'] = False
                self.last['Reference'] = None
        self.referenceEnabled = True
        self.referenceView = True
        self.invalidateRegistration()
        self.updateDisplayFrame()
        self.publish()
        self.cameraFitPending = True
        self.log.info('Reference display set: %s (raw SDK poses unchanged)', reference or 'SIMULATION')

    def clearReferenceFrame(self):
        self.referenceEnabled = False
        self.referenceView = False
        self.previewOnly = True
        self.invalidateRegistration()
        self.updateDisplayFrame()
        self.publish()
        self.cameraFitPending = True
        self.log.info('Reference display cleared: Tracker preview, NOT READY')

    def invalidateRegistration(self):
        for node in slicer.util.getNodesByClass('vtkMRMLLinearTransformNode'):
            if node.GetName() == 'ReferenceToImage':
                node.SetAttribute('Aimooe.RegistrationValidated', 'False')

    def updateDisplayFrame(self):
        display = self.nodes.get('TrackerToDisplay')
        if not display:
            return
        matrix = vtk.vtkMatrix4x4()
        if self.referenceView:
            if self.state('Reference') != 'TRACKING':
                display.SetAttribute('Aimooe.Valid', 'False')
                return
            reference = vtk.vtkMatrix4x4()
            self.nodes['ReferenceToTracker'].GetMatrixTransformToParent(reference)
            vtk.vtkMatrix4x4.Invert(reference, matrix)
        old = vtk.vtkMatrix4x4()
        display.GetMatrixTransformToParent(old)
        if any(abs(old.GetElement(i,j)-matrix.GetElement(i,j)) > 1e-10 for i in range(4) for j in range(4)):
            display.SetMatrixTransformToParent(matrix)
        display.SetAttribute('Aimooe.Valid', 'True')
        display.SetAttribute('Aimooe.DisplayFrame', 'Reference' if self.referenceView else 'Tracker')

    def importAsset(self, path, kind, reuse=True):
        path = Path(path)
        if not path.is_file():
            raise FileNotFoundError(str(path))
        loaders = {'Model': slicer.util.loadModel, 'Volume': slicer.util.loadVolume,
                   'Segmentation': slicer.util.loadSegmentation}
        if kind not in loaders:
            raise ValueError('Unsupported asset type')
        # Repeated import selects the existing independent asset.
        for node in slicer.util.getNodesByClass('vtkMRMLDisplayableNode'):
            if reuse and node.GetAttribute('Aimooe.SourceFile') == str(path.resolve()):
                return node
        if kind == 'Model':
            # CAD mesh coordinates are kept verbatim, independent of Slicer IO
            # anatomical LPS/RAS defaults. The supplied matrix uses these numbers.
            readers = {'.stl': vtk.vtkSTLReader, '.obj': vtk.vtkOBJReader,
                       '.ply': vtk.vtkPLYReader, '.vtk': vtk.vtkPolyDataReader,
                       '.vtp': vtk.vtkXMLPolyDataReader}
            if path.suffix.lower() not in readers:
                raise ValueError('Unsupported mesh; convert STEP to STL first')
            reader = readers[path.suffix.lower()]()
            reader.SetFileName(str(path))
            reader.Update()
            if reader.GetOutput().GetNumberOfPoints() == 0:
                raise ValueError('Empty or invalid mesh')
            mesh = vtk.vtkPolyData()
            mesh.DeepCopy(reader.GetOutput())
            node = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', path.stem)
            node.SetAndObservePolyData(mesh)
            node.CreateDefaultDisplayNodes()
            node.SetAttribute('Aimooe.Coordinates', 'RAW_CAD_NO_AXIS_FLIP')
        else:
            node = loaders[kind](str(path))
        if node is None:
            raise RuntimeError('Slicer failed to load ' + str(path))
        node.SetAttribute('Aimooe.SourceFile', str(path.resolve()))
        node.SetAttribute('Aimooe.Alignment', 'UNREGISTERED')
        if kind == 'Segmentation':
            node.CreateClosedSurfaceRepresentation()
        if kind == 'Volume':
            slicer.util.setSliceViewerLayers(background=node)
            slicer.util.resetSliceViews()
        self.log.info('Imported %s: %s; unregistered', kind, path)
        return node

    def bindAsset(self, model, role, matrixPath, toolName):
        if model is None or not model.IsA('vtkMRMLModelNode'):
            raise ValueError('Select a model node')
        if model.GetName() in self.displayModelNames:
            raise ValueError('请单独导入模型后绑定，不要修改内置针模型的变换链')
        if role not in ('Tool', 'Reference') or not toolName:
            raise ValueError('Select the actual tool name and binding role')
        data = json.loads(Path(matrixPath).read_text(encoding='utf-8-sig'))
        if data.get('units') != 'mm' or data.get('to') != role or data.get('toolName') != toolName:
            raise ValueError('JSON requires units=mm, to=selected role, toolName=selected tool')
        matrix = matrix_from_rows(data['matrix'])
        self.createTransforms()
        transform = self.node('ModelTo' + role + '_' + model.GetID())
        transform.SetMatrixTransformToParent(matrix)
        transform.SetAndObserveTransformNodeID(self.nodes[role+'ToTracker'].GetID())
        model.SetAndObserveTransformNodeID(transform.GetID())
        model.SetAttribute('Aimooe.Alignment', 'USER_SUPPLIED_RIGID_MATRIX_NOT_CALIBRATION')
        model.SetAttribute('Aimooe.BoundTool', toolName)
        self.assetBindings = [item for item in self.assetBindings if item[0] != model]
        self.assetBindings.append((model, role, toolName))
        self.updateAssetVisibility()
        self.log.info('Model bound to %s/%s using %s', role, toolName, matrixPath)

    def updateAssetVisibility(self):
        for model, role, name in self.assetBindings:
            if model.GetScene() is None:
                continue
            selected = self.simulation or (self.tracker and self.tracker.selected.get(role) == name)
            current = self.state(role) == 'TRACKING'
            held = bool(selected and self.visualTracking(role) and not current)
            visible = bool(selected and (current or held)) or (self.simulation and not self.running)
            if self.referenceView:
                visible = visible and self.visualTracking('Reference')
            display = model.GetDisplayNode()
            diagnostic = False
            if model.GetAttribute('Aimooe.Pair'):
                diagnostic = self.updateDiagnosticModel(model, role, name)
                visible = visible or diagnostic
            if model.GetAttribute('Aimooe.Pair'):
                updated = max(self.last.get(role) or 0, float(model.GetAttribute('Aimooe.LastDisplayTime') or 0))
                history = bool(selected and self.running and not visible
                               and updated is not None and 0 <= time.monotonic()-updated < getattr(self, 'historyPreviewSeconds', 0))
                model.SetAttribute('Aimooe.Historical', str(held or history))
                color = json.loads(model.GetAttribute('Aimooe.LiveColor'))
                if display.GetColor() != tuple(color):
                    display.SetColor(*color)
                opacity = 1.0  # History remains explicitly tagged and expires after 250 ms.
                if display.GetOpacity() != opacity:
                    display.SetOpacity(opacity)
                visible = visible or history
            if display.GetVisibility() != bool(visible):
                display.SetVisibility(visible)


    def updateCameraClipping(self):
        manager = slicer.app.layoutManager()
        if manager is None:
            return
        # Only change near/far planes, not the user's camera pose or zoom.
        for index in range(manager.threeDViewCount):
            renderer = manager.threeDWidget(index).threeDView().renderWindow().GetRenderers().GetFirstRenderer()
            if self.autoKeepInView:
                bounds = [float('inf'), float('-inf')] * 3
                models = [self.nodes[name] for name in self.displayModelNames if name in self.nodes]
                models += [item[0] for item in self.assetBindings]
                for model in models:
                    if model.GetScene() is None or not model.GetDisplayNode().GetVisibility():
                        continue
                    current = [0.] * 6
                    model.GetRASBounds(current)
                    for axis in range(3):
                        bounds[2*axis] = min(bounds[2*axis], current[2*axis])
                        bounds[2*axis+1] = max(bounds[2*axis+1], current[2*axis+1])
                if bounds[0] <= bounds[1]:
                    width, height = renderer.GetSize()
                    outside = False
                    for x in bounds[:2]:
                        for y in bounds[2:4]:
                            for z in bounds[4:6]:
                                renderer.SetWorldPoint(x,y,z,1.)
                                renderer.WorldToDisplay()
                                px,py,pz = renderer.GetDisplayPoint()
                                outside |= (px < width*.05 or px > width*.95 or py < height*.05 or py > height*.95)
                    if outside and width > 0 and height > 0:
                        renderer.ResetCamera(bounds)
                        renderer.GetActiveCamera().Zoom(0.85)
            camera = renderer.GetActiveCamera()
            bounds = renderer.ComputeVisiblePropBounds()
            if bounds[0] > bounds[1]:
                continue
            eye = camera.GetPosition()
            direction = camera.GetDirectionOfProjection()
            center = [(bounds[2*i]+bounds[2*i+1])*.5 for i in range(3)]
            depth = sum((center[i]-eye[i])*direction[i] for i in range(3))
            radius = sum(abs(direction[i])*(bounds[2*i+1]-bounds[2*i])*.5 for i in range(3))
            near, far = camera.GetClippingRange()
            if depth-radius < near*1.2 or depth+radius > far*.8:
                renderer.ResetCameraClippingRange()
                near, far = camera.GetClippingRange()
                camera.SetClippingRange(max(0.001,near*.5),far*1.5)

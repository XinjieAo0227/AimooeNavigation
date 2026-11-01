"""Tool definition geometry and independent image groups; never implies registration."""
import hashlib
import json
import uuid
from pathlib import Path
import vtk
import slicer
from .AimooeCoordinates import read_tool_definition
from .AimooeModels import sphere

ROOT = Path(__file__).resolve().parents[2]


class LibraryMixin:
    def createDefinitionModel(self, path, role):
        if role not in ('Tool', 'Reference', 'CalibrationTool'):
            raise ValueError('Select Tool, Reference or CalibrationTool')
        definition = read_tool_definition(path)
        self.createTransforms()
        name = 'ToolMarkers_' + definition['name'] + '_' + role
        model = self.node(name, 'vtkMRMLModelNode')
        append = vtk.vtkAppendPolyData()
        for point in definition['markers']:
            # Small glyphs indicate exact centers, not physical sphere diameter.
            append.AddInputData(sphere(point, 1.5))
        append.Update()
        mesh = vtk.vtkPolyData()
        mesh.DeepCopy(append.GetOutput())
        model.SetAndObservePolyData(mesh)
        model.CreateDefaultDisplayNodes()
        model.GetDisplayNode().SetColor(0.2, 0.9, 0.8)
        model.SetAndObserveTransformNodeID(self.nodes[role+'ToTracker'].GetID())
        model.SetAttribute('Aimooe.Geometry', 'EXACT_FILE_CENTERS_ONLY_NOT_CAD_BODY')
        model.SetAttribute('Aimooe.ToolDefinition', str(Path(path).resolve()))
        model.SetAttribute('Aimooe.ToolDefinitionSHA256', hashlib.sha256(Path(path).read_bytes()).hexdigest())
        self.assetBindings = [entry for entry in self.assetBindings if entry[0] != model]
        self.assetBindings.append((model, role, definition['name']))
        self.updateAssetVisibility()
        self.log.info('Loaded marker center geometry: %s, role=%s', path, role)
        return model

    @staticmethod
    def assetKind(path):
        text = str(path).lower()
        if text.endswith('.seg.nrrd'):
            return 'Segmentation'
        if text.endswith(('.stl', '.obj', '.ply', '.vtk', '.vtp')):
            return 'Model'
        if text.endswith(('.nrrd', '.nhdr', '.nii', '.nii.gz', '.mha', '.mhd', '.hdr', '.img')):
            return 'Volume'
        raise ValueError('Unsupported group file: ' + str(path))

    def importAssetGroup(self, files, name, toolFile, groupId=None):
        if self.running:
            raise ValueError('请先 Stop Tracking，再导入影像组，避免磁盘加载阻塞采集')
        definition = read_tool_definition(toolFile) if toolFile else {'name':''}
        if not files or not name.strip():
            raise ValueError('Choose files and a group name')
        paths = [Path(path).resolve() for path in files]
        for path in paths:
            if not path.is_file():
                raise FileNotFoundError(str(path))
            self.assetKind(path)
        groupId = groupId or uuid.uuid4().hex
        if not groupId.isalnum():
            raise ValueError('Invalid group identifier')
        existing = [n for n in slicer.util.getNodesByClass('vtkMRMLDisplayableNode')
                    if n.GetAttribute('Aimooe.GroupId') == groupId]
        if existing:
            self.showAssetGroup(groupId)
            return groupId
        before = {slicer.mrmlScene.GetNthNode(i).GetID() for i in range(slicer.mrmlScene.GetNumberOfNodes())}
        sh = slicer.vtkMRMLSubjectHierarchyNode.GetSubjectHierarchyNode(slicer.mrmlScene)
        folder = sh.CreateFolderItem(sh.GetSceneItemID(), name.strip() + ' [UNREGISTERED]')
        try:
            for path in paths:
                node = self.importAsset(path, self.assetKind(path), reuse=False)
                node.SetAttribute('Aimooe.GroupId', groupId)
                node.SetAttribute('Aimooe.GroupTool', definition['name'])
                sh.SetItemParent(sh.GetItemByDataNode(node), folder)
            destination = getattr(self, 'groupManifestDirectory', ROOT/'Config/Groups')
            destination.mkdir(parents=True, exist_ok=True)
            manifest = {'schemaVersion': 1, 'id': groupId, 'name': name.strip(),
                        'toolFile': str(Path(toolFile).resolve()) if toolFile else None, 'toolName': definition['name'],
                        'files': [str(path) for path in paths], 'registration': 'UNREGISTERED'}
            (destination/(groupId+'.json')).write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
        except Exception:
            for i in reversed(range(slicer.mrmlScene.GetNumberOfNodes())):
                node = slicer.mrmlScene.GetNthNode(i)
                if node.GetID() not in before:
                    slicer.mrmlScene.RemoveNode(node)
            sh.RemoveItem(folder)
            self.log.exception('Group import rolled back')
            raise
        self.showAssetGroup(groupId)
        return groupId

    def showAssetGroup(self, groupId):
        firstVolume = None
        for node in slicer.util.getNodesByClass('vtkMRMLDisplayableNode'):
            identity = node.GetAttribute('Aimooe.GroupId')
            if not identity:
                continue
            active = identity == groupId
            if node.GetDisplayNode():
                node.GetDisplayNode().SetVisibility(active)
            if active and node.IsA('vtkMRMLScalarVolumeNode') and firstVolume is None:
                firstVolume = node
        slicer.util.setSliceViewerLayers(background=firstVolume, foreground=None, label=None)
        if firstVolume:
            slicer.util.resetSliceViews()

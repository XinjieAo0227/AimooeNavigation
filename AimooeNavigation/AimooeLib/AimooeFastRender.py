"""GPU actor matrices for rigid CAD; original geometry and calibration stay intact."""
import vtk
import slicer

class FastToolRenderer:
    def __init__(self):
        self.items={}
    def update(self,bindings):
        manager=slicer.app.layoutManager()
        if not manager:return
        models={node.GetID():node for node,_,_ in bindings if node.GetScene() and node.GetAttribute('Aimooe.Pair')}
        views={manager.threeDWidget(i).threeDView().mrmlViewNode().GetID():manager.threeDWidget(i).threeDView()
               for i in range(manager.threeDViewCount) if manager.threeDWidget(i).threeDView().visible}
        wanted={(identity,viewid) for identity in models for viewid in views}
        for key in set(self.items)-wanted:
            renderer,actor,mapper=self.items.pop(key)
            renderer.RemoveActor(actor)
        for identity,model in models.items():
            display=model.GetDisplayNode()
            # Keep MRML data and visibility as the authoritative scene state,
            # while avoiding per-vertex CPU transformation by native display managers.
            if display.GetNumberOfViewNodeIDs()!=1 or display.GetNthViewNodeID(0)!='AimooeGPUOnly':
                display.RemoveAllViewNodeIDs()
                display.AddViewNodeID('AimooeGPUOnly')
            transform=vtk.vtkMatrix4x4()
            parent=model.GetParentTransformNode()
            if parent:parent.GetMatrixTransformToWorld(transform)
            for viewid,view in views.items():
                key=(identity,viewid)
                if key not in self.items:
                    renderer=view.renderWindow().GetRenderers().GetFirstRenderer()
                    mapper=vtk.vtkPolyDataMapper();mapper.ScalarVisibilityOff()
                    actor=vtk.vtkActor();actor.SetMapper(mapper);actor.PickableOff()
                    renderer.AddActor(actor)
                    self.items[key]=(renderer,actor,mapper)
                renderer,actor,mapper=self.items[key]
                if mapper.GetInput()!=model.GetPolyData():mapper.SetInputData(model.GetPolyData())
                actor.SetUserMatrix(transform)
                actor.SetVisibility(display.GetVisibility())
                actor.GetProperty().SetColor(display.GetColor())
                actor.GetProperty().SetOpacity(display.GetOpacity())
                view.scheduleRender()
    def clear(self):
        for renderer,actor,_ in self.items.values():renderer.RemoveActor(actor)
        self.items.clear()

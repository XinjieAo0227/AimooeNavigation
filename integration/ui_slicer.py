import json
import traceback
from pathlib import Path

import numpy as np
import qt
import slicer
import vtk

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / 'Logs/ui082-test.json'
w = None


def finish(error=None):
    REPORT.write_text(json.dumps({'result': 'FAIL' if error else 'PASS', 'error': error}, indent=2), encoding='utf-8')
    if w:
        w.cleanup()
    slicer.app.exit(1 if error else 0)


def verify():
    try:
        assert w.guidanceTabs.count == 4
        assert all('规划' not in w.guidanceTabs.tabText(i) for i in range(w.guidanceTabs.count))
        assert not hasattr(w, 'targetTable')
        assert hasattr(w, 'pointPrecisionButton') and hasattr(w, 'surfacePrecisionButton')
        assert not hasattr(w, 'placeNavigationTarget')
        assert not hasattr(w, 'placeValidationPoint') and not hasattr(w, 'captureValidation')
        assert not hasattr(w, 'pointPrecisionLabel') and not hasattr(w, 'surfacePrecisionLabel')
        assert callable(w.showCoordinates)
        w.showCoordinates(); w.liveRate.value=7; w.liveRateChanged(7)
        assert 140 <= w.liveTimer.interval <= 145
        assert hasattr(w,'allToolTable')

        volume = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLScalarVolumeNode', 'UI082Case')
        slicer.util.updateVolumeFromArray(volume, np.ones((4, 4, 4), dtype=np.int16))
        w.volumeSelector.setCurrentNode(volume)
        w.createPointLists()
        source = np.array([[0, 0, 0], [30, 0, 0], [0, 30, 0], [0, 0, 30]], float)
        target = source + [2, 3, 4]
        for p, q in zip(source, target):
            w.referencePoints.currentNode().AddControlPoint(vtk.vtkVector3d(*p))
            w.imagePoints.currentNode().AddControlPoint(vtk.vtkVector3d(*q))
        w.registerSelected()

        precision = {'POINT': {'repeatability': [
            {'image': [0, 0, 0]}, {'image': [2, 0, 0]}],
            'absolute': [{'measured': [1, 0, 0], 'truth': [0, 0, 0]}]},
            'SURFACE': {'repeatability': [], 'absolute': []}}
        volume.SetAttribute('Aimooe.PrecisionReport', json.dumps(precision))
        w.refreshPrecisionMetrics()
        model,maximum=w.buildPrecisionVisualization('POINT',volume,precision['POINT'])
        assert maximum == 1.0 and model.GetPolyData().GetPointData().GetScalars().GetName()=='ErrorMm'
        assert model.GetDisplayNode().GetScalarVisibility()
        owned = [volume.GetAttribute('Aimooe.PrecisionVisualization_POINT')]
        w.logic.running=True;w.logic.last['Tool']=__import__('time').monotonic();w.logic.valid['Tool']=False
        assert w.logic.state('Tool')=='LOST' and w.logic.visualTracking('Tool')
        w.logic.last['Tool']-=1
        assert not w.logic.visualTracking('Tool');w.logic.running=False
        w.clearCurrentCase()
        assert all(slicer.mrmlScene.GetNodeByID(identity) is None for identity in owned)
        finish()
    except Exception:
        finish(traceback.format_exc())


try:
    slicer.util.selectModule('AimooeNavigation')
    w = slicer.modules.aimooenavigation.widgetRepresentation().self()
    qt.QTimer.singleShot(500, verify)
except Exception:
    finish(traceback.format_exc())

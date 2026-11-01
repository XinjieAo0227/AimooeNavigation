"""Exercise the single-button precision capture and all-tool coordinate panel."""
import json
import time
import traceback
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import qt
import slicer
import vtk

ROOT=Path(__file__).resolve().parents[1]
REPORT=ROOT/'Logs/precision082-test.json'
w=None
baselineHz=0.


def finish(error=None, details=None):
    REPORT.write_text(json.dumps({'result':'FAIL' if error else 'PASS','error':error,
                                  'details':details or {}},indent=2),encoding='utf-8')
    if w:w.cleanup()
    slicer.app.exit(1 if error else 0)


def check_collection():
    try:
        volume=w.volumeSelector.currentNode()
        data=w.precisionReport(volume)['POINT']
        count=len(data['repeatability'])
        assert count>=3, (count,w.pointPrecisionStatus.text)
        assert len(data['absolute'])==count
        assert all(item['truth']==[2.,3.,4.] for item in data['absolute'])
        node=slicer.mrmlScene.GetNodeByID(volume.GetAttribute('Aimooe.PrecisionVisualization_POINT'))
        assert node and node.GetPolyData().GetPointData().GetScalars().GetName()=='ErrorMm'
        captureHz=w.logic.rate()
        assert captureHz>=baselineHz-2.,(baselineHz,captureHz)
        w.togglePrecisionCollection('POINT')
        assert not w.precisionTimer.isActive()
        assert w.precisionStage is None
        w.togglePrecisionCollection('POINT')
        assert w.precisionTimer.isActive()
        qt.QTimer.singleShot(8500,lambda:check_auto(captureHz))
    except Exception:
        finish(traceback.format_exc())


def check_auto(captureHz):
    try:
        volume=w.volumeSelector.currentNode()
        data=w.precisionReport(volume)['POINT']
        assert len(data['repeatability'])==30 and len(data['absolute'])==30
        assert not w.precisionTimer.isActive() and w.precisionStage is None
        assert w.pointPrecisionButton.text=='开始点配准精度采集'
        node=slicer.mrmlScene.GetNodeByID(volume.GetAttribute('Aimooe.PrecisionVisualization_POINT'))
        assert node and node.GetPolyData().GetNumberOfPoints()>1000
        finish(details={'autoSamples':30,'absoluteSamples':30,'baselineTrackingHz':baselineHz,
                        'captureTrackingHz':captureHz,'visualizationPoints':node.GetPolyData().GetNumberOfPoints()})
    except Exception:
        finish(traceback.format_exc())


def begin():
    try:
        assert not hasattr(w,'targetTable') and not hasattr(w,'startTargetNavigation')
        assert '所有工具实时坐标 / Live coordinates' in [child.text for child in slicer.util.findChildren(w.parent)
                                                       if child.inherits('QPushButton')]
        w.showCoordinates()
        real_tracker=w.logic.tracker
        real_simulation=w.logic.simulation
        w.logic.simulation=False
        w.logic.tracker=SimpleNamespace(samples={},selected={'Tool':'LTS','Reference':'RFS',
                                                              'CalibrationTool':'CAL'},
                                        names=['LTS','RFS','CAL','UNSELECTED'],last_frame=1)
        tools=w.logic.liveCoordinates()['trackedTools']
        w.logic.tracker=real_tracker;w.logic.simulation=real_simulation
        assert len(tools)==4 and tools[-1]['status']=='NOT_VISIBLE',tools
        volume=slicer.mrmlScene.AddNewNodeByClass('vtkMRMLScalarVolumeNode','Precision082Case')
        slicer.util.updateVolumeFromArray(volume,np.ones((4,4,4),dtype=np.int16))
        w.volumeSelector.setCurrentNode(volume)
        w.createPointLists()
        source=np.array([[0,0,0],[30,0,0],[0,30,0],[0,0,30]],float)
        for point in source:
            w.referencePoints.currentNode().AddControlPoint(vtk.vtkVector3d(*point))
            w.imagePoints.currentNode().AddControlPoint(vtk.vtkVector3d(*(point+[2,3,4])))
        w.registerSelected()
        truth=slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode','IndependentGroundTruth')
        truth.CreateDefaultDisplayNodes();truth.AddControlPoint(vtk.vtkVector3d(2,3,4))
        w.pointTruthSelector.setCurrentNode(truth)
        w.logic.startSimulation();w.logic.loadSelectedPairs(w.tool.currentText,w.reference.currentText)
        qt.QTimer.singleShot(500,start_collection)
    except Exception:
        finish(traceback.format_exc())


def start_collection():
    try:
        w.logic.applyChosenTip()
        qt.QTimer.singleShot(2000,commence_collection)
    except Exception:
        finish(traceback.format_exc())


def commence_collection():
    global baselineHz
    try:
        baselineHz=w.logic.rate()
        w.logic.frames.clear()
        w.togglePrecisionCollection('POINT')
        assert w.precisionTimer.isActive()
        qt.QTimer.singleShot(1800,check_collection)
    except Exception:
        finish(traceback.format_exc())


try:
    slicer.util.selectModule('AimooeNavigation')
    w=slicer.modules.aimooenavigation.widgetRepresentation().self()
    qt.QTimer.singleShot(500,begin)
except Exception:
    finish(traceback.format_exc())

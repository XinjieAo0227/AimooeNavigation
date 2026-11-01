"""Physical needle-tip pivot calibration and its Slicer UI."""
import json
import re
import time
from pathlib import Path

import numpy as np
import qt
import slicer
import vtk


def pivot_calibration(matrices):
    """Solve R_i * tipTool + t_i = fixedPivot in tracker coordinates."""
    values=np.asarray(matrices,dtype=float)
    if values.ndim!=3 or values.shape[1:]!=(4,4) or len(values)<20 or not np.isfinite(values).all():
        raise ValueError('Pivot Calibration 至少需要 20 个有限的 4×4 位姿样本')
    blocks=[];rhs=[]
    for matrix in values:
        rotation=matrix[:3,:3]
        if not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-3) or np.linalg.det(rotation)<.99:
            raise ValueError('样本包含无效旋转矩阵')
        blocks.append(np.hstack((rotation,-np.eye(3))))
        rhs.append(-matrix[:3,3])
    a=np.vstack(blocks);b=np.concatenate(rhs)
    solution,_,rank,singular=np.linalg.lstsq(a,b,rcond=None)
    if rank<6:
        raise ValueError('Pivot 样本姿态退化；请绕固定针尖改变更多方向')
    tip=solution[:3];pivot=solution[3:]
    predicted=np.einsum('nij,j->ni',values[:,:3,:3],tip)+values[:,:3,3]
    residual=np.linalg.norm(predicted-pivot,axis=1)
    relative=np.einsum('ij,njk->nik',values[0,:3,:3].T,values[:,:3,:3])
    angles=np.degrees(np.arccos(np.clip((np.trace(relative,axis1=1,axis2=2)-1)/2,-1,1)))
    spread=float(np.max(angles))
    if spread<20:
        raise ValueError('Pivot 姿态覆盖不足 20°；结果不可靠，未允许应用')
    return {'tipTool':tip.tolist(),'pivotTracker':pivot.tolist(),'sampleCount':len(values),
            'rank':int(rank),'orientationSpreadDeg':spread,
            'rmsMm':float(np.sqrt(np.mean(residual**2))),
            'maximumMm':float(np.max(residual)),'singularValues':singular.tolist()}


class CalibrationWidgetMixin:
    def setupCalibration(self):
        self.pivotSamples=[];self.pivotFrames=set();self.pendingPivot=None
        self.pivotTimer=qt.QTimer();self.pivotTimer.setInterval(50)
        self.pivotTimer.connect('timeout()',self.capturePivotSample)
        panel=self.guidanceTabs.widget(1).layout()
        self.pivotStatus=qt.QLabel('长针单工具模式必须先完成真实 Pivot Calibration。')
        self.pivotStatus.wordWrap=True
        panel.addRow(self.pivotStatus)
        self.button(panel,'开始针尖 Pivot Calibration',self.startPivotCalibration)
        self.button(panel,'停止并计算 Pivot Calibration',self.finishPivotCalibration)
        self.button(panel,'应用物理针尖标定',self.applyPivotCalibration)
        self.button(panel,'载入当前工具已保存的针尖标定',self.loadTipCalibration)

    def startPivotCalibration(self):
        if self.logic.simulation:
            raise ValueError('物理针尖标定禁止使用模拟位姿')
        if not self.logic.running or self.logic.state('Tool')!='TRACKING':
            raise ValueError('请先仅启动长针并确认 Tool 为 TRACKING')
        self.pivotSamples=[];self.pivotFrames=set();self.pendingPivot=None
        self.pivotTimer.start()
        self.pivotStatus.text='保持真实针尖固定在同一点，缓慢改变长针方向；正在采样。'

    def capturePivotSample(self):
        if not self.logic.running or self.logic.state('Tool')!='TRACKING':
            return
        frame=getattr(self.logic.tracker,'last_frame',None)
        if frame is None or frame in self.pivotFrames:
            return
        matrix=vtk.vtkMatrix4x4();self.logic.nodes['ToolToTracker'].GetMatrixTransformToParent(matrix)
        self.pivotSamples.append([[matrix.GetElement(i,j) for j in range(4)] for i in range(4)])
        self.pivotFrames.add(frame)
        self.pivotStatus.text='Pivot Calibration 已采集 %d 帧；继续覆盖不同方向。'%len(self.pivotSamples)

    def finishPivotCalibration(self):
        self.pivotTimer.stop()
        self.pendingPivot=pivot_calibration(self.pivotSamples)
        result=self.pendingPivot
        self.pivotStatus.text=('待应用：%d 帧，方向覆盖 %.1f°，RMS %.3f mm，最大残差 %.3f mm。'
            '这些是标定拟合指标，不是临床导航精度。')%(result['sampleCount'],result['orientationSpreadDeg'],result['rmsMm'],result['maximumMm'])

    def calibrationPath(self):
        name=self.tool.currentText.strip()
        if not name: raise ValueError('未选择 Navigation Tool')
        safe=re.sub(r'[^A-Za-z0-9_.-]+','_',name)
        return Path(__file__).resolve().parents[2]/'Config/TipCalibrations'/(safe+'.json')

    def applyPivotCalibration(self):
        if not self.pendingPivot:
            raise ValueError('没有待应用的 Pivot Calibration 结果')
        result=dict(self.pendingPivot,tool=self.tool.currentText,created=time.strftime('%Y-%m-%dT%H:%M:%S%z'),method='PIVOT_CALIBRATION')
        path=self.calibrationPath();path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        self._applyTipCalibration(result,path)

    def loadTipCalibration(self):
        path=self.calibrationPath()
        if not path.is_file():
            raise FileNotFoundError('当前工具没有已保存的针尖标定：'+str(path))
        result=json.loads(path.read_text(encoding='utf-8-sig'))
        if result.get('tool')!=self.tool.currentText or result.get('method')!='PIVOT_CALIBRATION':
            raise ValueError('标定文件的工具或方法不匹配')
        self._applyTipCalibration(result,path)

    def _applyTipCalibration(self,result,path):
        matrix=vtk.vtkMatrix4x4()
        for i,value in enumerate(result['tipTool']):matrix.SetElement(i,3,float(value))
        node=self.logic.nodes['TipToTool'];node.SetMatrixTransformToParent(matrix)
        node.SetAttribute('Aimooe.TipCalibration','CALIBRATED')
        node.SetAttribute('Aimooe.CalibrationMethod','PIVOT_CALIBRATION')
        node.SetAttribute('Aimooe.CalibrationFile',str(path))
        node.SetAttribute('Aimooe.CalibrationRMSmm',str(result['rmsMm']))
        node.SetAttribute('Aimooe.CalibrationTool',result['tool'])
        self.pivotStatus.text='已应用物理针尖标定：RMS %.3f mm；文件 %s'%(result['rmsMm'],path.name)
        self.logic.invalidateRegistration();self.logic.publish()

    def cleanupCalibration(self):
        if hasattr(self,'pivotTimer'):self.pivotTimer.stop()

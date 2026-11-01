"""Exact RAS-to-screen tip projection without reslicing a volume for each dot."""
import qt
import vtk
import slicer

class TipOverlay(qt.QWidget):
    def __init__(self,parent):
        super().__init__(parent)
        self.point=None
        self.setAttribute(qt.Qt.WA_TransparentForMouseEvents)
        self.setAttribute(qt.Qt.WA_NoSystemBackground)
        self.setAttribute(qt.Qt.WA_TranslucentBackground)
    def paintEvent(self,event):
        if self.point is None:return
        painter=qt.QPainter(self)
        painter.setRenderHint(qt.QPainter.Antialiasing)
        painter.setPen(qt.Qt.NoPen)
        painter.setBrush(qt.QColor(255,0,0))
        painter.drawEllipse(qt.QPointF(*self.point),4,4)
        painter.end()

class TipOverlays:
    def __init__(self):self.items={}
    def update(self,xyz,visible=True):
        manager=slicer.app.layoutManager()
        if not manager:return
        for name in manager.sliceViewNames():
            widget=manager.sliceWidget(name);view=widget.sliceView()
            if name not in self.items or self.items[name][0]!=view:
                if name in self.items:self.items[name][1].deleteLater()
                self.items[name]=(view,TipOverlay(view))
            overlay=self.items[name][1]
            if not visible:
                overlay.hide();continue
            inverse=vtk.vtkMatrix4x4()
            vtk.vtkMatrix4x4.Invert(widget.mrmlSliceNode().GetXYToRAS(),inverse)
            x,y,_,_=inverse.MultiplyPoint([*xyz,1.])
            # MRML XY is bottom-left; Qt coordinates are top-left.
            rw=view.renderWindow();width,height=rw.GetSize()
            if width<=0 or height<=0:overlay.hide();continue
            overlay.point=(x*view.width/width,(height-1-y)*view.height/height)
            overlay.setGeometry(0,0,view.width,view.height)
            overlay.show();overlay.raise_();overlay.update()
    def close(self):
        for _,overlay in self.items.values():overlay.hide();overlay.deleteLater()
        self.items.clear()

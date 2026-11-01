"""Qt capture with bounded encoder queue; never blocks the tracking timer."""
from pathlib import Path
import os
import shutil
import time
import qt
import slicer


class RegionOverlay(qt.QWidget):
    def __init__(self, target, callback):
        super().__init__(target)
        self.callback = callback
        self.setGeometry(target.rect)
        self.setStyleSheet('background: rgba(20, 90, 180, 35);')
        self.setCursor(qt.Qt.CrossCursor)
        self.band = qt.QRubberBand(qt.QRubberBand.Rectangle, self)
        self.origin = None
        self.show()
        self.raise_()
        self.setFocus()

    def mousePressEvent(self, event):
        self.origin = event.pos()
        self.band.setGeometry(qt.QRect(self.origin, qt.QSize()))
        self.band.show()

    def mouseMoveEvent(self, event):
        if self.origin is not None:
            self.band.setGeometry(qt.QRect(self.origin, event.pos()).normalized())

    def mouseReleaseEvent(self, event):
        rect = qt.QRect(self.origin, event.pos()).normalized().intersected(self.rect)
        self.hide()
        if rect.width() >= 16 and rect.height() >= 16:
            self.callback(rect)
        self.deleteLater()

    def keyPressEvent(self, event):
        if event.key() == qt.Qt.Key_Escape:
            self.hide()
            self.deleteLater()


class CaptureSession:
    def __init__(self, root, logger):
        self.root, self.log = Path(root), logger
        self.target = None
        self.region = None
        self.process = None
        self.timer = qt.QTimer()
        self.timer.connect('timeout()', self.frame)
        self.frames = self.dropped = 0
        self.error = ''
        self.stopping = False
        self.deadline = qt.QTimer()
        self.deadline.setSingleShot(True)
        self.deadline.connect('timeout()', self.timeout)

    def pixmap(self):
        target = self.target or slicer.util.mainWindow()
        pixmap = target.grab()
        if self.region is not None:
            # QWidget rectangles use logical pixels; QPixmap uses device pixels.
            ratio = pixmap.devicePixelRatioF()
            r = self.region
            pixmap = pixmap.copy(int(r.x()*ratio), int(r.y()*ratio),
                                int(r.width()*ratio), int(r.height()*ratio))
        return pixmap

    def screenshot(self, path):
        path = Path(path)
        if path.suffix.lower() not in ('.png', '.jpg', '.jpeg', '.bmp'):
            raise ValueError('PNG / JPEG / BMP required')
        path.parent.mkdir(parents=True, exist_ok=True)
        if not self.pixmap().save(str(path)):
            raise IOError('Failed to save screenshot: '+str(path))
        self.log.info('Screenshot: %s', path)

    def start(self, path, fps=10):
        if self.process is not None:
            raise ValueError('Recording is already active or finalizing')
        encoder = Path(os.environ.get('AIMOOE_FFMPEG', shutil.which('ffmpeg') or 'ffmpeg'))
        if not encoder.is_file():
            raise FileNotFoundError('Install an independently licensed FFmpeg and set AIMOOE_FFMPEG.')
        path = Path(path)
        if path.suffix.lower() not in ('.mp4', '.mkv', '.avi'):
            raise ValueError('MP4 / MKV / AVI required')
        path.parent.mkdir(parents=True, exist_ok=True)
        pixmap = self.pixmap()
        self.width = max(2, pixmap.width()//2*2)
        self.height = max(2, pixmap.height()//2*2)
        self.frames = self.dropped = 0
        self.error = ''
        self.stopping = False
        self.path = str(path)
        self.started = time.monotonic()
        self.process = qt.QProcess()
        self.process.setStandardErrorFile(str(path)+'.encoder.log')
        self.process.connect('finished(int,QProcess::ExitStatus)', self.finished)
        self.process.connect('errorOccurred(QProcess::ProcessError)', self.failed)
        args = ['-y','-loglevel','warning','-f','image2pipe','-vcodec','mjpeg',
                '-framerate',str(fps),'-i','pipe:0','-an','-c:v',
                'mpeg4' if path.suffix.lower()=='.avi' else 'libx264',
                '-pix_fmt','yuv420p',str(path)]
        self.process.start(str(encoder), args)
        self.timer.start(round(1000/fps))

    def frame(self):
        if self.process is None or self.stopping:
            return
        if self.process.state() != qt.QProcess.Running:
            return
        if self.process.bytesToWrite() > 2*1024*1024:
            self.dropped += 1
            return
        try:
            pixmap = self.pixmap().scaled(self.width,self.height,
                                          qt.Qt.IgnoreAspectRatio,qt.Qt.FastTransformation)
            buffer = qt.QBuffer()
            buffer.open(qt.QIODevice.WriteOnly)
            if not pixmap.save(buffer,'JPG',85):
                raise IOError('Frame encoding failed')
            self.process.write(buffer.data())
            self.frames += 1
        except Exception as exc:
            self.error = str(exc)
            self.log.exception('Recording failed')
            self.stop()

    def stop(self):
        self.timer.stop()
        if self.process is not None and not self.stopping:
            self.stopping = True
            self.process.closeWriteChannel()
            self.deadline.start(5000)

    def shutdown(self):
        self.stop()
        # Only at application shutdown: let the muxer write the container index.
        process = self.process
        if process is not None and not process.waitForFinished(2000):
            self.timeout()

    def timeout(self):
        if self.process is not None:
            self.error = 'Encoder did not finish within 5 seconds'
            self.log.error(self.error)
            self.process.kill()  # Only this session's encoder, never other apps.

    def failed(self, code):
        self.error = 'Encoder error '+str(code)
        self.log.error(self.error)
        self.timer.stop()
        if self.process and self.process.state()==qt.QProcess.NotRunning:
            self.finished(-1,0)

    def finished(self, code, status):
        self.timer.stop()
        self.deadline.stop()
        if code:
            self.error = self.error or ('Encoder exit '+str(code))
        self.log.info('Recording completed: %s frames=%d dropped=%d error=%s',
                      getattr(self,'path',''),self.frames,self.dropped,self.error)
        import json
        if hasattr(self,'path'):
            Path(self.path+'.json').write_text(json.dumps({'frames':self.frames,'dropped':self.dropped,
                'elapsedSeconds':time.monotonic()-self.started,'nominalFps':10,'error':self.error},indent=2),encoding='utf-8')
        process = self.process
        self.process = None
        if process:
            process.deleteLater()

"""Single-owner SDK process; latest complete snapshot, independent of GUI rendering."""
import json
import logging
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path

FIELDS=('connected','tracking','names','selected','previewUpdated','samples','allToolSamples','lastUpdateTime',
        'last_frame','marker_count','toolInfo','quality_message','hardware_validated','measured_markers',
        'rejected_frames','role_reason','diagnostics')

def encode_matrices(values):
    return {key:None if value is None else [[value.GetElement(i,j) for j in range(4)] for i in range(4)]
            for key,value in values.items()}

def worker(runtime,logpath):
    print('worker boot',file=sys.stderr,flush=True)
    commands=queue.Queue()
    def receive():
        for line in sys.stdin:
            print('command received',file=sys.stderr,flush=True)
            try: commands.put(json.loads(line))
            except ValueError: continue
        commands.put({'id':-1,'method':'disconnectDevice','args':[]})
    def send(value):
        print('AIMSTREAM '+json.dumps(value,ensure_ascii=True,separators=(',',':')),flush=True)
    from AimooeLib.AimooeTracker import AimooeTracker
    from AimooeLib.AimooeModels import matrix_from_rows
    print('imports ready',file=sys.stderr,flush=True)
    threading.Thread(target=receive,daemon=True).start()
    tracker=AimooeTracker(runtime,logging.getLogger('AimooeStream'))
    if runtime=='TEST_95HZ':
        class TestTracker(AimooeTracker):
            def initializeSdk(self): pass
            def connectDevice(self,interface): self.connected=True
            def loadToolFiles(self,path): self.names=['LTS','RFS'];return self.names
            def startTracking(self): self.tracking=True;self.due=time.monotonic();self.last_frame=0
            def stopTracking(self): self.tracking=False
            def disconnectDevice(self): self.tracking=False;self.connected=False
            def updateTracking(self):
                now=time.monotonic()
                if now<self.due: return None
                self.due+=1/95;self.last_frame+=1
                self.lastUpdateTime=dict.fromkeys(self.poses,now)
                matrix=matrix_from_rows([[1,0,0,self.last_frame],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
                self.poses=dict.fromkeys(self.poses,matrix)
                return self.poses
        tracker=TestTracker(runtime,logging.getLogger('AimooeStreamTest'))
    recording=None
    sequence=0
    started=time.monotonic()
    try:
        while True:
            try: command=commands.get_nowait()
            except queue.Empty: command=None
            if command:
                method=command['method']
                print('dispatch '+method,file=sys.stderr,flush=True)
                try:
                    if method=='startTracking':
                        tracker.minimumMatchPts=command.get('minimumMatchPts',0)
                    value=getattr(tracker,method)(*command.get('args',[]))
                    if method=='startTracking':
                        started=time.monotonic();sequence=0
                        if recording: recording.close();recording=None
                        if command.get('recordRaw',False):
                            recording=open(logpath,'a',encoding='utf-8',buffering=1024*1024)
                    send({'reply':command['id'],'value':value,'state':{k:getattr(tracker,k) for k in FIELDS}})
                except Exception as exc:
                    send({'reply':command['id'],'error':str(exc)})
                if method=='disconnectDevice': break
            if tracker.tracking:
                poses=tracker.updateTracking()
                if poses is not None:
                    sequence+=1
                    now=time.monotonic()
                    state={k:getattr(tracker,k) for k in FIELDS}
                    packet={'sequence':sequence,'received':now,'state':state,
                            'poses':encode_matrices(poses),'previewPoses':encode_matrices(tracker.previewPoses),
                            'acquisitionHz':sequence/max(now-started,.001)}
                    send(packet)
                    if recording:
                        recording.write(json.dumps(packet,separators=(',',':'))+'\n')
                        if sequence%100==0: recording.flush()
            time.sleep(.001)
    finally:
        if tracker.connected: tracker.disconnectDevice()
        if recording: recording.close()


class AimooeStreamTracker:
    def __init__(self,runtime,logger,python,logfolder):
        from .AimooeModels import matrix_from_rows
        self.matrix_from_rows=matrix_from_rows
        self.log=logger
        self.connected=False;self.tracking=False;self.names=[];self.selected={}
        self.previewPoses={};self.previewUpdated={};self.samples={};self.allToolSamples={};self.minimumMatchPts=0
        self.poses={'Tool':None,'Reference':None,'CalibrationTool':None};self.lastUpdateTime={'Tool':None,'Reference':None,'CalibrationTool':None}
        self.last_frame=None;self.marker_count=0;self.toolInfo=[];self.quality_message=''
        self.hardware_validated=False;self.measured_markers=[];self.rejected_frames=0
        self.role_reason={};self.diagnostics={'frames':0,'qualityReasons':{},'roles':{}}
        self.acquisitionHz=0.;self.lastConsumedSequence=0;self.receivedTime=0.
        self.recordRaw=False
        self._lock=threading.Lock();self._latest=None;self._responses={};self._counter=0
        folder=Path(logfolder);folder.mkdir(parents=True,exist_ok=True)
        self.recordingPath=folder/('coordinates-'+time.strftime('%Y%m%d-%H%M%S')+'.jsonl')
        self._stderr=open(self.recordingPath.with_suffix('.log'),'w',encoding='utf-8')
        executable=Path(python).with_name('python-real.exe')
        if not executable.is_file(): executable=Path(python)
        self.process=subprocess.Popen([str(executable),str(Path(__file__).resolve()),str(runtime),str(self.recordingPath)],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self._stderr,text=True,encoding='utf-8',
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0),bufsize=1)
        self._reader=threading.Thread(target=self._read,daemon=True)
        self._reader.start()

    def _read(self):
        for line in self.process.stdout:
            if not line.startswith('AIMSTREAM '): continue
            try: value=json.loads(line[10:])
            except ValueError: continue
            with self._lock:
                if 'reply' in value: self._responses[value['reply']]=value
                else: self._latest=value

    def _rpc(self,method,*args):
        self._counter+=1;identity=self._counter
        self.process.stdin.write(json.dumps({'id':identity,'method':method,'args':args,
            'minimumMatchPts':self.minimumMatchPts,'recordRaw':self.recordRaw})+'\n')
        self.process.stdin.flush()
        deadline=time.monotonic()+(120 if method=='initializeSdk' else 30)
        while time.monotonic()<deadline:
            with self._lock: value=self._responses.pop(identity,None)
            if value is not None:
                if 'error' in value: raise RuntimeError(value['error'])
                for key,item in value['state'].items(): setattr(self,key,item)
                return value['value']
            if self.process.poll() is not None: raise RuntimeError('SDK 采集进程已退出；查看 '+str(self.recordingPath.with_suffix('.log')))
            time.sleep(.005)
        raise TimeoutError('SDK operation timed out: '+method)

    def initializeSdk(self): self._rpc('initializeSdk')
    def connectDevice(self,interface,localIP='',deviceIP=''):
        return self._rpc('connectDevice',interface,localIP,deviceIP) if (localIP or deviceIP) else self._rpc('connectDevice',interface)
    def loadToolFiles(self,path): return self._rpc('loadToolFiles',str(path))
    def selectTools(self,tool,reference,calibrationTool=None): return self._rpc('selectTools',tool,reference,calibrationTool)
    def startTracking(self):
        result=self._rpc('startTracking')
        with self._lock: self._latest=None
        self.lastConsumedSequence=0
        return result
    def stopTracking(self):
        return self._rpc('stopTracking')
    def disconnectDevice(self):
        try:
            if self.process.poll() is None: self._rpc('disconnectDevice')
        finally:
            try: self.process.wait(timeout=5)
            except subprocess.TimeoutExpired: self.process.terminate();self.process.wait(timeout=5)
            self.connected=False;self.tracking=False
            self._stderr.close()

    def updateTracking(self):
        if self.process.poll() is not None: raise RuntimeError('独立采集进程意外退出')
        with self._lock: packet=self._latest
        if not packet or packet['sequence']==self.lastConsumedSequence: return None
        matrix_from_rows=self.matrix_from_rows
        for key,value in packet['state'].items(): setattr(self,key,value)
        self.poses={k:None if v is None else matrix_from_rows(v) for k,v in packet['poses'].items()}
        self.previewPoses={k:matrix_from_rows(v) for k,v in packet['previewPoses'].items()}
        self.receivedTime=packet['received'];self.acquisitionHz=packet['acquisitionHz']
        self.lastConsumedSequence=packet['sequence']
        return self.poses

if __name__=='__main__':
    sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
    worker(sys.argv[1],sys.argv[2])

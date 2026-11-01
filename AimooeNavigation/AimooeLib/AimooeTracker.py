"""Verified AimPosition 3.12 binding adapter. No Slicer/Qt dependencies."""
import importlib
import ipaddress
import math
import os
import sys
import time
from pathlib import Path
from .AimooeTransforms import pose_matrix
from .constants import STALE_SECONDS, LOST_SECONDS


class AimooeTracker:
    def __init__(self, runtime, logger):
        self.runtime = Path(runtime)
        self.log = logger
        self.ap = None
        self.handle = None
        self.dll_directory = None
        self.connected = False
        self.tracking = False
        self.names = []
        self.selected = {}
        self.previewPoses = {}
        self.previewUpdated = {}
        self.samples = {}
        self.allToolSamples = {}
        self.minimumMatchPts = 0  # Verified SDK: 0=all points; 3=experimental partial match
        self.poses = {'Tool': None, 'Reference': None, 'CalibrationTool': None}
        self.lastUpdateTime = {'Tool': None, 'Reference': None, 'CalibrationTool': None}
        self.last_frame = None
        self.marker_count = 0
        self.toolInfo = []
        self.quality_message = ""
        self.hardware_validated = False
        self.measured_markers = []
        self.rejected_frames = 0
        self.last_quality_log = float("-inf")
        self.role_reason = {}
        self.diagnostics = {'frames':0,'qualityReasons':{},'roles':{}}

    def check(self, code, operation):
        if code != self.ap.E_ReturnValue.AIMOOE_OK:
            self.log.error('%s returned %s', operation, code)
            raise RuntimeError(f'{operation}: {code}')

    def initializeSdk(self):
        if self.handle is not None:
            return
        if sys.version_info[:2] != (3, 12) or sys.maxsize <= 2**32:
            raise RuntimeError('AimPosition requires Python 3.12 x64')
        if not (self.runtime / 'AimPosition.pyd').is_file():
            raise FileNotFoundError(f'Missing {self.runtime / "AimPosition.pyd"}')
        self.dll_directory = os.add_dll_directory(str(self.runtime))
        if str(self.runtime) not in sys.path:
            sys.path.insert(0, str(self.runtime))
        try:
            self.ap = importlib.import_module('AimPosition')
            if Path(self.ap.__file__).resolve().parent != self.runtime.resolve():
                raise RuntimeError('AimPosition was loaded from another SDK directory; restart Slicer')
            self.version = self.ap.Aim_API_GetLibVersion()
            self.handle = self.ap.Aim_API_Initial()
            if self.handle is None:
                raise RuntimeError('Aim_API_Initial returned None')
            self.marker = self.ap.T_MarkerInfo()
            self.hardware = self.ap.T_AimPosStatusInfo()
            self.log.info('SDK initialized: %s', self.version)
        except Exception:
            self.log.exception('SDK initialization failed; check pyd and dependent DLLs')
            raise

    @staticmethod
    def parseIPv4(value, label):
        try:
            address=ipaddress.ip_address(str(value).strip())
        except ValueError:
            raise ValueError(label+'不是有效 IPv4 地址: '+str(value))
        if address.version!=4:
            raise ValueError(label+'必须是 IPv4 地址')
        return str(address),[int(part) for part in str(address).split('.')]

    def connectDevice(self, interface='Ethernet', localIP='', deviceIP=''):
        self.initializeSdk()
        choices = {'Ethernet': self.ap.E_Interface.I_ETHERNET,
                   'USB': self.ap.E_Interface.I_USB,
                   'WiFi': self.ap.E_Interface.I_WIFI}
        if interface not in choices:
            raise ValueError('Select Ethernet, USB or WiFi')
        local=None;device=None
        if interface in ('Ethernet','WiFi'):
            if localIP:local,_=self.parseIPv4(localIP,'本机 IP')
            if deviceIP:device,octets=self.parseIPv4(deviceIP,'定位仪 IP')
        self.networkConfiguration={'interface':interface,'localIP':local,'deviceIP':device,
            'sdkIPMode':('Aim_SetEthernetConnectIP' if interface=='Ethernet' else
                         'WiFi hotspot managed by SDK/OS' if interface=='WiFi' else 'USB')}
        self.interfaceName = interface
        self.interface = choices[interface]
        if interface=='Ethernet' and device:
            setter=getattr(self.ap,'Aim_SetEthernetConnectIP',None)
            if setter is None:
                raise RuntimeError('SDK 未导出 Aim_SetEthernetConnectIP，不能应用自定义定位仪 IP')
            self.check(setter(self.handle,*octets),'Aim_SetEthernetConnectIP')
        if interface=='WiFi':
            self.log.info('WiFi configuration requested: local=%s device=%s; vendor I_WIFI uses the AimPosition hotspot and exposes no IP binding API',local,device)
        code = self.ap.Aim_ConnectDevice(
            self.handle, self.interface, self.ap.T_AIMPOS_DATAPARA())
        if code != self.ap.E_ReturnValue.AIMOOE_OK:
            self.log.error('Aim_ConnectDevice(%s): %s', interface, code)
            raise RuntimeError(
                f'Aim_ConnectDevice({interface}): {code}; localIP={local}; deviceIP={device}. '
                'Disconnect Aimooe ToolBox first; verify cable, power and selected interface.')
        self.connected = True
        self.check(self.ap.Aim_SetToolFindRTDirection(
            self.handle, self.ap.E_RTDirection.FromToolToOpticalTrackingSystem),
            'Aim_SetToolFindRTDirection')
        self.log.info('Device connected: %s', interface)

    def loadToolFiles(self, path):
        if self.tracking:
            raise RuntimeError('Stop tracking before changing tool files')
        if self.handle is None or not self.connected:
            raise RuntimeError('Connect device before loading tools; disconnected SDK count may be an error code')
        folder = Path(path)
        if not folder.is_dir():
            raise FileNotFoundError(f'Tool directory not found: {folder}')
        normalized = folder.resolve().as_posix().rstrip('/') + '/'
        self.check(self.ap.Aim_SetToolInfoFilePath(self.handle, normalized),
                   'Aim_SetToolInfoFilePath')
        count = self.ap.Aim_GetCountOfToolInfo(self.handle)
        if count <= 0:
            raise RuntimeError('No AimTools loaded')
        info = self.ap.Aim_GetAllToolFilesBaseInfo(self.handle, count)
        if len(info) != count:
            raise RuntimeError(f'Aim_GetCountOfToolInfo={count}, Aim_GetAllToolFilesBaseInfo returned {len(info)} entries; SDK tool enumeration failed. See Logs/sdk-probe.txt.')
        self.toolInfo = info
        self.names = [entry['name'] for entry in info]
        if len(self.names) != count or len(set(self.names)) != count:
            raise RuntimeError('Invalid or duplicate tool names returned by SDK')
        self.log.info('Tools loaded from %s: %s', normalized, self.names)
        return self.names

    def selectTools(self, tool, reference=None, calibrationTool=None):
        requested=[name for name in (tool,reference,calibrationTool) if name]
        if not requested or not tool:
            raise ValueError('Navigation Tool is required')
        if len(set(requested)) != len(requested) or any(name not in self.names for name in requested):
            raise ValueError('Select distinct loaded tools for each role')
        self.selected = {'Tool': tool}
        if reference:
            self.selected['Reference'] = reference
        if calibrationTool:
            self.selected['CalibrationTool']=calibrationTool
        self.poses={role:None for role in self.selected}
        self.lastUpdateTime={role:None for role in self.selected}
        self.role_reason={role:'NOT_STARTED' for role in self.selected}
        self.diagnostics={'frames':0,'qualityReasons':{},'roles':{
            role:{'validFrames':0,'invalidFrames':0,'lossStarted':None,'longestLossSeconds':0.0}
            for role in self.selected}}

    def startTracking(self):
        if not self.connected or len(self.selected) not in (1,2,3) or 'Tool' not in self.selected:
            raise RuntimeError('Connect and select tools first')
        # DT_INFO uses SDK's acquisition thread. Never combine it with
        # Aim_GetMarkerAndStatusFromHardware (that requires DT_NONE).
        self.check(self.ap.Aim_SetAcquireData(self.handle, self.interface,
                                             (self.ap.E_DataType.DT_MARKER_INFO_WITH_WIFI
                                              if getattr(self, 'interfaceName', '') == 'WiFi'
                                              else self.ap.E_DataType.DT_INFO)), 'Aim_SetAcquireData')
        self.samples = {}
        self.allToolSamples = {}
        self.tracking = True
        self.hardware_validated = False
        self.quality_message = ""
        self.last_frame = None
        self.poses = dict.fromkeys(self.poses)
        self.lastUpdateTime = dict.fromkeys(self.lastUpdateTime)
        self.log.info('Tracking start: %s', self.selected)

    def stopTracking(self):
        self.previewPoses = {}
        self.previewUpdated = {}
        self.samples = {}
        self.allToolSamples = {}
        self.measured_markers = []
        if self.tracking:
            self.tracking = False
            self.poses = dict.fromkeys(self.poses)
            self.check(self.ap.Aim_SetAcquireData(self.handle, self.interface,
                                                 self.ap.E_DataType.DT_NONE), 'Aim_SetAcquireData(DT_NONE)')
            self.log.info('Hardware tracking stopped')

    def disconnectDevice(self):
        failures = []
        try:
            self.stopTracking()
        except Exception as exc:
            failures.append(str(exc))
            self.log.exception('Stop before disconnect failed')
        if self.handle is not None:
            try:
                # This Python 3.12 binary does NOT export Aim_DisconnectEthernet.
                # Close releases the entire handle for both USB and Ethernet.
                self.check(self.ap.Aim_API_Close(self.handle), 'Aim_API_Close')
            except Exception as exc:
                failures.append(str(exc))
                self.log.exception('SDK close failed')
            finally:
                self.handle = None
        self.connected = False
        self.tracking = False
        self.marker_count = 0
        self.measured_markers = []
        self.poses = dict.fromkeys(self.poses)
        if self.dll_directory:
            self.dll_directory.close()
            self.dll_directory = None
        self.log.info('Device disconnected / SDK closed')
        if failures:
            raise RuntimeError('; '.join(failures))

    def updateTracking(self):
        if not self.tracking:
            return None
        try:
            return self._readTrackingFrame()
        except Exception:
            self.samples = {}
            self.allToolSamples = {}
            self.previewPoses = {}
            self.previewUpdated = {}
            # Failed reads must never leave a previous pose accessible as valid.
            self.poses = dict.fromkeys(self.poses)
            self.measured_markers = []
            self.log.exception('Tracking frame failed; all poses invalidated')
            raise

    def _readTrackingFrame(self):
        code = self.ap.Aim_GetMarkerInfo(self.handle, self.interface, self.marker)
        if code == self.ap.E_ReturnValue.AIMOOE_NOT_REFLASH:
            return None
        self.check(code, 'Aim_GetMarkerInfo')
        if self.marker.ID == self.last_frame:
            return None
        self.previewPoses = {}
        self.samples = {}
        self.allToolSamples = {}
        self.last_frame = self.marker.ID
        self.marker_count = int(self.marker.MarkerNumber)
        self.measured_markers = []
        self.role_reason={role:'' for role in self.selected}
        status_code = (self.ap.E_ReturnValue.AIMOOE_NOT_REFLASH
                       if getattr(self, 'interfaceName', '') == 'WiFi'
                       else self.ap.Aim_GetStatusInfo(self.handle, self.interface, self.hardware))
        if status_code not in (self.ap.E_ReturnValue.AIMOOE_OK,
                               self.ap.E_ReturnValue.AIMOOE_NOT_REFLASH):
            self.check(status_code, 'Aim_GetStatusInfo')
        if status_code == self.ap.E_ReturnValue.AIMOOE_OK:
            self.hardware_validated = True
        reason = ''
        if not self.hardware_validated:
            reason = ('WiFi marker-only mode: hardware status unavailable'
                      if getattr(self, 'interfaceName', '') == 'WiFi'
                      else 'Waiting for fresh hardware status')
        elif self.hardware.HardwareStatus != self.ap.E_HardwareStatus.HW_OK:
            reason = f'Hardware status: {self.hardware.HardwareStatus}'
        elif self.marker.MarkerBGLightStatus != self.ap.E_BackgroundLightStatus.BG_LIGHT_OK:
            reason = f'Background light: {self.marker.MarkerBGLightStatus}'
        previous = self.quality_message
        self.quality_message = reason
        if reason:
            self.rejected_frames += 1
            for role in self.role_reason:self.role_reason[role]=reason
        now = time.monotonic()
        if reason != previous and now - self.last_quality_log >= 5.0:
            self.last_quality_log = now
            self.log.warning('Optical quality: %s; rejected frames=%s; device remains connected',
                             reason or 'RECOVERED', self.rejected_frames)
        # A quality alarm invalidates navigation, but does not imply no tool was
        # identified. Continue full matching for the separately labelled preview.
        if self.marker_count < 3:
            self.poses = dict.fromkeys(self.poses)
            for role in self.role_reason:self.role_reason[role]='INSUFFICIENT_MARKERS'
            self.recordDiagnostics(self.poses,reason or 'INSUFFICIENT_MARKERS')
            return self.poses
        result = self.ap.Aim_FindToolInfo_Map(
            self.handle, self.marker, self.ap.StringVector(self.names), 0 if reason else self.minimumMatchPts)
        poses = {}
        matched_indices = {}
        for role, name in self.selected.items():
            tool = result.get(name)
            poses[role] = None
            if tool is not None and tool.validflag:
                self.copySample(role, tool, reason)
                if reason:
                    indices = list(getattr(tool, 'toolptidx', []))
                    indices = [int(i) for i in indices if 0 <= i < self.marker_count]
                    if len(set(indices)) != 4:
                        continue
                    raw = [float(self.marker.MarkerCoordinate[3*i+j]) for i in indices for j in range(3)]
                    if not all(math.isfinite(v) for v in raw):
                        continue
                    try:
                        self.previewPoses[role] = pose_matrix(tool.Rto, tool.Tto)
                    except ValueError:
                        self.log.warning('Rejected malformed diagnostic pose for %s', role)
                        continue
                    matched_indices[role] = set(indices)
                    self.previewUpdated[role] = time.monotonic()
                    continue  # Never publish the diagnostic result as navigation.
                if self.minimumMatchPts == 3:
                    indices = list(getattr(tool, 'toolptidx', []))
                    indices = [int(i) for i in indices if 0 <= i < self.marker_count]
                    if len(set(indices)) < 3:
                        continue
                    matched_indices[role] = set(indices)
                    import numpy as np
                    points = np.array([[self.marker.MarkerCoordinate[3*i+j] for j in range(3)] for i in indices], dtype=float)
                    if not np.isfinite(points).all():
                        raise ValueError('Non-finite partial marker geometry')
                    singular = np.linalg.svd(points-points.mean(axis=0), compute_uv=False)
                    if singular[0] <= 0 or singular[1]/singular[0] < 0.1:
                        continue  # Near-collinear triplets amplify orientation noise.
                poses[role] = pose_matrix(tool.Rto, tool.Tto)
                self.lastUpdateTime[role] = time.monotonic()
                if role == 'Tool' and hasattr(tool, 'toolptidx'):
                    # SDK indices follow the tool-file marker order. Raw points
                    # are copied immediately; no borrowed native arrays escape.
                    for index in tool.toolptidx:
                        if 0 <= index < self.marker_count:
                            point = [float(self.marker.MarkerCoordinate[3*index+i]) for i in range(3)]
                            if not all(math.isfinite(value) for value in point):
                                raise ValueError('Non-finite SDK marker coordinate')
                            self.measured_markers.append(point)
                        else:
                            self.measured_markers.append(None)
            if poses[role] is None and not self.role_reason.get(role):
                self.role_reason[role]='TOOL_NOT_MATCHED'
        if self.minimumMatchPts == 3 and 'Reference' in self.selected and all(role in matched_indices for role in ('Tool','Reference')):
            if matched_indices['Tool'] & matched_indices['Reference']:
                poses = dict.fromkeys(poses)
                self.measured_markers = []
                self.quality_message = 'Ambiguous partial match: shared markers across tools'
                for role in ('Tool','Reference'):self.role_reason[role]='AMBIGUOUS_SHARED_MARKERS'
        if reason and 'Reference' in self.selected and all(role in matched_indices for role in ('Tool','Reference')):
            if matched_indices['Tool'] & matched_indices['Reference']:
                self.previewPoses = {}
        overlaps = [set(s['indices']) - {-1} for s in self.samples.values()]
        if any(overlaps[i] & overlaps[j] for i in range(len(overlaps)) for j in range(i+1,len(overlaps))):
            poses = dict.fromkeys(poses)
            self.quality_message = 'Ambiguous match: shared markers across tools'
            for role in self.role_reason:self.role_reason[role]='AMBIGUOUS_SHARED_MARKERS'
            for sample in self.samples.values():
                sample['ambiguous'] = True
        # Reuse this frame's one SDK matching call for the Live coordinates
        # panel. Keep all-file diagnostics separate from role-gated navigation.
        selectedByName={name:role for role,name in self.selected.items()}
        for name in self.names:
            tool=result.get(name)
            if tool is None or not tool.validflag:continue
            role=selectedByName.get(name)
            if role in self.samples:
                sample=dict(self.samples[role])
            else:
                transform=pose_matrix(tool.Rto,tool.Tto)
                indices=[int(i) for i in getattr(tool,'toolptidx',[])]
                markers=[]
                for index in indices:
                    point=([float(self.marker.MarkerCoordinate[3*index+j]) for j in range(3)]
                           if 0<=index<self.marker_count else None)
                    if point is not None and not all(math.isfinite(value) for value in point):
                        point=None
                    markers.append(point)
                sample={'name':name,'frame':int(self.last_frame),'monotonic':time.monotonic(),
                        'matrix':[[transform.GetElement(i,j) for j in range(4)] for i in range(4)],
                        'indices':indices,'markers':markers,'quality':reason,'ambiguous':False}
            self.allToolSamples[name]=sample
        names=list(self.allToolSamples)
        for i,name in enumerate(names):
            points=set(self.allToolSamples[name]['indices'])-{-1}
            for other in names[i+1:]:
                if points & (set(self.allToolSamples[other]['indices'])-{-1}):
                    self.allToolSamples[name]['ambiguous']=True
                    self.allToolSamples[other]['ambiguous']=True
        self.poses = poses
        self.recordDiagnostics(poses,reason)
        return poses

    def recordDiagnostics(self, poses, quality):
        now=time.monotonic()
        self.diagnostics['frames']+=1
        key=quality or 'OK'
        reasons=self.diagnostics['qualityReasons']
        reasons[key]=reasons.get(key,0)+1
        for role in self.selected:
            stats=self.diagnostics['roles'].setdefault(role,
                {'validFrames':0,'invalidFrames':0,'lossStarted':None,'longestLossSeconds':0.0})
            if poses.get(role) is not None:
                stats['validFrames']+=1
                if stats['lossStarted'] is not None:
                    stats['longestLossSeconds']=max(stats['longestLossSeconds'],now-stats['lossStarted'])
                    stats['lossStarted']=None
                self.role_reason[role]=''
            else:
                stats['invalidFrames']+=1
                if stats['lossStarted'] is None:stats['lossStarted']=now
                stats['longestLossSeconds']=max(stats['longestLossSeconds'],now-stats['lossStarted'])

    def copySample(self, role, tool, quality):
        # Owned, same-frame diagnostics. Never retain SDK/native array views.
        indices = [int(i) for i in getattr(tool, 'toolptidx', [])]
        points = []
        for i in indices:
            point = ([float(self.marker.MarkerCoordinate[3*i+j]) for j in range(3)]
                     if 0 <= i < self.marker_count else None)
            if point is not None and not all(math.isfinite(v) for v in point):
                raise ValueError('Non-finite SDK marker coordinate')
            points.append(point)
        def vector(name):
            values = getattr(tool, name, None)
            if values is None:
                return None
            values = [float(values[i]) for i in range(3)]
            return values if all(math.isfinite(v) for v in values) else None
        def scalar(name):
            value = getattr(tool, name, None)
            return float(value) if value is not None and math.isfinite(float(value)) else None
        matrix = pose_matrix(tool.Rto, tool.Tto)
        self.samples[role] = {
            'name': self.selected[role], 'frame': int(self.last_frame),
            'monotonic': time.monotonic(), 'unixTime': time.time(),
            'matrix': [[matrix.GetElement(i,j) for j in range(4)] for i in range(4)],
            'indices': indices, 'markers': points,
            'tipTool': vector('toolCstip'), 'tipTracker': vector('tooltip'),
            'meanErrorMm': scalar('MeanError'), 'rmsMm': scalar('Rms'),
            'quality': quality, 'ambiguous': False}

    def getToolPose(self):
        return self.getTrackingStatus()['Tool'] == 'TRACKING' and self.poses['Tool'] or None

    def getReferencePose(self):
        return self.getTrackingStatus().get('Reference') == 'TRACKING' and self.poses.get('Reference') or None

    def getTrackingStatus(self):
        result = {'Connected': self.connected}
        now = time.monotonic()
        for role, updated in self.lastUpdateTime.items():
            age = float('inf') if updated is None else now - updated
            result[role] = ('LOST' if not self.tracking or self.poses.get(role) is None or age >= LOST_SECONDS
                            else 'STALE' if age >= STALE_SECONDS else 'TRACKING')
        return result

import logging
import tempfile
import time
import unittest
from types import SimpleNamespace as NS
from pathlib import Path
from AimooeLib.AimooeTracker import AimooeTracker

class TrackerTests(unittest.TestCase):
    def setUp(self):
        self.t = AimooeTracker('.', logging.getLogger('test'))
        pose = NS(validflag=True, Rto=[[1,0,0],[0,1,0],[0,0,1]], Tto=[1,2,3])
        self.code = 0
        self.result = {'one': pose, 'two': pose}
        self.t.ap = NS(
            E_ReturnValue=NS(AIMOOE_OK=0, AIMOOE_NOT_REFLASH=6),
            E_DataType=NS(DT_NONE=0, DT_INFO=1),
            E_HardwareStatus=NS(HW_OK=0),
            E_BackgroundLightStatus=NS(BG_LIGHT_OK=0),
            Aim_GetMarkerInfo=lambda *args: self.code,
            Aim_GetStatusInfo=lambda *args: 0,
            Aim_FindToolInfo_Map=lambda *args: self.result,
            StringVector=lambda value: value,
            Aim_SetAcquireData=lambda *args: 0,
            Aim_API_Close=lambda *args: 0)
        self.t.handle = object()
        self.t.interface = 1
        self.t.marker = NS(ID=1, MarkerBGLightStatus=0, MarkerNumber=8, MarkerCoordinate=list(range(24)))
        self.t.hardware = NS(HardwareStatus=0)
        self.t.connected = self.t.tracking = True
        self.t.selected = {'Tool': 'one', 'Reference': 'two'}
        self.t.names = ['one', 'two']

    def test_valid_conversion(self):
        poses = self.t.updateTracking()
        self.assertEqual(poses['Tool'].GetElement(2,3), 3)
        self.assertEqual(self.t.getTrackingStatus()['Reference'], 'TRACKING')

    def test_no_refresh_preserves_timestamp(self):
        self.t.updateTracking()
        updated = self.t.lastUpdateTime.copy()
        self.code = 6
        self.assertIsNone(self.t.updateTracking())
        self.assertEqual(self.t.lastUpdateTime, updated)

    def test_duplicate_frame(self):
        self.t.updateTracking()
        self.assertIsNone(self.t.updateTracking())

    def test_reference_occlusion(self):
        self.result.pop('two')
        poses = self.t.updateTracking()
        self.assertIsNone(poses['Reference'])
        self.assertEqual(self.t.getTrackingStatus()['Reference'], 'LOST')

    def test_stale_lost(self):
        self.t.updateTracking()
        self.t.lastUpdateTime['Tool'] = time.monotonic() - 0.6
        self.assertEqual(self.t.getTrackingStatus()['Tool'], 'STALE')
        self.assertIsNone(self.t.getToolPose())
        self.t.lastUpdateTime['Tool'] -= 2
        self.assertEqual(self.t.getTrackingStatus()['Tool'], 'LOST')

    def test_device_error(self):
        self.code = 2
        with self.assertRaisesRegex(RuntimeError, 'Aim_GetMarkerInfo'):
            self.t.updateTracking()

    def test_selection(self):
        with self.assertRaises(ValueError):
            self.t.selectTools('one', 'one')

    def test_optional_calibration_tool_role(self):
        self.t.names=['one','two','calibration']
        self.t.selectTools('one','two','calibration')
        self.assertEqual(self.t.selected,
                         {'Tool':'one','Reference':'two','CalibrationTool':'calibration'})
        self.assertEqual(set(self.t.poses),set(self.t.selected))

    def test_single_navigation_tool_role(self):
        self.t.selectTools('one',None)
        self.assertEqual(self.t.selected,{'Tool':'one'})
        self.t.startTracking()
        self.assertIsNotNone(self.t.updateTracking()['Tool'])

    def test_ipv4_validation(self):
        self.assertEqual(self.t.parseIPv4('192.0.2.10','device'),
                         ('192.0.2.10',[192,0,2,10]))
        with self.assertRaises(ValueError):
            self.t.parseIPv4('192.0.2.999','device')

    def test_disconnect(self):
        self.t.disconnectDevice()
        self.assertFalse(self.t.connected)
        self.assertIsNone(self.t.handle)

    def test_tool_enumeration_contract(self):
        self.t.tracking = False
        self.t.ap.Aim_SetToolInfoFilePath = lambda *args: 0
        self.t.ap.Aim_GetCountOfToolInfo = lambda *args: 2
        self.t.ap.Aim_GetAllToolFilesBaseInfo = lambda *args: []
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(RuntimeError, 'enumeration failed'):
                self.t.loadToolFiles(folder)
            self.t.ap.Aim_GetAllToolFilesBaseInfo = lambda *args: [{'name':'one'}, {'name':'two'}]
            self.assertEqual(self.t.loadToolFiles(folder), ['one','two'])

    def test_load_requires_connection(self):
        self.t.tracking = False
        self.t.connected = False
        with self.assertRaisesRegex(RuntimeError, 'Connect device before'):
            self.t.loadToolFiles('.')

    def test_marker_diagnostics(self):
        self.t.updateTracking()
        self.assertEqual(self.t.marker_count, 8)
        self.assertEqual(self.t.last_frame, 1)

    def test_background_rejects_and_recovers(self):
        self.t.marker.MarkerBGLightStatus = 1
        self.assertEqual(self.t.updateTracking(), {'Tool': None, 'Reference': None})
        self.assertTrue(self.t.connected)
        self.assertTrue(self.t.quality_message)
        self.t.marker.ID += 1
        self.t.marker.MarkerBGLightStatus = 0
        self.assertIsNotNone(self.t.updateTracking()['Tool'])
        self.assertEqual(self.t.quality_message, '')

    def test_waits_for_hardware_status(self):
        self.t.ap.Aim_GetStatusInfo = lambda *args: 6
        self.assertIsNone(self.t.updateTracking()['Tool'])
        self.assertIn('Waiting', self.t.quality_message)


    def test_marker_index_order_and_copy(self):
        self.result['one'].toolptidx = [3, 0, 2, -1]
        self.t.marker.MarkerCoordinate = list(range(24))
        self.t.updateTracking()
        self.assertEqual(self.t.measured_markers, [[9.,10.,11.], [0.,1.,2.], [6.,7.,8.], None])
        self.t.marker.MarkerCoordinate[9] = 1000
        self.assertEqual(self.t.measured_markers[0][0], 9)

    def test_nonfinite_marker_invalidates_previous_pose(self):
        self.t.updateTracking()
        self.t.marker.ID += 1
        self.result['one'].toolptidx = [0]
        self.t.marker.MarkerCoordinate = [float('nan'), 0, 0]
        with self.assertRaisesRegex(ValueError, 'Non-finite SDK marker'):
            self.t.updateTracking()
        self.assertIsNone(self.t.getToolPose())
        self.assertEqual(self.t.measured_markers, [])

    def test_invalid_rotation_invalidates_previous_pose(self):
        self.t.updateTracking()
        self.t.marker.ID += 1
        self.result['one'].Rto = [[0,0,0], [0,0,0], [0,0,0]]
        with self.assertRaises(ValueError):
            self.t.updateTracking()
        self.assertIsNone(self.t.getToolPose())
        self.assertIsNone(self.t.getReferencePose())

    def test_quality_failure_clears_measured_points(self):
        self.result['one'].toolptidx = [0]
        self.t.marker.MarkerCoordinate = [1,2,3]
        self.t.updateTracking()
        self.t.marker.ID += 1
        self.t.hardware.HardwareStatus = 99
        self.t.updateTracking()
        self.assertEqual(self.t.measured_markers, [])
        self.assertIsNone(self.t.getToolPose())


    def test_quality_chatter_is_rate_limited_without_accepting_bad_frames(self):
        from unittest.mock import Mock
        self.t.log = Mock()
        for i in range(30):
            self.t.marker.ID = i + 1
            self.t.marker.MarkerBGLightStatus = i % 2
            pose = self.t.updateTracking()['Tool']
            if i % 2:
                self.assertIsNone(pose)
            else:
                self.assertIsNotNone(pose)
        self.assertEqual(self.t.rejected_frames, 15)
        self.assertLessEqual(self.t.log.warning.call_count, 1)
        self.assertTrue(self.t.connected)

    def test_partial_three_nondegenerate_points(self):
        self.t.minimumMatchPts = 3
        self.result.pop('two')
        self.result['one'].toolptidx = [0, 1, 2, -1]
        self.t.marker.MarkerNumber = 3
        self.t.marker.MarkerCoordinate = [0,0,0, 50,0,0, 0,50,0]
        self.assertIsNotNone(self.t.updateTracking()['Tool'])

    def test_two_points_never_pose(self):
        self.t.minimumMatchPts = 3
        self.t.marker.MarkerNumber = 2
        self.assertIsNone(self.t.updateTracking()['Tool'])
        self.assertTrue(self.t.connected)

    def test_partial_collinear_rejected(self):
        self.t.minimumMatchPts = 3
        self.result.pop('two')
        self.result['one'].toolptidx = [0,1,2,-1]
        self.t.marker.MarkerNumber = 3
        self.t.marker.MarkerCoordinate = [0,0,0,50,0,0,100,0.01,0]
        self.assertIsNone(self.t.updateTracking()['Tool'])

    def test_partial_shared_markers_rejected(self):
        self.t.minimumMatchPts = 3
        self.result['one'].toolptidx = [0,1,2,-1]
        self.t.marker.MarkerNumber = 3
        self.t.marker.MarkerCoordinate = [0,0,0,50,0,0,0,50,0]
        self.assertEqual(self.t.updateTracking(), {'Tool':None,'Reference':None})

    def test_default_uses_full_match(self):
        arguments=[]
        self.t.ap.Aim_FindToolInfo_Map=lambda *args: arguments.append(args[-1]) or self.result
        self.t.updateTracking()
        self.assertEqual(arguments,[0])

    def test_quality_diagnostic_never_becomes_navigation(self):
        self.result.pop('two')
        self.result['one'].toolptidx=[0,1,2,3]
        self.t.marker.MarkerCoordinate=[0,0,0,50,0,0,0,50,0,20,40,0]
        self.t.hardware.HardwareStatus=99
        poses=self.t.updateTracking()
        self.assertIsNone(poses['Tool'])
        self.assertIsNone(self.t.getToolPose())
        self.assertIn('Tool',self.t.previewPoses)
        self.assertEqual(self.t.getTrackingStatus()['Tool'],'LOST')
        self.t.marker.ID+=1
        self.result.clear()
        self.t.updateTracking()
        self.assertEqual(self.t.previewPoses,{})

    def test_alarm_partial_result_not_previewed(self):
        self.result.pop('two')
        self.result['one'].toolptidx=[0,1,2,-1]
        self.t.hardware.HardwareStatus=99
        self.t.updateTracking()
        self.assertEqual(self.t.previewPoses,{})

    def test_ambiguous_diagnostic_shared_points_rejected(self):
        self.result['one'].toolptidx=[0,1,2,3]
        self.t.marker.MarkerCoordinate=[0,0,0,50,0,0,0,50,0,20,40,0]
        self.t.hardware.HardwareStatus=99
        self.t.updateTracking()
        self.assertEqual(self.t.previewPoses,{})


class StudioTrackerTests(unittest.TestCase):
    setUp = TrackerTests.setUp
    def test_wifi_uses_marker_only_acquisition(self):
        self.t.interfaceName = 'WiFi'
        self.t.ap.E_DataType.DT_MARKER_INFO_WITH_WIFI = 2
        calls = []
        self.t.ap.Aim_SetAcquireData = lambda *args: calls.append(args[-1]) or 0
        self.t.startTracking()
        self.assertEqual(calls, [2])
        self.t.ap.Aim_GetStatusInfo = lambda *args: self.fail('Marker-only WiFi must not request hardware status')
        self.t.updateTracking()
        self.assertIn('unavailable', self.t.quality_message)

    def test_same_frame_metrics_and_tip_are_copies(self):
        tool = self.result['one']
        tool.toolptidx = [0,1,2,3]
        tool.toolCstip = [10,20,30]
        tool.tooltip = [11,22,33]
        tool.MeanError = .12
        tool.Rms = .23
        self.result.pop('two')
        self.t.updateTracking()
        sample = self.t.samples['Tool']
        self.assertEqual(sample['frame'],1)
        self.assertEqual(sample['tipTracker'],[11,22,33])
        self.assertEqual(sample['meanErrorMm'],.12)
        tool.tooltip[0] = 999
        self.t.marker.MarkerCoordinate[0] = 999
        self.assertEqual(sample['tipTracker'][0],11)
        self.assertEqual(sample['markers'][0][0],0)
        self.code = 2
        with self.assertRaises(RuntimeError):
            self.t.updateTracking()
        self.assertEqual(self.t.samples,{})

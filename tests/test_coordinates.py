import unittest
import vtk
from AimooeLib.AimooeTransforms import pose_matrix
from AimooeLib.AimooeCoordinates import coordinate_snapshot
I = [[1,0,0],[0,1,0],[0,0,1]]

class CoordinateTests(unittest.TestCase):
    def test_transform_chain(self):
        tip = pose_matrix(I, [1,2,3])
        tool = pose_matrix([[0,-1,0],[1,0,0],[0,0,1]], [10,20,30])
        ref = pose_matrix(I, [5,6,7])
        reg = pose_matrix(I, [100,200,300])
        result = coordinate_snapshot(tip,tool,ref,True,True,reg,True,source='CAD_NOMINAL_UNCALIBRATED')
        rows = {r['space']:r['xyz'] for r in result['rows']}
        self.assertEqual(rows['Tracker'], [8,21,33])
        self.assertEqual(rows['Reference'], [3,15,26])
        self.assertEqual(rows['Image'], [103,215,326])
        self.assertIn('UNCALIBRATED',result['source'])

    def test_missing_reference_and_registration(self):
        m = vtk.vtkMatrix4x4()
        result = coordinate_snapshot(m,m,m,True,False)
        self.assertIsNotNone(result['rows'][2]['xyz'])
        self.assertIsNone(result['rows'][3]['xyz'])
        self.assertIsNone(result['rows'][4]['xyz'])

    def test_missing_tool_blocks_stale_world_coordinates(self):
        m = pose_matrix(I, [10,20,30])
        result = coordinate_snapshot(m,m,m,False,True,m,True)
        self.assertIsNotNone(result['rows'][1]['xyz'])
        for row in result['rows'][2:]:
            self.assertIsNone(row['xyz'])

    def test_unregistered_identity_is_not_registration(self):
        m = vtk.vtkMatrix4x4()
        result = coordinate_snapshot(m,m,m,True,True,m,False)
        self.assertIsNone(result['rows'][4]['xyz'])

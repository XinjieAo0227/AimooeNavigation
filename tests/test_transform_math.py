import unittest
import vtk
from AimooeLib.AimooeTransforms import pose_matrix, relative_matrix

I = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

class TransformTests(unittest.TestCase):
    def test_tool_moves(self):
        result = relative_matrix(pose_matrix(I, [8, 0, 0]), pose_matrix(I, [2, 0, 0]))
        self.assertAlmostEqual(result.GetElement(0, 3), 6)

    def test_reference_moves(self):
        result = relative_matrix(pose_matrix(I, [8, 0, 0]), pose_matrix(I, [3, 0, 0]))
        self.assertAlmostEqual(result.GetElement(0, 3), 5)

    def test_common_translation(self):
        r = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        a = relative_matrix(pose_matrix(I, [8, 4, 3]), pose_matrix(r, [2, 1, 0]))
        b = relative_matrix(pose_matrix(I, [18, 24, 33]), pose_matrix(r, [12, 21, 30]))
        for i in range(4):
            for j in range(4):
                self.assertAlmostEqual(a.GetElement(i, j), b.GetElement(i, j))
        self.assertAlmostEqual(a.GetElement(0, 3), 3)
        self.assertAlmostEqual(a.GetElement(1, 3), -6)

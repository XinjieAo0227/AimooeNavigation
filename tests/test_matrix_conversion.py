import unittest
from AimooeLib.AimooeTransforms import pose_matrix

class ConversionTests(unittest.TestCase):
    def test_conversion(self):
        r = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        m = pose_matrix(r, [4, 5, 6])
        self.assertEqual(m.MultiplyPoint([1, 0, 0, 1]), (4, 6, 6, 1))
    def test_invalid(self):
        for r, t in [([[1,0,0],[0,1,0],[0,0,1]], [float('nan'),0,0]),
                     ([[-1,0,0],[0,1,0],[0,0,1]], [0,0,0])]:
            with self.assertRaises(ValueError):
                pose_matrix(r,t)

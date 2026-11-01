import unittest
import numpy as np
from AimooeLib.AimooeNavigationCore import repeatability_statistics, absolute_accuracy_statistics, error_statistics

class PrecisionTests(unittest.TestCase):
    def test_known_repeatability(self):
        result=repeatability_statistics([[0,0,0],[2,0,0]])
        self.assertEqual(result['count'],2)
        self.assertAlmostEqual(result['rms'],1,places=12)
        np.testing.assert_allclose(result['centroid'],[1,0,0],atol=1e-12)
    def test_independent_truth(self):
        result=absolute_accuracy_statistics([[0,0,0],[2,0,0]],[[0,0,0],[1,0,0]])
        self.assertAlmostEqual(result['mean'],0.5,places=12)
        self.assertAlmostEqual(result['rms'],np.sqrt(0.5),places=12)
    def test_translation_invariance(self):
        a=np.array([[0,0,0],[2,1,0],[0,0,4]],float)
        self.assertAlmostEqual(repeatability_statistics(a)['rms'],repeatability_statistics(a+12)['rms'],places=12)
    def test_rejects_invalid_samples(self):
        for a in [[[1,2]],[[np.nan,0,0]],[[np.inf,0,0]]]:
            with self.assertRaises(ValueError): repeatability_statistics(a)
        with self.assertRaises(ValueError): absolute_accuracy_statistics([[0,0,0]],[[0,0,0],[1,0,0]])
        with self.assertRaises(ValueError): error_statistics([-1])
    def test_empty_and_singleton(self):
        self.assertIsNone(repeatability_statistics([]))
        self.assertEqual(repeatability_statistics([[1,2,3]])['axisStd'],[0,0,0])

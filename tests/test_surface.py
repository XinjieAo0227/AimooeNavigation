"""Standalone VTK tests: underconstrained inputs and rigid surface recovery."""
import sys
import unittest
from pathlib import Path
import numpy as np
import vtk

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'AimooeNavigation'))
from AimooeLib.AimooeRegistration import refine


class SurfaceRegistrationTests(unittest.TestCase):
    def setUp(self):
        sphere=vtk.vtkSphereSource()
        sphere.SetRadius(30)
        sphere.SetThetaResolution(32)
        sphere.SetPhiResolution(24)
        sphere.Update()
        self.mesh=sphere.GetOutput()

    def test_too_few(self):
        with self.assertRaises(ValueError):
            refine([[0,0,0]]*3,self.mesh,np.eye(4))

    def test_collinear(self):
        with self.assertRaises(ValueError):
            refine([[i,0,0] for i in range(30)],self.mesh,np.eye(4))

    def test_nonfinite(self):
        with self.assertRaises(ValueError):
            refine([[float('nan'),0,0]]*30,self.mesh,np.eye(4))

    def test_empty_surface(self):
        points=[self.mesh.GetPoint(i) for i in range(0,self.mesh.GetNumberOfPoints(),8)]
        with self.assertRaises(ValueError):
            refine(points,vtk.vtkPolyData(),np.eye(4))

    def test_recovers_small_rigid_translation(self):
        # An asymmetric ellipsoid prevents a spherical rotation degeneracy.
        scale=vtk.vtkTransform()
        scale.Scale(1.,0.7,1.3)
        filter=vtk.vtkTransformPolyDataFilter()
        filter.SetInputData(self.mesh)
        filter.SetTransform(scale)
        filter.Update()
        mesh=filter.GetOutput()
        truth=np.array([0.3,-0.2,0.1])
        points=np.array([mesh.GetPoint(i) for i in range(0,mesh.GetNumberOfPoints(),3)])-truth
        result=refine(points,mesh,np.eye(4))
        self.assertLess(result['after']['rms'],result['before']['rms'])
        np.testing.assert_allclose(np.array(result['matrix'])[:3,3],truth,atol=0.08,rtol=0)


if __name__=='__main__':
    unittest.main()

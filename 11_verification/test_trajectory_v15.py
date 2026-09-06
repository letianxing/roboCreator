import importlib.util, json, unittest, hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('dyn',ROOT/'05_dynamics/trajectory_screen_v15.py');d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)
class DynamicsTests(unittest.TestCase):
    def setUp(self):
        self.q=np.array([.4,-.9,.3]);self.v=np.array([.3,-.2,.1]);self.a=np.array([.1,.4,-.3]);self.L=[.09,.09,.16];self.m=[.308,.308,1.088]
    def test_jacobian_finite_difference(self):
        pos,J,_,_=d.kin(self.q,self.v,self.a,self.L,2,.5);eps=1e-6
        for i in range(3):
            step=np.eye(3)[i]*eps
            diff=(d.kin(self.q+step,self.v,self.a,self.L,2,.5)[0]-d.kin(self.q-step,self.v,self.a,self.L,2,.5)[0])/(2*eps)
            np.testing.assert_allclose(J[:,i],diff,atol=1e-9)
    def test_gravity_potential_gradient_with_offset(self):
        z=np.zeros(3);eps=1e-6;off=[0,0,.015]
        tau=d.dynamics(self.q,z,z,self.L,self.m,com_offsets=off)[0]
        for i in range(3):
            e=np.eye(3)[i]*eps
            grad=(d.dynamics(self.q+e,z,z,self.L,self.m,com_offsets=off)[3]-d.dynamics(self.q-e,z,z,self.L,self.m,com_offsets=off)[3])/(2*eps)
            self.assertAlmostEqual(tau[i],grad,places=7)
    def test_mass_matrix_symmetric_positive(self):
        z=np.zeros(3);gravity=d.dynamics(self.q,z,z,self.L,self.m)[0]
        M=np.column_stack([d.dynamics(self.q,z,np.eye(3)[i],self.L,self.m)[0]-gravity for i in range(3)])
        np.testing.assert_allclose(M,M.T,atol=1e-12);self.assertGreater(np.linalg.eigvalsh(M).min(),0)
    def test_power_and_moment_identity(self):
        tau,F,M,E,res=d.dynamics(self.q,self.v,self.a,self.L,self.m,.29,[0,0,.015])
        self.assertLess(abs(res),1e-12);self.assertAlmostEqual(tau[0],M,places=12)
    def test_analytic_point_acceleration(self):
        eps=1e-4
        pos,_,_,acc=d.kin(self.q,self.v,self.a,self.L,2,1)
        forward=d.kin(self.q+self.v*eps+.5*self.a*eps**2,self.v,self.a,self.L,2,1)[0]
        back=d.kin(self.q-self.v*eps+.5*self.a*eps**2,self.v,self.a,self.L,2,1)[0]
        np.testing.assert_allclose((forward-2*pos+back)/eps**2,acc,atol=1e-7)
    def test_rest_to_rest_endpoints(self):
        t,q,v,a=d.trajectory(self.q,self.q+.2,.5,801)
        np.testing.assert_allclose(v[[0,-1]],0,atol=1e-10);np.testing.assert_allclose(a[[0,-1]],0,atol=1e-10)
    def test_generated_energy_and_provenance(self):
        s=json.loads((ROOT/'05_dynamics/trajectory_v15/summary.json').read_text())
        self.assertEqual(s['input_sha256'],hashlib.sha256((ROOT/'03_system/layout_load_inputs_v15.json').read_bytes()).hexdigest())
        self.assertEqual(len(s['cases']),12)
        for c in s['cases']:
            self.assertLess(abs(c['work_energy_residual_J']),1e-7);self.assertFalse(c['task_verified'])
    def test_time_resolution_of_leg_peak(self):
        peaks=[]
        for N in (401,801,1601):
            t,q,v,a=d.trajectory(np.array([1,-2,1])*np.deg2rad(60),np.array([1,-2,1])*np.deg2rad(20),.25,N)
            peaks.append(max(abs(d.dynamics(qi,vi,ai,self.L,self.m,com_offsets=[0,0,.015])[0][1]) for qi,vi,ai in zip(q,v,a)))
        self.assertLess(abs(peaks[-1]-peaks[-2])/peaks[-1],1e-4)
if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(DynamicsTests))
    record={'tests_run':result.testsRun,'successful':result.wasSuccessful(),'failures':[str(x) for x,_ in result.failures],
            'errors':[str(x) for x,_ in result.errors],'input_sha256':hashlib.sha256((ROOT/'03_system/layout_load_inputs_v15.json').read_bytes()).hexdigest(),
            'scope':'Planar mathematical implementation only, not task validation'}
    (ROOT/'11_verification/trajectory_tests_v15.json').write_text(json.dumps(record,indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)

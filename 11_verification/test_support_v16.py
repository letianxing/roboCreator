import json,math,hashlib,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];P=ROOT/'03_system/support_mechanisms_v16.json';G=ROOT/'09_cad/support_mechanisms_v16/mechanism_results.json';D=ROOT/'05_dynamics/support_load_paths_v16.json'
class SupportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p=json.loads(P.read_text());cls.g=json.loads(G.read_text());cls.d=json.loads(D.read_text());cls.h=hashlib.sha256(P.read_bytes()).hexdigest()
    def test_provenance(self):self.assertEqual(self.g['input_sha256'],self.h);self.assertEqual(self.d['input_sha256'],self.h)
    def test_saved_pose_count_and_no_unexpected_intersections(self):
        self.assertEqual(len(self.g['records']),14);self.assertTrue(all(not x['unexpected_intersections'] for x in self.g['records']))
    def test_all_files_exist(self):
        for row in self.g['records']:
            for name in row['files']:self.assertGreater((ROOT/'09_cad/support_mechanisms_v16'/name).stat().st_size,100)
    def test_neutral_module_height(self):
        row=next(x for x in self.g['records'] if x['pose']=='ankle_p0_p0');self.assertLessEqual(row['bounds_mm'][5],117+1e-9);self.assertGreaterEqual(row['bounds_mm'][2],-2-1e-9)
        self.assertAlmostEqual(self.g['bilateral_neutral_bounds']['outside_width_mm'],185);self.assertGreater(self.g['bilateral_neutral_bounds']['inside_gap_mm'],0)
        self.assertAlmostEqual(self.g['clearance_to_nominal_knee_axis_z_mm'],8)
    def test_linkage_work_ratio(self):
        for x in self.d['ankle_linkage']['rows']:
            self.assertAlmostEqual(x['rod_axial_force_N']*x['effective_lever_m'],x['torque_Nm'],places=12)
    def test_parallel_crank_constant_rod_length(self):
        a=self.p['ankle'];sep=a['roll_motor_axis_xyz_mm'][2]-a['roll_axis_xyz_mm'][2];self.assertAlmostEqual(sep,a['rod_nominal_length_mm'])
        r=a['crank_radius_mm']
        for q in range(-25,26):
            off=(r*math.cos(math.radians(q)),r*math.sin(math.radians(q)))
            self.assertAlmostEqual(math.dist((off[0],a['roll_axis_xyz_mm'][2]+off[1]),(off[0],a['roll_motor_axis_xyz_mm'][2]+off[1])),sep)
    def test_contact_balance_nonnegative(self):
        for x in self.d['gauntlet_contact']['rows']:
            self.assertAlmostEqual(x['gauntlet_force_N']+x['palm_force_N'],x['total_per_arm_vertical_N']);self.assertGreaterEqual(x['palm_force_N'],0);self.assertGreaterEqual(x['gauntlet_force_N'],0)
    def test_bypass_reduces_reference_wrist_moment(self):
        h=self.p['forearm_support']
        for x in self.d['gauntlet_contact']['rows']:
            no_bypass=x['total_per_arm_vertical_N']*h['palm_contact_center_x_mm']/1000
            self.assertLessEqual(x['wrist_torque_from_palm_Nm'],no_bypass+1e-12)
    def test_buckling_formula(self):
        a=self.p['ankle'];d=a['rod_root_diameter_mm']/1000;L=a['rod_nominal_length_mm']/1000;E=a['load_cases']['steel_E_GPa']*1e9;I=math.pi*d**4/64
        expected=math.pi**2*E*I/L**2;self.assertAlmostEqual(expected,self.d['ankle_linkage']['ideal_rod_buckling_load_N'])
    def test_no_task_pass(self):self.assertFalse(self.d['task_verified']);self.assertTrue(all(not x['manufacturable'] for x in self.g['records']))
if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(SupportTests))
    rec={'tests_run':result.testsRun,'successful':result.wasSuccessful(),'failures':[str(x) for x,_ in result.failures],'errors':[str(x) for x,_ in result.errors],'input_sha256':hashlib.sha256(P.read_bytes()).hexdigest(),'scope':'local mechanism arithmetic/geometry regression only'}
    (ROOT/'11_verification/support_tests_v16.json').write_text(json.dumps(rec,indent=2)+'\n');raise SystemExit(0 if result.wasSuccessful() else 1)

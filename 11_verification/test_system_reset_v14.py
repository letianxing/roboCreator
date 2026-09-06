"""Read-only tests of the v14 screening model; no hardware commands."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("review", ROOT / "05_dynamics/system_reset_v14.py")
review = importlib.util.module_from_spec(spec)
spec.loader.exec_module(review)


class SystemReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p = json.loads((ROOT / "03_system/mass_reset_v14.json").read_text(encoding="utf-8"))
        cls.b = json.loads((ROOT / cls.p["baseline_file"]).read_text(encoding="utf-8"))
        cls.r = review.evaluate(cls.p, cls.b)

    def test_mass_reference_not_rejection(self):
        self.assertFalse(self.r["candidates"][1]["arithmetic_within_ceiling"])
        self.assertFalse(self.r["candidates"][1]["mass_exceedance_rejects_candidate"])

    def test_bad_count_detected(self):
        p = copy.deepcopy(self.p)
        p["candidates"][0]["counts"]["XM430"] += 1
        with self.assertRaises(AssertionError):
            review.evaluate(p, self.b)

    def test_bad_budget_detected(self):
        p = copy.deepcopy(self.p)
        p["allocations"][0]["mass_kg"] += 0.1
        with self.assertRaises(AssertionError):
            review.evaluate(p, self.b)

    def test_payload_zero(self):
        p = copy.deepcopy(self.p)
        p["screening"]["water_mass_kg"] = 0
        p["screening"]["bottle_packaging_mass_kg"] = 0
        r = review.evaluate(p, self.b)
        self.assertLess(max(c["combined_com_x_m"] for c in r["static_com_cases"]), 1e-12)
        self.assertTrue(all(c["payload_x_max_for_margin_m"] is None for c in r["static_com_cases"]))

    def test_force_linear_in_mass(self):
        cases = self.r["vertical_jump_cases"]
        for c in cases:
            self.assertAlmostEqual(c["force_per_foot_equal_split_N"] / c["robot_mass_kg"],
                                   9.81 * (1 + 0.1 / c["stroke_m"]) / 2)

    def test_longer_stop_less_average_force(self):
        cases = self.r["vertical_landing_cases"][:3]
        self.assertGreater(cases[0]["displacement_average_total_force_N"], cases[1]["displacement_average_total_force_N"])
        self.assertGreater(cases[1]["displacement_average_total_force_N"], cases[2]["displacement_average_total_force_N"])
        self.assertTrue(all(c["peak_force_N"] is None for c in cases))

    def test_no_hardware_pass_from_mass(self):
        self.assertTrue(self.r["candidates"][2]["arithmetic_within_ceiling"])
        self.assertIsNone(self.r["candidates"][2]["hardware_feasible"])
        self.assertFalse(self.r["engineering_validated"])


if __name__ == "__main__":
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(SystemReviewTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    evidence_path = ROOT / "11_verification/system_reset_tests_v14.json"
    record = {"tests_run": result.testsRun, "successful": result.wasSuccessful(),
              "failures": [str(t) for t, _ in result.failures], "errors": [str(t) for t, _ in result.errors],
              "input_sha256": review.sha(ROOT / "03_system/mass_reset_v14.json"),
              "test_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "scope": "analytical regression only, not real robot validation"}
    evidence_path.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    if result.wasSuccessful():
        gate_path = ROOT / "11_verification/v14_checkpoint_checks.json"
        gates = json.loads(gate_path.read_text(encoding="utf-8"))
        name = "Seven independent regression and invalid-input tests"
        gates["checks"] = [r for r in gates["checks"] if r["name"] != name]
        gates["checks"].append({"name": name, "status": "passed", "evidence": [
            "11_verification/test_system_reset_v14.py", "11_verification/system_reset_tests_v14.json"]})
        gate_path.write_text(json.dumps(gates, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    raise SystemExit(0 if result.wasSuccessful() else 1)

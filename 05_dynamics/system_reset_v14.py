"""Reproducible system mass and analytical sensitivity review; no CAD mutation."""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INPUT = ROOT / "03_system/mass_reset_v14.json"
OUTPUT = ROOT / "05_dynamics/system_reset_results_v14.json"
GATE = ROOT / "11_verification/v14_checkpoint_checks.json"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def close(a, b):
    if not math.isclose(a, b, abs_tol=1e-9, rel_tol=1e-9):
        raise AssertionError((a, b))


def evaluate(p, baseline):
    b = next(r for r in baseline["results"] if r["pose"] == "standing")
    categories = {}
    for c in b["components"]:
        categories[c["category"]] = categories.get(c["category"], 0) + c["mass_kg"]
    close(sum(categories.values()), b["with_costume"]["mass_kg"])
    close(sum(c["category"] == "motor" for c in b["components"]), 26)
    close(sum(p["dof"].values()), 26)
    allocations = {r["component"]: r["mass_kg"] for r in p["allocations"]}
    assert len(allocations) == len(p["allocations"])
    assert all(v > 0 for v in allocations.values())
    ceiling = p["mass_review_ceiling_kg"]
    close(sum(allocations.values()), ceiling)
    nonmotor = sum(allocations.values()) - allocations["actuators_including_integrated_drivers"]
    candidates = []
    for c in p["candidates"]:
        if "counts" in c:
            count = sum(c["counts"].values())
            motor_mass = sum(n * p["actuator_references"][k]["mass_kg"] for k, n in c["counts"].items())
        else:
            count = sum(r["count"] for r in c["target_groups"])
            motor_mass = sum(r["count"] * r["unit_mass_kg"] for r in c["target_groups"])
        close(count, 26)
        total = motor_mass + nonmotor
        candidates.append({"id": c["id"], "source_status": c["status"], "actuator_mass_kg": motor_mass,
                           "nonmotor_allocations_including_reserve_kg": nonmotor,
                           "budget_total_kg": total, "remaining_to_review_ceiling_kg": ceiling - total,
                           "arithmetic_within_ceiling": total <= ceiling,
                           "mass_exceedance_rejects_candidate": p["mass_review_ceiling_is_hard_limit"] and total > ceiling,
                           "hardware_feasible": None, "task_verified": False})
    d = p["dimensions"]
    height_sum = sum(d[k] for k in ("sole_to_ankle_mm", "thigh_length_mm", "shin_length_mm", "hip_to_shoulder_mm", "shoulder_to_crown_mm"))
    close(height_sum, d["height_with_crown_mm"])
    foot_outside = d["foot_width_mm"] + d["foot_center_spacing_mm"]
    assert foot_outside <= d["dressed_relaxed_width_range_mm"][1]
    s = p["screening"]
    g = s["gravity_m_s2"]
    payload = s["water_mass_kg"] + s["bottle_packaging_mass_kg"]
    com_cases, wrist_cases, jump_cases, land_cases = [], [], [], []
    energy_residuals = []
    for mass in s["robot_masses_kg"]:
        for x in s["bottle_forward_offsets_m"]:
            com = (mass * s["robot_com_forward_offset_m"] + payload * x) / (mass + payload)
            margin = s["foot_front_edge_m"] - com
            boundary = s["foot_front_edge_m"] - s["required_static_margin_m"]
            xmax = ((mass + payload) * boundary - mass * s["robot_com_forward_offset_m"]) / payload if payload > 0 else None
            close((mass + payload) * com, mass * s["robot_com_forward_offset_m"] + payload * x)
            com_cases.append({"robot_mass_kg": mass, "payload_x_m": x, "combined_com_x_m": com,
                              "front_margin_m": margin, "meets_assumed_static_margin": margin >= s["required_static_margin_m"],
                              "payload_x_max_for_margin_m": xmax, "payload_to_robot_mass_ratio": payload / mass})
        force = mass * g * s["hands_support_weight_fraction"] / 2
        close(2 * force + mass * g * (1 - s["hands_support_weight_fraction"]), mass * g)
        for lever in s["palm_wrist_levers_m"]:
            wrist_cases.append({"robot_mass_kg": mass, "per_hand_vertical_force_N": force,
                                "effective_perpendicular_lever_m": lever, "wrist_static_torque_Nm": force * lever})
        height = s["jump_com_rise_m"]
        v = math.sqrt(2 * g * height)
        for stroke in s["push_strokes_m"]:
            a = g * height / stroke
            t = v / a
            force = mass * (g + a)
            residual = force * stroke - mass * g * stroke - 0.5 * mass * v * v
            close(residual, 0)
            close((force - mass * g) * t, mass * v)
            energy_residuals.append(abs(residual))
            jump_cases.append({"robot_mass_kg": mass, "stroke_m": stroke, "apex_above_takeoff_m": height,
                               "takeoff_speed_m_s": v, "constant_acceleration_push_time_s": t,
                               "force_per_foot_equal_split_N": force / 2, "force_to_weight_ratio": force / (mass * g),
                               "ideal_contact_work_J": force * stroke, "ideal_average_contact_power_W": force * stroke / t})
        for distance in s["landing_stop_distances_m"]:
            force = mass * g * (1 + height / distance)
            residual = force * distance - mass * g * distance - mass * g * height
            close(residual, 0)
            energy_residuals.append(abs(residual))
            land_cases.append({"robot_mass_kg": mass, "stop_distance_m": distance,
                               "assumed_fall_com_height_m": height, "displacement_average_total_force_N": force,
                               "displacement_average_force_per_foot_equal_split_N": force / 2,
                               "peak_force_N": None})
    arm_cases = [{"payload_lever_m": r, "per_shoulder_static_torque_Nm":
                  payload * g * s["bottle_per_arm_load_fraction"] * r + s["arm_self_mass_kg"] * g * s["arm_com_lever_m"]}
                 for r in s["shoulder_bottle_levers_m"]]
    grip_cases = []
    for mu in s["grip_friction_coefficients"]:
        normal = s["grip_load_multiplier"] * payload * g / (2 * mu)
        close(2 * mu * normal, s["grip_load_multiplier"] * payload * g)
        grip_cases.append({"mu": mu, "normal_force_per_opposed_contact_N": normal, "contact_count": 2,
                           "note": "Two opposed resultant contacts; not motor torque and not each finger. Bottle crush and pad compliance unverified."})
    power_cases = []
    for density in s["assumed_pack_specific_energy_Wh_kg"]:
        energy = s["battery_mass_kg"] * density * s["usable_energy_fraction"]
        power = energy / (s["runtime_min"] / 60)
        close(power * s["runtime_min"] / 60, energy)
        power_cases.append({"assumed_pack_specific_energy_Wh_kg": density, "usable_energy_Wh": energy,
                            "maximum_average_pack_draw_for_runtime_W": power,
                            "note": "All motor/controller/converter/sensor losses must fit; no peak current or thermal approval."})
    speed_cases = []
    for t in s["joint_sweep_times_s"]:
        rpm = s["joint_sweep_deg"] / 360 / t * 60
        speed_cases.append({"sweep_deg": s["joint_sweep_deg"], "time_s": t, "minimum_peak_speed_from_average_rpm": rpm,
                            "XM430_W350_no_load_limit_exceeded_at_12V": rpm > p["actuator_references"]["XM430"]["no_load_rpm"],
                            "note": "If average exceeds no-load speed, impossible under stated direct-drive condition. Otherwise not a pass; acceleration and load require more."})
    max_major_mass = (ceiling - nonmotor - 8 * p["actuator_references"]["XC330"]["mass_kg"]) / 18
    full_voltage = s["li_ion_series_cells_example"] * s["cell_full_voltage_example_V"]
    return {"status": "analytical_budget_review_only", "engineering_validated": False,
            "baseline": {"mass_kg": b["with_costume"]["mass_kg"], "dimensions_xyz_mm": b["dimensions_mm"], "category_mass_kg": categories},
            "allocation_total_kg": sum(allocations.values()), "nonmotor_allocations_kg": nonmotor,
            "max_average_mass_for_18_major_actuators_with_8_XC330_kg": max_major_mass,
            "candidates": candidates,
            "proportion_arithmetic": {"straight_chain_height_mm": height_sum, "foot_outside_width_mm": foot_outside, "geometry_validated": False},
            "static_com_cases": com_cases, "shoulder_payload_cases": arm_cases, "wrist_support_cases": wrist_cases,
            "grip_cases": grip_cases, "vertical_jump_cases": jump_cases, "vertical_landing_cases": land_cases,
            "battery_runtime_cases": power_cases, "joint_speed_cases": speed_cases,
            "electrical_conflict": {"example_pack_full_voltage_V": full_voltage,
                                    "XC330_max_input_V": p["actuator_references"]["XC330"]["voltage_max_V"],
                                    "direct_connection_overvoltage": full_voltage > p["actuator_references"]["XC330"]["voltage_max_V"]},
            "numerical_checks": {"max_energy_residual_J": max(energy_residuals), "energy_case_count": len(energy_residuals)}}


def main():
    p = json.loads(INPUT.read_text(encoding="utf-8"))
    baseline_path = ROOT / p["baseline_file"]
    before = sha(baseline_path)
    result = evaluate(p, json.loads(baseline_path.read_text(encoding="utf-8")))
    result["provenance"] = {"input": INPUT.relative_to(ROOT).as_posix(), "input_sha256": sha(INPUT),
                            "script_sha256": sha(Path(__file__)), "baseline_sha256": before,
                            "source_checked_date": p["updated_at"]}
    # Regression anchors independent of rounded report formatting.
    close(result["candidates"][0]["budget_total_kg"], 3.06)
    close(result["candidates"][1]["budget_total_kg"], 3.708)
    close(result["candidates"][2]["budget_total_kg"], 2.95)
    close(result["static_com_cases"][0]["combined_com_x_m"], 0.058 / 2.58)
    assert result["joint_speed_cases"][0]["XM430_W350_no_load_limit_exceeded_at_12V"]
    assert result["electrical_conflict"]["direct_connection_overvoltage"]
    assert sha(baseline_path) == before
    OUTPUT.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    close(json.loads(OUTPUT.read_text(encoding="utf-8"))["allocation_total_kg"], 3)
    evidence = [INPUT.relative_to(ROOT).as_posix(), OUTPUT.relative_to(ROOT).as_posix(), "05_dynamics/system_reset_v14.py"]
    gates = {"stage": "v0.14", "scope": "budget and analytical identities only; no geometry/dynamics/manufacturing release",
             "checks": [{"name": n, "status": "passed", "evidence": evidence} for n in [
                 "v13 baseline category sum and 26 actuator count; baseline hash unchanged",
                 "v14 non-overlapping mass allocations and three candidate counts/sums",
                 "ideal height chain and foot width arithmetic, not physical packaging",
                 "static COM moment balance and assumed hand/contact force balance",
                 f"{len(result['vertical_jump_cases']) + len(result['vertical_landing_cases'])} vertical energy cases and {len(result['vertical_jump_cases'])} push-off impulse identities",
                 "runtime units, speed and overvoltage regression anchors; output reload"]],
             "pending": ["Match real components to task envelope; 3 kg is advisory, added mass requires system review", "Exact camera/mic/battery/driver configuration and measured masses",
                         "26 DOF sufficiency and full trajectory collision/IK with Wukong costume", "Torque-speed-duration-current-temperature and gear shock limits",
                         "Manufacturable short-load-path ankle/wrist and support bearings", "Contact friction, bottle grip and handover disturbances",
                         "Fullbody dynamics, three-direction flips on four surfaces and physical safety gates", "Final cost and manufacturing release"]}
    GATE.write_text(json.dumps(gates, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"candidate_budgets": result["candidates"], "major_actuator_average_ceiling_g": result["max_average_mass_for_18_major_actuators_with_8_XC330_kg"] * 1000,
                      "numerical_checks": result["numerical_checks"], "status": result["status"]}, indent=2))


if __name__ == "__main__":
    main()

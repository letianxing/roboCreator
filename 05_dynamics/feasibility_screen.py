"""Stdlib-only analytical screening. Not full robot dynamics or a safety certificate."""
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def calculate(data):
    p = {x['parameter_id']: x['value'] for x in data['parameters']}
    g = 9.81
    assert len(p) == len(data['parameters']), 'Duplicate parameter IDs'
    assert all(v > 0 for v in p.values()), 'Inputs must be positive'
    hsum = sum(p[k] for k in ('sole_height','thigh','shin','torso_to_head_base','head_height'))
    assert math.isclose(hsum, p['height'], abs_tol=1e-9)
    allowance_mass = sum(data['mass_allowances_kg'].values())
    allowance_cost = sum(data['cost_allowances_cny_excluding_actuators_and_camera'].values())
    variants = []
    for dof in (21,24,26):
        motor_mass = dof * p['actuator_mass']
        motor_cost = dof * p['servo_reference_price']
        variants.append(dict(dof=dof, actuator_mass_kg=motor_mass,
            estimated_total_mass_kg=motor_mass+allowance_mass,
            reference_actuator_cost_cny=motor_cost,
            subtotal_without_camera_cny=motor_cost+allowance_cost,
            remaining_for_camera_cny=p['budget']-motor_cost-allowance_cost))
    m = variants[1]['estimated_total_mass_kg']
    payload = p['bottle_mass']
    arm_torque = payload*g*p['bottle_lever']/2 + p['arm_mass_each']*g*p['arm_com_horizontal']
    # Two opposed hands, each supplies normal force N; 2*mu*N >= payload*g.
    grip_normal = p['grip_force_factor']*payload*g/(2*p['grip_mu'])
    reach = []
    l1, l2 = p['upper_arm'], p['forearm']+p['grip_extension']
    for x in (0.08,0.15):
        dz = p['bottle_grasp_height']-p['shoulder_height']
        d = math.hypot(x,dz)
        feasible = abs(l1-l2) <= d <= l1+l2
        bend = math.degrees(math.acos(max(-1,min(1,(d*d-l1*l1-l2*l2)/(2*l1*l2))))) if feasible else None
        reach.append(dict(target_x_m=x,target_z_m=p['bottle_grasp_height'],distance_m=d,
            planar_position_reachable=feasible,elbow_bend_deg=bend,
            caveat='ignores lateral offset, hand orientation, joint limits and collisions'))
    loadcases = []
    for supports in (2,1):
        loadcases.append(dict(supporting_limbs=supports,
            equivalent_gravity_joint_torque_Nm=m*g*p['joint_force_lever']/supports))
    landing = []
    for descent in (0.05,0.10,0.15):
        for s in (0.01,0.03,0.05):
            factor = 1+descent/s
            landing.append(dict(com_descent_m=descent,braking_m=s,
                force_over_weight=factor,total_equivalent_force_N=m*g*factor,
                illustrative_two_leg_joint_torque_Nm=m*g*factor*p['joint_force_lever']/2))
    takeoff=[]
    for h in (0.05,0.10,0.15):
        vz=math.sqrt(2*g*h)
        flight=2*vz/g
        takeoff.append(dict(com_apex_gain_m=h,vz_mps=vz,flight_s_same_com_height=flight,
            mean_body_rotation_rad_s_if_full_turn_in_flight=2*math.pi/flight))
    speed_case = math.pi/2 / 0.15
    battery=[dict(mean_battery_power_W=w,nominal_energy_Wh=w*p['runtime']/60/0.8,
                  caveat='80 percent usable energy assumed; power is sensitivity input') for w in (30,60,90)]
    return dict(status='analytical_screen_only', geometry_sum_m=hsum,variants=variants,
        bottle=dict(assumed_mass_kg=payload,per_arm_static_torque_Nm=arm_torque,
            per_hand_normal_force_with_factor_N=grip_normal,
            com_shift_due_to_bottle_only_m=payload*p['bottle_lever']/(m+payload)),
        reach=reach,quasistatic_loadcases=loadcases,landing=landing,takeoff=takeoff,
        joint_speed_example=dict(move_rad=math.pi/2,duration_s=0.15,mean_rad_s=speed_case,
            note='illustrative trajectory, not a necessary flip condition; body rotation is not joint speed'),
        battery=battery,
        fault_supply_screen=dict(twelve_st3215_stall_current_A=12*2.7,
            note='fault scenario sum, not normal current or a battery specification'),
        ak40_v3_leg_only=dict(count=12,mass_kg=12*.190,price_usd=12*135.90,
            budget_only_break_even_cny_per_usd=p['budget']/(12*135.90),
            note='no live FX used; excludes all other hardware, shipping and tax'))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=ROOT/'05_dynamics'/'screening_results_v01.json')
    args=parser.parse_args()
    source=ROOT/'03_system'/'screening_inputs_v01.json'
    raw=source.read_bytes()
    result=calculate(json.loads(raw))
    result['input_sha256']=hashlib.sha256(raw).hexdigest()
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':
    main()

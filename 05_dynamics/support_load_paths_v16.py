"""Analytical load paths for the remote-roll linkage and gauntlet support pad."""
import json,math,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];PFILE=ROOT/'03_system/support_mechanisms_v16.json';OUT=ROOT/'05_dynamics/support_load_paths_v16.json'
p=json.loads(PFILE.read_text(encoding='utf-8'));a=p['ankle'];h=p['forearm_support'];g=9.81
link=[];r=a['crank_radius_mm']/1000;d=a['rod_root_diameter_mm']/1000;L=a['rod_nominal_length_mm']/1000;E=a['load_cases']['steel_E_GPa']*1e9;I=math.pi*d**4/64;Pcr=math.pi**2*E*I/(a['load_cases']['rod_effective_length_factor']*L)**2
for q in range(a['roll_range_deg'][0],a['roll_range_deg'][1]+1):
    lever=r*math.cos(math.radians(q))
    for tau in a['load_cases']['roll_torque_Nm']:
        F=tau/lever;link.append({'roll_deg':q,'torque_Nm':tau,'effective_lever_m':lever,'rod_axial_force_N':F,'ideal_Euler_buckling_load_N':Pcr,'ideal_buckling_ratio':Pcr/F})
contacts=[]
for mass in h['robot_mass_kg']:
    for mult in h['load_multipliers']:
        F=mass*g*h['two_hand_weight_fraction']/2*mult
        for k in h['pad_stiffness_each_N_mm']:
            gap=h['gauntlet_pad_lead_mm'];threshold=k*gap
            if F<=threshold:Fc,Fp,delta=F,0,F/k
            else:delta=(F+k*gap)/(2*k);Fc=k*delta;Fp=k*(delta-gap)
            contacts.append({'robot_mass_kg':mass,'load_multiplier':mult,'total_per_arm_vertical_N':F,'pad_stiffness_each_N_mm':k,'gauntlet_force_N':Fc,'palm_force_N':Fp,'gauntlet_displacement_mm':delta,'wrist_torque_from_palm_Nm':Fp*h['palm_contact_center_x_mm']/1000,'force_balance_residual_N':Fc+Fp-F})
Fmax=max(x['gauntlet_force_N'] for x in contacts);b,t=h['bracket_cross_section_mm'];Lb=h['bracket_effective_length_mm'];Ibr=b*t**3/12
stress=Fmax*Lb*(t/2)/Ibr;disp=Fmax*Lb**3/(3*h['aluminum_E_GPa']*1000*Ibr)
cad=json.loads((ROOT/'09_cad/support_mechanisms_v16/mechanism_results.json').read_text())
budget=p['global_budget_reference'];struct_pool=sum(budget[k] for k in ['load_frames_and_external_transmissions_kg','bearings_shafts_output_supports_kg','fasteners_inserts_wiring_connectors_kg'])
ankle_nonmotor_pair=2*cad['neutral_ankle_structure_mass_kg'];ankle_motor_pair=2*(.082+.190)
out={'version':'v0.16','input_sha256':hashlib.sha256(PFILE.read_bytes()).hexdigest(),'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'ankle_linkage':{'rows':link,'ideal_rod_buckling_load_N':Pcr,'max_rod_force_N':max(x['rod_axial_force_N'] for x in link),'minimum_ideal_buckling_ratio':min(x['ideal_buckling_ratio'] for x in link),'external_crank_support_required':True,'note':'1:1 equal-radius ideal parallel crank. Joint play, misalignment, crank bending, pin/rod-end ratings and motor torque-speed remain absent.'},'gauntlet_contact':{'rows':contacts,'primary_pad_max_force_N':Fmax,'bracket_nominal_bending_stress_MPa':stress,'bracket_tip_deflection_mm':disp,'note':'Two equal linear pad stiffnesses with a height lead; floor and pad properties unmeasured. Stress is a nominal prismatic cantilever screen without holes/connection/contact concentration; no safety factor.'},'mass_feedback':{'two_ankle_motor_mass_kg':ankle_motor_pair,'two_ankle_nonmotor_geometry_kg':ankle_nonmotor_pair,'known_two_ankles_total_before_unmodeled_kg':ankle_motor_pair+ankle_nonmotor_pair,'global_structure_bearing_fastener_pool_kg':struct_pool,'nonmotor_geometry_fraction_of_global_pool':ankle_nonmotor_pair/struct_pool,'unmodeled_parts_remain':cad['unmodeled_mass'],'note':'Only nonmotor geometry is compared to the nonmotor pool. Motors remain in the separate actuator budget.'},'task_verified':False,'limitations':['No complete get-up path','No roll inverse dynamics','No bearing/fastener selection','No fatigue/impact/thermal validation','No hard-surface or flip result']}
assert max(abs(x['force_balance_residual_N']) for x in contacts)<1e-10
OUT.write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8');print(json.dumps(out['mass_feedback'],indent=2));print('max rod N',out['ankle_linkage']['max_rod_force_N'],'Pcr',Pcr,'pad max N',Fmax,'stress MPa',stress)

import json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'05_dynamics/fullbody_v13';OUT.mkdir(exist_ok=True)
old=json.loads((ROOT/'09_cad/fullbody_v12/wholebody_results.json').read_text())['results']
new=json.loads((ROOT/'09_cad/fullbody_v13/wholebody_results.json').read_text())['results']
old={p['pose']:p for p in old};new={p['pose']:p for p in new}
def moment(p,j):return next(x['quasistatic_required_Nm'] for x in p['joint_loads'] if x['joint']==j)
stroke=json.loads((ROOT/'05_dynamics/fullbody_v12/task_loads.json').read_text())['reference_com_stroke_m']
rows=[]
for key,data in [('v12',old),('v13',new)]:
    M=data['standing']['with_costume']['mass_kg'];factor=1+.1/stroke
    peak=max(abs(j['quasistatic_required_Nm']) for j in data['pickup_contact']['joint_loads'])
    rows.append({'version':key,'mass_kg':M,'handover_hip_roll_Nm':abs(moment(data['handover'],'R_hip_1')),
        'handover_peak_Nm':max(abs(j['quasistatic_required_Nm']) for j in data['handover']['joint_loads']),
        'shared_jump_com_stroke_m':stroke,'shared_jump_height_m':.1,
        'effective_vertical_force_each_foot_N':M*9.81*factor/2,
        'frozen_posture_peak_joint_screen_Nm':peak*factor})
result={'status':'architecture_comparison_only','results':rows,
    'same_target_comparisons':[{ 'pose':p,'old_hardware_pairs':len(old[p]['hardware_interferences']),
        'new_hardware_pairs':len(new[p]['hardware_interferences']),'old_ik':old[p]['ik']['arm_pose_pass'],'new_ik':new[p]['ik']['arm_pose_pass']}
        for p in ['pickup_contact','pickup_lift','handover','getup_intermediate']],
    'notes':['Standing arm posture changed to ready stance, so standing max width/torque is not an isolated geometry comparison',
        'Foot outer width decreases from 350 to 290 mm independently of arm pose',
        'Original targets retained; 150mm-forward pickup alternatives are additional, not replacement successes',
        'Shared takeoff stroke prevents attributing a changed pose stroke to actuator improvement',
        'Frozen acceleration loads are not inverse dynamics or a motor operating envelope']}
(OUT/'architecture_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))

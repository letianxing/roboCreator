"""Ankle-roll/foot envelope and static support screening; not dynamics or CAD."""
import json, math
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

def box_overlap(a,b,pad=.001):
    amin=np.array(a['center'])-np.array(a['half'])-pad
    amax=np.array(a['center'])+np.array(a['half'])+pad
    bmin=np.array(b['center'])-np.array(b['half'])-pad
    bmax=np.array(b['center'])+np.array(b['half'])+pad
    return bool(np.all(amin<=bmax) and np.all(bmin<=amax))

def evaluate(name, hip_y, ankle_y, foot_l, foot_w, ankle_module_mm, sole_mm):
    # ankle center is at z=sole; roll module is stacked laterally in the foot.
    z=sole_mm/1000+ankle_module_mm[2]/2000
    boxes=[
      {'name':'ankle_pitch','center':[0,ankle_y,z],'half':np.array(ankle_module_mm)/2000},
      {'name':'ankle_roll','center':[0,ankle_y+ankle_module_mm[1]/1000/2,z],'half':np.array(ankle_module_mm)/2000},
      {'name':'foot_shell','center':[0,ankle_y,sole_mm/2000],'half':np.array([foot_l/2,foot_w/2,sole_mm/2000])},
    ]
    pairs=[[a['name'],b['name']] for i,a in enumerate(boxes) for b in boxes[i+1:] if box_overlap(a,b)]
    penetrations=[]
    for i,a in enumerate(boxes):
        for b in boxes[i+1:]:
            depths=np.minimum(np.array(a['center'])+a['half'],np.array(b['center'])+b['half'])-np.maximum(np.array(a['center'])-a['half'],np.array(b['center'])-b['half'])
            if np.all(depths>1e-9):penetrations.append([a['name'],b['name']])
    # Simplified rectangular support polygon around both feet; COM point is centered.
    support_half_y=abs(ankle_y)+foot_w/2
    support_half_x=foot_l/2
    return dict(name=name,hip_y_m=hip_y,ankle_y_m=ankle_y,foot_length_m=foot_l,
      foot_width_m=foot_w,outer_width_m=2*(abs(ankle_y)+foot_w/2),
      ankle_module_mm=ankle_module_mm,sole_mm=sole_mm,proxy_boxes=boxes,
      box_clearance_violations=pairs,strict_penetrations=penetrations,dual_support_margin_m=min(support_half_x,support_half_y),
      notes=['ankle modules use supplied cuboid dimensions only',
        'foot support is rectangular and assumes full flat contact',
        'does not include motor shaft, brackets, bearings, fasteners, wiring or shoe shell'])

def main():
    configs=[
      evaluate('wide_ak40_ankle',.130,.130,.120,.055,[53,53,40.2],3),
      evaluate('compact_sts_ankle',.095,.095,.105,.050,[45.2,24.7,35],3),
      evaluate('compact_sts_ankle_narrow',.080,.080,.100,.045,[45.2,24.7,35],3),
    ]
    # With a centered COM, report sole-ground roll boundary only; not dynamic.
    for c in configs:
        c['single_support_margin_com_at_robot_center_m']=min(c['foot_length_m']/2,c['foot_width_m']/2-abs(c['ankle_y_m']))
        c['single_support_margin_com_over_foot_center_m']=min(c['foot_length_m']/2,c['foot_width_m']/2)
        c['height_ankle_module_top_m']=c['sole_mm']/1000+c['ankle_module_mm'][2]/1000
    result={'status':'v06_unit_and_support_audit_corrected_not_manufacturing','configs':configs,
      'decision_rule':'candidate requires no proxy box conflict; width is reported, not optimized',
      'limitations':['Ankle roll axis and module orientation are idealized',
        'No leg IK, torque, impact, friction, COM shift or swept collision',
        'Foot width/length are packaging dimensions, not traction dimensions',
        'A conflict may be due to conservative overlap, no conflict is not proof of clearance',
        'Hard-surface landing and aerial dynamics remain unresolved']}
    path=ROOT/'10_assembly'/'ankle_foot_results_v06_corrected.json'
    path.write_text(json.dumps(result,indent=2,default=lambda x: x.tolist() if isinstance(x,np.ndarray) else x)+'\n')
    for c in configs:
        print(c['name'],'width_mm',round(c['outer_width_m']*1000,1),'strict_penetrations',c['strict_penetrations'],
              'dual_margin_mm',round(c['dual_support_margin_m']*1000,1))

if __name__=='__main__': main()

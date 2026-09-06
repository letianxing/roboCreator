"""Analytic cuff/knee contact continuation with fixed support points."""
import json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];d=json.loads((ROOT/'04_kinematics/recovery_v19/phase_results.json').read_text())
start=d['phase_samples'][0];xk=start['knee_contact_x_mm'];rows=[]
def residual(h,beta):
    xhip=xk+math.sqrt(max(0,90**2-(h-32)**2));a=math.radians(beta)
    shoulder_x=xhip+80*math.sin(a);shoulder_z=h+80*math.cos(a)
    distance_xy=math.hypot(150-shoulder_x,125-75)
    # Flat forearm: cuff->elbow projected distance=85-12=73 mm, elbow height=25 mm.
    return (distance_xy-73)**2+(shoulder_z-25)**2-80**2
for h in [start['body_height_mm'],70,75,80,85,90,95,100,105,110,118,122]:
    brackets=[]
    for beta in range(60,100):
        if residual(h,beta)*residual(h,beta+1)<=0:brackets.append((float(beta),float(beta+1)))
    roots=[{'body_pitch_deg':60.0,'squared_length_residual_mm2':residual(h,60)}] if abs(residual(h,60))<.01 else []
    for lo,hi in brackets:
        for _ in range(45):
            mid=(lo+hi)/2
            if residual(h,lo)*residual(h,mid)<=0:hi=mid
            else:lo=mid
        beta=(lo+hi)/2;roots.append({'body_pitch_deg':beta,'squared_length_residual_mm2':residual(h,beta)})
    rows.append({'body_height_mm':h,'roots':roots,'geometry_only':True})
out={'status':'analytic_candidate_family_not_trajectory','rows':rows,
 'conditions':['Knee contact points fixed; radius32mm','Current arm lengths80/85mm and cuff offset12/25mm','Cuff normal vertical, elbow/pad plane geometry','Body pitch limited to60..100deg'],
 'limitations':['Root alone does not guarantee shoulder joint limits, CAD clearance or contact dynamics','Continuation establishes candidate pitch values for future IK, not a released robot motion']}
(ROOT/'04_kinematics/recovery_v19/cuff_continuation.json').write_text(json.dumps(out,indent=2)+'\n')
print([(round(r['body_height_mm'],2),[round(x['body_pitch_deg'],3) for x in r['roots']]) for r in rows])

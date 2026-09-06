"""Positive-control FK/IK and smaller lateral steps; preserve original failed search."""
import json
import numpy as np
import asymmetric_search_v20 as s

q=np.deg2rad(s.e['left_leg_pose']['q_deg'])
P,R,_=s.fk(q)
expected=np.array(s.e['left_leg_pose']['foot_translation_mm'])/1000
assert np.linalg.norm(P-expected)<1e-10
assert np.linalg.norm(R-np.eye(3))<1e-10
control=s.solve(expected)
assert control['ik_found']
rows=[s.solve(np.array([x,y,2])/1000) for x in (20,40) for y in (0,10,20)]
out={'FK_positive_control_error_mm':float(np.linalg.norm(P-expected)*1000),
     'IK_positive_control':control,'additional_cases':rows,
     'scope':'Numerical pose search only; failure is not proof of global infeasibility. No additional case has been CAD collision checked.'}
(s.OUT/'ik_audit.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
print('Control passed. Additional IK found:',sum(r['ik_found'] for r in rows),'/',len(rows))
print([(r['pose']['foot_translation_mm'],r['position_error_mm'],r['orientation_error_deg'],r['ik_found']) for r in rows])
